#include "hover/hover.h"


namespace p11 = project11;

Hover::Hover()
{
  vis_display_.id = "hover";
}

Hover::~Hover()
{

}

void Hover::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const tf2_ros::Buffer *tf)
{
  tf_buffer_ = tf;
  display_pub_ = nh.advertise<geographic_visualization_msgs::GeoVizItem>
    ("project11/display",5);

}


void Hover::setTarget(const geometry_msgs::Point& target)
{
  target_ = target;
}

void Hover::setParameters(float min_distance, float max_distance, float max_speed)
{
  minimum_distance_ = min_distance;
  maximum_distance_ = max_distance;
  maximum_speed_ = max_speed;
}

void Hover::clearDisplay()
{
  vis_display_.point_groups.clear();
  vis_display_.polygons.clear();
}

void Hover::updateDisplay(const std::string& map_frame)
{
  try
  {
    geometry_msgs::TransformStamped map_to_earth = tf_buffer_->lookupTransform("earth", map_frame, ros::Time(0));  
    geometry_msgs::Point ecef_point_msg;
    tf2::doTransform(target_, ecef_point_msg, map_to_earth);
    p11::ECEF ecef_point;
    p11::fromMsg(ecef_point_msg, ecef_point);
    p11::LatLongDegrees ll_point = ecef_point;
    updateDisplay(ll_point);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM("Unable to find transform to generate display: " << ex.what());
  }

}

void Hover::updateDisplay(p11::LatLongDegrees target)
{
  clearDisplay();

  geographic_visualization_msgs::GeoVizPointList plist;
  geographic_msgs::GeoPoint gp;
  p11::toMsg(target, gp);
  plist.points.push_back(gp);
  plist.size = 10;
  vis_display_.point_groups.push_back(plist);

  geographic_visualization_msgs::GeoVizPolygon polygon;
  // exterior ring is counter-clockwise
  for (double azimuth = 360.0; azimuth >= 0.0;  azimuth -= 10.0)
  {
    p11::LatLongDegrees p = p11::WGS84::direct(target, p11::AngleDegrees(azimuth), maximum_distance_);
    geographic_msgs::GeoPoint gp;
    p11::toMsg(p, gp);
    polygon.outer.points.push_back(gp);
  }
    
  geographic_visualization_msgs::GeoVizSimplePolygon inner;
  // inner ring is clockwise
  for (double azimuth = 0.0; azimuth <= 360.0;  azimuth += 10.0)
  {
    p11::LatLongDegrees p = p11::WGS84::direct(target, p11::AngleDegrees(azimuth), minimum_distance_);
    geographic_msgs::GeoPoint gp;
    p11::toMsg(p, gp);
    inner.points.push_back(gp);
  }
  polygon.inner.push_back(inner);
  polygon.edge_size = 2.0;
  vis_display_.polygons.push_back(polygon);
}

void Hover::sendDisplay(bool dim)
{
  double intensity = 1.0;
  if(dim)
    intensity = 0.5;

  for(auto& pl: vis_display_.point_groups)
  {
    pl.color.r = .5*intensity;
    pl.color.g = .8*intensity;
    pl.color.b = .5*intensity;
    pl.color.a = 1.0*intensity;
  }

  for(auto& polygon: vis_display_.polygons)
  {
    polygon.fill_color.r = 0.0*intensity;
    polygon.fill_color.g = 1.0*intensity;
    polygon.fill_color.b = 0.0*intensity;
    polygon.fill_color.a = 0.5*intensity;

    polygon.edge_color.r = 0.0*intensity;
    polygon.edge_color.g = 0.0*intensity;
    polygon.edge_color.b = 1.0*intensity;
    polygon.edge_color.a = 0.75*intensity;
  }
    
  display_pub_.publish(vis_display_);
}


bool Hover::generateCommands(geometry_msgs::TwistStamped &cmd_vel, const std::string& map_frame)
{
  try
  {
    auto t =  tf_buffer_->lookupTransform(cmd_vel.header.frame_id, map_frame, ros::Time(0));
        
    geometry_msgs::Point target_local;
    tf2::doTransform(target_, target_local,t);

    current_range_ = sqrt(target_local.x*target_local.x+target_local.y*target_local.y);
    current_bearing_ = atan2(target_local.y, target_local.x);

    //ROS_INFO_STREAM("bearing: " << current_bearing_.value());
        
    current_target_speed_ = 0.0;
    if (current_range_ >= maximum_distance_)
      current_target_speed_ = maximum_speed_;
    else if (current_range_ > minimum_distance_)
    {
      float p = (current_range_-minimum_distance_)/(maximum_distance_-minimum_distance_);
      current_target_speed_ = p*maximum_speed_;
    }
    else
    {
      // in the zero speed zone don't turn towards target if behind us
      if(abs(current_bearing_.value()) > M_PI/2.0)
        current_bearing_ += M_PI;
    }
          
    cmd_vel.twist.angular.z = current_bearing_.value();
    cmd_vel.twist.linear.x = current_target_speed_;
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(1.0,"Hover: " << ex.what() << " cmd_vel frame: " << cmd_vel.header.frame_id << " map_frame: " << map_frame);
  }
  return false;

}

