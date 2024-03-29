#include <hover/hover_plugin.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hover::HoverPlugin, project11_navigation::TaskToTwistWorkflow)

namespace hover
{

void HoverPlugin::configure(std::string name, project11_navigation::Context::Ptr context)
{
  context_ = context;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);
  Hover::initialize(nh, private_nh, context_->tfBuffer());
  private_nh.param("minimum_distance", minimum_distance_, minimum_distance_);
  private_nh.param("maximum_distance", maximum_distance_, maximum_distance_);
  private_nh.param("maximum_speed", maximum_speed_, maximum_speed_);
  clearDisplay();
  sendDisplay();
}

void HoverPlugin::setGoal(const project11_navigation::TaskPtr& input)
{
  if(current_task_ != input)
    task_update_time_ = ros::Time();
  current_task_ = input;
  updateTask();
}

void HoverPlugin::updateTask()
{
  if(current_task_)
  {
    if(current_task_->lastUpdateTime() != task_update_time_)
    {
      geometry_msgs::Point target;
      if(current_task_->message().poses.empty())
      {
        auto odom = context_->robot().odometry();
        target = odom.pose.pose.position;
        if(odom.header.frame_id.empty())
        {
          ROS_WARN_STREAM_THROTTLE(5.0,"Empty frame_id in odom message used by hover to set hover position");
          return;
        }
        else
          map_frame_ = odom.header.frame_id;
      }
      else
      {
        target = current_task_->message().poses.front().pose.position;
        if(current_task_->message().poses.front().header.frame_id.empty())
          ROS_WARN_STREAM("Empty frame_id in pose used by hover to set hover position");
        else
          map_frame_ = current_task_->message().poses.front().header.frame_id;
      }
      setTarget(target);

      task_update_time_ = current_task_->lastUpdateTime();
    }
  }
  else
  {
    clearDisplay();
    sendDisplay();
  }
  updateDisplayMarkers();
}

bool HoverPlugin::running()
{
  updateTask();
  return bool(current_task_);
}

bool HoverPlugin::getResult(geometry_msgs::TwistStamped& output)
{
  sendDisplay();
  if(running())
    return generateCommands(output, map_frame_);
  return false;
}

void HoverPlugin::updateDisplayMarkers()
{
  if(current_task_)
  {
    //current_task_->markerArray().markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.ns = current_task_->message().id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position = target_;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = .75;
    
    marker.scale.x = maximum_distance_;
    marker.scale.y = maximum_distance_;
    marker.lifetime = ros::Duration(2.0);
    // if(!marker.header.frame_id.empty())
    //   current_task_->markerArray().markers.push_back(marker);
  }
}

} // namespace hover
