#include "ros/ros.h"

#include "hover/hoverAction.h"
#include "actionlib/server/simple_action_server.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "dynamic_reconfigure/server.h"
#include "hover/hoverConfig.h"
#include "project11/utils.h"
#include "project11/tf2_utils.h"

namespace p11 = project11;

class Hover
{
public:
  Hover(std::string const &name):
    m_action_server(m_node_handle, name, false)
    {
        m_cmd_vel_pub = m_node_handle.advertise<geometry_msgs::TwistStamped>("cmd_vel",1);
        m_display_pub = m_node_handle.advertise<geographic_visualization_msgs::GeoVizItem>("project11/display",5);
        
        m_enabled = true;
        m_enable_sub = m_node_handle.subscribe<std_msgs::Bool>("enable", 10, [&](const std_msgs::BoolConstPtr& msg){this->m_enabled = msg->data; this->sendDisplay();});
        
        ros::NodeHandle nh_private("~");
    
        nh_private.param<std::string>("map_frame", m_map_frame, "map");
        nh_private.param<std::string>("base_frame", m_base_frame, "base_link");

        m_config_server.setCallback(std::bind(&Hover::reconfigureCallback, this,  std::placeholders::_1, std::placeholders::_2));
          
        m_action_server.registerGoalCallback(std::bind(&Hover::goalCallback, this));
        m_action_server.registerPreemptCallback(std::bind(&Hover::preemptCallback, this));
        m_action_server.start();
        
        m_timer = m_node_handle.createTimer(ros::Duration(0.1), std::bind(&Hover::timerCallback, this, std::placeholders::_1));
    }
    
    ~Hover()
    {
    }
    
    void goalCallback()
    {
        auto goal = m_action_server.acceptNewGoal();
        p11::fromMsg(goal->target, m_target);
        m_target_map = m_transforms.wgs84_to_map(goal->target,m_map_frame);
        sendDisplay();
    }
    
    void sendDisplay()
    {
        geographic_visualization_msgs::GeoVizItem vizItem;
        vizItem.id = "hover";
        if(m_action_server.isActive())
        {
            geographic_visualization_msgs::GeoVizPointList plist;
            geographic_msgs::GeoPoint gp;
            p11::toMsg(m_target, gp);
            plist.points.push_back(gp);
            plist.size = 10;
            if(m_enabled)
            {
              plist.color.r = .5;
              plist.color.g = .8;
              plist.color.b = .5;
              plist.color.a = 1.0;
            }
            else
            {
              plist.color.r = .2;
              plist.color.g = .3;
              plist.color.b = .2;
              plist.color.a = .5;
            }  
            vizItem.point_groups.push_back(plist);
            
            geographic_visualization_msgs::GeoVizPolygon polygon;
            // exterior ring is counter-clockwise
            for (double azimuth = 360.0; azimuth >= 0.0;  azimuth -= 10.0)
            {
                p11::LatLongDegrees p = p11::WGS84::direct(m_target, p11::AngleDegrees(azimuth), m_maximum_distance);
                geographic_msgs::GeoPoint gp;
                p11::toMsg(p, gp);
                polygon.outer.points.push_back(gp);
            }
            
            geographic_visualization_msgs::GeoVizSimplePolygon inner;
            // inner ring is clockwise
            for (double azimuth = 0.0; azimuth <= 360.0;  azimuth += 10.0)
            {
                p11::LatLongDegrees p = p11::WGS84::direct(m_target, p11::AngleDegrees(azimuth), m_minimum_distance);
                geographic_msgs::GeoPoint gp;
                p11::toMsg(p, gp);
                inner.points.push_back(gp);
            }
            polygon.inner.push_back(inner);
            
            if(m_enabled)
            {
              polygon.fill_color.r = 0.0;
              polygon.fill_color.g = 1.0;
              polygon.fill_color.b = 0.0;
              polygon.fill_color.a = 0.5;
            }
            else
            {
                polygon.fill_color.r = 0.0;
                polygon.fill_color.g = 0.5;
                polygon.fill_color.b = 0.0;
                polygon.fill_color.a = 0.25;
            }
            
            polygon.edge_size = 2.0;
            
            if(m_enabled)
            {
              polygon.edge_color.r = 0.0;
              polygon.edge_color.g = 0.0;
              polygon.edge_color.b = 1.0;
              polygon.edge_color.a = 0.75;
            }
            else
            {
              polygon.edge_color.r = 0.0;
              polygon.edge_color.g = 0.0;
              polygon.edge_color.b = 0.5;
              polygon.edge_color.a = 0.375;
            }
            
            vizItem.polygons.push_back(polygon);
        }
        m_display_pub.publish(vizItem);
        m_lastDisplayTime = ros::Time::now();
    }
    
    void preemptCallback()
    {
        m_action_server.setPreempted();
        sendDisplay();
    }


    void timerCallback(const ros::TimerEvent event)
    {
        if(m_action_server.isActive() && m_enabled)
        {
          try
          {
            auto t = m_transforms().lookupTransform(m_base_frame, m_map_frame, ros::Time(0));
            
            geometry_msgs::Point target_local;
            tf2::doTransform(m_target_map,target_local,t);

            double range = sqrt(target_local.x*target_local.x+target_local.y*target_local.y);
            p11::AngleRadiansZeroCentered bearing(atan2(target_local.y, target_local.x));
            
            float target_speed = 0.0;
            if (range >= m_maximum_distance)
                target_speed = m_maximum_speed;
            else if (range > m_minimum_distance)
            {
                float p = (range-m_minimum_distance)/(m_maximum_distance-m_minimum_distance);
                target_speed = p*m_maximum_speed;
            }
              
            hover::hoverFeedback feedback;
            feedback.range = range;
            feedback.bearing = p11::AngleDegrees(bearing).value();
            feedback.speed = target_speed;
              
            m_action_server.publishFeedback(feedback);
              
            ros::Time now = ros::Time::now();
            
            geometry_msgs::TwistStamped cmd_vel;
            cmd_vel.header.stamp = now;
            cmd_vel.twist.angular.z = bearing.value();
            cmd_vel.twist.linear.x = target_speed;
            m_cmd_vel_pub.publish(cmd_vel);
          }
          catch (tf2::TransformException &ex)
          {
            ROS_WARN_STREAM("Hover: " << ex.what());
          }
        }
        if(ros::Time::now() - m_lastDisplayTime > ros::Duration(1))
          sendDisplay();
    }
    
    void reconfigureCallback(hover::hoverConfig &config, uint32_t level)
    {
        m_minimum_distance = config.minimum_distance;
        m_maximum_distance = config.maximum_distance;
        m_maximum_speed = config.maximum_speed;
        sendDisplay();
    }
    
private:
    ros::NodeHandle m_node_handle;
    actionlib::SimpleActionServer<hover::hoverAction> m_action_server;

    
    ros::Publisher m_cmd_vel_pub;
    ros::Publisher m_display_pub;
    
    dynamic_reconfigure::Server<hover::hoverConfig> m_config_server;

    // goal variables
    p11::LatLongDegrees m_target;
    geometry_msgs::Point m_target_map;
    
    float m_minimum_distance; // meters
    float m_maximum_distance; // meters
    float m_maximum_speed;    // m/s
    
    p11::Transformations m_transforms;
    
    std::string m_map_frame;
    std::string m_base_frame;
    
    ros::Timer m_timer;
    ros::Time m_lastDisplayTime;

    bool m_enabled;
    ros::Subscriber m_enable_sub;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover");

    Hover h("hover_action");
    
    ros::spin();
    
    return 0;
}
