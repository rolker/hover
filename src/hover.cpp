#include "ros/ros.h"

#include "project11/gz4d_geo.h"
#include "hover/hoverAction.h"
#include "actionlib/server/simple_action_server.h"
#include "marine_msgs/NavEulerStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPointStamped.h"

class Hover
{
public:
    Hover(std::string const &name):
        m_action_server(m_node_handle, name, false)
    {
        m_desired_heading_pub = m_node_handle.advertise<marine_msgs::NavEulerStamped>("/project11/desired_heading",1);
        m_desired_speed_pub = m_node_handle.advertise<geometry_msgs::TwistStamped>("/project11/desired_speed",1);
        
        m_position_sub = m_node_handle.subscribe("/position", 10, &Hover::positionCallback, this);

        m_action_server.registerGoalCallback(boost::bind(&Hover::goalCallback, this));
        m_action_server.registerPreemptCallback(boost::bind(&Hover::preemptCallback, this));
        m_action_server.start();
    }
    
    ~Hover()
    {
    }
    
    void goalCallback()
    {
        auto goal = m_action_server.acceptNewGoal();
        
        m_target[0] = goal->target.latitude;
        m_target[1] = goal->target.longitude;
        m_minimum_distance = goal->minimum_distance;
        m_maximum_distance = goal->maximum_distance;
        m_maximum_speed = goal->maximum_speed;
    }
    
    void preemptCallback()
    {
        m_action_server.setPreempted();
    }
    
    void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
    {
        if(m_action_server.isActive())
        {
            gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> vehicle_position(inmsg->position.latitude, inmsg->position.longitude,0.0);
            std::pair<double,double> azimuth_distance_to_target = gz4d::geo::WGS84::Ellipsoid::inverse(vehicle_position, m_target);
            
            float target_speed = 0.0;
            float range = azimuth_distance_to_target.second;
            if (range >= m_maximum_distance)
                target_speed = m_maximum_speed;
            else if (range > m_minimum_distance)
            {
                float p = (range-m_minimum_distance)/(m_maximum_distance-m_minimum_distance);
                target_speed = p*m_maximum_speed;
            }
            
            hover::hoverFeedback feedback;
            feedback.range = azimuth_distance_to_target.second;
            feedback.bearing = azimuth_distance_to_target.first;
            feedback.speed = target_speed;
            
            m_action_server.publishFeedback(feedback);
            
            ros::Time now = ros::Time::now();
            
            marine_msgs::NavEulerStamped desired_heading;
            desired_heading.header.stamp = now;
            desired_heading.orientation.heading = azimuth_distance_to_target.first;
            m_desired_heading_pub.publish(desired_heading);
            
            geometry_msgs::TwistStamped desired_speed;
            desired_speed.header.stamp = now;
            desired_speed.twist.linear.x = target_speed;
            m_desired_speed_pub.publish(desired_speed);
        }
    }
    
private:
    ros::NodeHandle m_node_handle;
    actionlib::SimpleActionServer<hover::hoverAction> m_action_server;

    
    ros::Publisher m_desired_speed_pub;
    ros::Publisher m_desired_heading_pub;
    ros::Subscriber m_position_sub;

    // goal variables
    gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> m_target;
    float m_minimum_distance; // meters
    float m_maximum_distance; // meters
    float m_maximum_speed;    // m/s
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover");

    Hover h("hover_action");
    
    ros::spin();
    
    return 0;
}
