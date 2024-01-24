#include "hover/hover_action.h"
#include "std_msgs/Bool.h"

namespace p11 = project11;

HoverAction::HoverAction(std::string name):
  action_server_(ros::NodeHandle(), name, false), enabled_(true)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  initialize(nh, nh_private, transforms_());

  nh_private.param<std::string>("map_frame", map_frame_, "map");
  nh_private.param<std::string>("base_frame", base_frame_, "base_link");
  float update_rate;
  nh_private.param<float>("update_rate", update_rate , 10.0);

  config_server_.setCallback(std::bind(&HoverAction::reconfigureCallback, this,  std::placeholders::_1, std::placeholders::_2));


  // Subscribers
  enable_sub_ = nh.subscribe<std_msgs::Bool>("enable", 10, [&](const std_msgs::BoolConstPtr& msg)
      {
        enabled_ = msg->data;
        sendDisplay();
      }
    );

  // Publishers
  cmd_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel",1);

  // Action server callbacks
  action_server_.registerGoalCallback(
    boost::bind(&HoverAction::goalCallback, this));
  action_server_.registerPreemptCallback(
    boost::bind(
      &HoverAction::preemptCallback, this));
  action_server_.start();

  // Updater
  timer_ = nh.createTimer(ros::Duration(1.0/update_rate),
				 std::bind(&HoverAction::timerCallback, this, std::placeholders::_1));

}

HoverAction::~HoverAction()
{

}

void HoverAction::goalCallback()
{
  auto goal = action_server_.acceptNewGoal();
  p11::LatLongDegrees target;
  p11::fromMsg(goal->target, target);
  auto target_map = transforms_.wgs84_to_map(goal->target, map_frame_);
  setTarget(target_map);
  updateDisplay(target);
}

void HoverAction::preemptCallback()
{
  action_server_.setPreempted();
  clearDisplay();
  sendDisplay();
}



void HoverAction::reconfigureCallback(hover::hoverConfig &config, uint32_t level)
{
  setParameters(config.minimum_distance, config.maximum_distance, config.maximum_speed);
  sendDisplay();
}


void HoverAction::timerCallback(const ros::TimerEvent event)
{
  if(action_server_.isActive())
    if(enabled_)
    {
      geometry_msgs::TwistStamped ts;
      ts.header.frame_id = base_frame_;
      ts.header.stamp = event.current_real;
      if(generateCommands(ts, map_frame_))
        cmd_vel_pub_.publish(ts);

      hover::hoverFeedback feedback;
      feedback.range = current_range_;
      feedback.bearing = p11::AngleDegrees(current_bearing_).value();
      feedback.speed = current_target_speed_;
          
      action_server_.publishFeedback(feedback);
      sendDisplay();
    }
    else
      sendDisplay(false);
}
  


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hover");

  HoverAction h("hover_action");
  
  ros::spin();
  
  return 0;
}
