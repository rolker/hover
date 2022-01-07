#ifndef HOVER_HOVER_ACTION_H
#define HOVER_HOVER_ACTION_H

#include "hover/hover.h"
#include "hover/hoverAction.h"
#include "actionlib/server/simple_action_server.h"
#include "dynamic_reconfigure/server.h"
#include "hover/hoverConfig.h"

class HoverAction: public Hover
{
public:
  HoverAction(std::string name);
  ~HoverAction();

  void goalCallback();
  void preemptCallback();

private:
  void timerCallback(const ros::TimerEvent event);
  void reconfigureCallback(hover::hoverConfig &config, uint32_t level);


  actionlib::SimpleActionServer<hover::hoverAction> action_server_;

  // Pub
  ros::Publisher cmd_vel_pub_;

  dynamic_reconfigure::Server<hover::hoverConfig> config_server_;


  // Only send control commands when enabled
  bool enabled_;
  ros::Subscriber enable_sub_;

  project11::Transformations transforms_;
  ros::Timer timer_;

  // tf frames
  std::string map_frame_;
  std::string base_frame_;
};


#endif