#ifndef HOVER_HOVER_PLUGIN_H
#define HOVER_HOVER_PLUGIN_H

#include "hover.h"
#include <project11_navigation/interfaces/task_to_twist_workflow.h>

namespace hover
{

class HoverPlugin: public project11_navigation::TaskToTwistWorkflow, Hover
{
public:
  void configure(std::string name, project11_navigation::Context::Ptr context) override;
  void setGoal(const project11_navigation::Task::Ptr& input) override;
  bool running() override;
  bool getResult(geometry_msgs::TwistStamped& output) override;
private:
  void updateTask();
  project11_navigation::Context::Ptr context_;
  project11_navigation::Task::Ptr current_task_;
  ros::Time task_update_time_;
  std::string map_frame_;

};

} // namespace hover

#endif