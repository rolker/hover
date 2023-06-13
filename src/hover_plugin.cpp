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
  Hover::initialize(nh, private_nh, &context_->tfBuffer());
  clearDisplay();
  sendDisplay();
}

void HoverPlugin::setGoal(const project11_navigation::Task::Ptr& input)
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
        auto odom = context_->getOdometry();
        target = odom.pose.pose.position;
        map_frame_ = odom.header.frame_id;
      }
      else
      {
        target = current_task_->message().poses.front().pose.position;
        map_frame_ = current_task_->message().poses.front().header.frame_id;
      }
      setTarget(target);
      updateDisplay(map_frame_);
      task_update_time_ = current_task_->lastUpdateTime();
    }
  }
  else
  {
    clearDisplay();
    sendDisplay();
  }
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

} // namespace hover
