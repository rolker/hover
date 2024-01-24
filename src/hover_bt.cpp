#include "hover/hover_bt.h"
#include "project11_navigation/task.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/buffer.h>
#include "project11/utils.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hover::HoverSettingsLoader>("HoverSettingsLoader");
  factory.registerNodeType<hover::HoverUpdate>("HoverUpdate");
  factory.registerNodeType<hover::HoverCommand>("HoverCommand");
}

namespace hover
{

HoverSettingsLoader::HoverSettingsLoader(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList HoverSettingsLoader::providedPorts()
{
  return {
    BT::OutputPort<double>("minimum_distance"),
    BT::OutputPort<double>("maximum_distance"),
    BT::OutputPort<double>("maximum_speed"),
    BT::OutputPort<double>("transit_distance")
  };
}

BT::NodeStatus HoverSettingsLoader::tick()
{
  ros::NodeHandle nh("~/hover");
  
  double minimum_distance = 1.0;
  nh.param("minimum_distance", minimum_distance, minimum_distance);
  setOutput("minimum_distance", minimum_distance);

  double maximum_distance = minimum_distance*2.0;
  nh.param("maximum_distance", maximum_distance, maximum_distance);
  setOutput("maximum_distance", maximum_distance);

  double maximum_speed = 1.0;
  nh.param("maximum_speed", maximum_speed, maximum_speed);
  setOutput("maximum_speed", maximum_speed);

  double transit_distance = maximum_distance;
  nh.param("transit_distance", transit_distance, transit_distance);
  setOutput("transit_distance", transit_distance);

  return BT::NodeStatus::SUCCESS;
}


HoverUpdate::HoverUpdate(const std::string& name, const BT::NodeConfig& config):
  BT::SyncActionNode(name, config)
{

}

BT::PortsList HoverUpdate::providedPorts()
{
  return {
    BT::InputPort<project11_navigation::TaskPtr>("current_task"),
    BT::InputPort<geometry_msgs::PoseStamped>("current_pose"),
    BT::OutputPort<geometry_msgs::PoseStamped>("goal_pose"),
    BT::InputPort<std::shared_ptr<visualization_msgs::MarkerArray> >("marker_array"),
    BT::InputPort<double>("maximum_distance")
  };
}

BT::NodeStatus HoverUpdate::tick()
{

  auto task_bb = getInput<project11_navigation::TaskPtr>("current_task");
  if(!task_bb)
  {
    ROS_WARN_STREAM("HoverUpdate node named " << name() <<  " is missing input [current_task]");
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    auto task = task_bb.value();
    if(!task)
    {
      ROS_WARN_STREAM("HoverUpdate node named " << name() <<  " [current_task] is null");
      return BT::NodeStatus::FAILURE;
    }
    if(task->message().type != "hover")
    {
      ROS_WARN_STREAM("HoverUpdate node named " << name() <<  " [current_task] is type " << task->message().type << " but expected 'hover'");
      return BT::NodeStatus::FAILURE;
    }

    auto current_pose = getInput<geometry_msgs::PoseStamped>("current_pose");
    if(!current_pose)
    {
      ROS_WARN_STREAM("HoverUpdate node named " << name() <<  " [current_pose] missing");
      return BT::NodeStatus::FAILURE;        
    }

    if(task->message().poses.empty())
    {
      auto message = task->message();
      message.poses.push_back(current_pose.value());
      task->update(message);
    }
    setOutput("goal_pose", task->message().poses.front());

    auto marker_array = getInput<std::shared_ptr<visualization_msgs::MarkerArray> >("marker_array");
    if(!marker_array)
    {
      ROS_WARN_STREAM("HoverUpdate node named " << name() <<  " [marker_array] missing");
      return BT::NodeStatus::FAILURE;        
    }
    if(!marker_array.value())
    {
      ROS_WARN_STREAM("HoverUpdate node named " << name() <<  " [marker_array] is null");
      return BT::NodeStatus::FAILURE;        
    }

    if(marker_array && marker_array.value())
    {
      const auto& target = task->message().poses.front();
      visualization_msgs::Marker marker;
      marker.header.frame_id = target.header.frame_id;
      marker.header.stamp = current_pose.value().header.stamp;
      marker.id = 0;
      marker.ns = task->message().id;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.pose.position =  target.pose.position;
      marker.pose.orientation.w = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = .75;
      auto max_distance = getInput<double>("maximum_distance");
      if(!max_distance)
      {
        ROS_WARN_STREAM("HoverUpdate node named " << name() <<  " [maximum_distance] missing");
        marker.scale.x =  1.0;
        marker.scale.y = 1.0;
      }
      else
      {
        marker.scale.x =  2.0*max_distance.value();
        marker.scale.y = 2.0*max_distance.value();
      }
      marker.scale.z = 0.01;
      marker.lifetime = ros::Duration(2.0);
      marker_array.value()->markers.push_back(marker);
    }


  }
  return BT::NodeStatus::SUCCESS;
}


HoverCommand::HoverCommand(const std::string& name, const BT::NodeConfig& config):
  BT::StatefulActionNode(name, config)
{

}

BT::PortsList HoverCommand::providedPorts()
{
  return {
    BT::BidirectionalPort<geometry_msgs::TwistStamped>("command_velocity"),
    BT::InputPort<double>("minimum_distance"),
    BT::InputPort<double>("maximum_distance"),
    BT::InputPort<double>("maximum_speed"),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer"),
    BT::InputPort<geometry_msgs::PoseStamped>("goal_pose")
  };
}


BT::NodeStatus HoverCommand::onStart()
{
  return onRunning();
}

BT::NodeStatus HoverCommand::onRunning()
{
  auto cmd_vel_in = getInput<geometry_msgs::TwistStamped>("command_velocity");
  if(!cmd_vel_in)
  {
    ROS_WARN_STREAM("HoverCommand named " << name() << " missing [command_velocity]");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_pose = getInput<geometry_msgs::PoseStamped>("goal_pose");
  if(!goal_pose)
  {
    ROS_WARN_STREAM("HoverCommand named " << name() << " missing [goal_pose]");
    return BT::NodeStatus::FAILURE;
  }

  auto tf_buffer = getInput<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer");
  if(!tf_buffer)
  {
    ROS_WARN_STREAM("HoverCommand named " << name() << " missing [tf_buffer]");
    return BT::NodeStatus::FAILURE;
  }

  auto minimum_distance = getInput<double>("minimum_distance");
  if(!minimum_distance)
  {
    ROS_WARN_STREAM("HoverCommand named " << name() << " missing [minimum_distance]");
    return BT::NodeStatus::FAILURE;
  }

  auto maximum_distance = getInput<double>("maximum_distance");
  if(!maximum_distance)
  {
    ROS_WARN_STREAM("HoverCommand named " << name() << " missing [maximum_distance]");
    return BT::NodeStatus::FAILURE;
  }

  auto maximum_speed = getInput<double>("maximum_speed");
  if(!maximum_speed)
  {
    ROS_WARN_STREAM("HoverCommand named " << name() << " missing [maximum_speed]");
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    auto t =  tf_buffer.value()->lookupTransform(cmd_vel_in.value().header.frame_id, goal_pose.value().header.frame_id, ros::Time(0));
        
    geometry_msgs::Point target_local;
    tf2::doTransform(goal_pose.value().pose.position, target_local,t);

    auto current_range = sqrt(target_local.x*target_local.x+target_local.y*target_local.y);
    project11::AngleRadiansZeroCentered current_bearing = atan2(target_local.y, target_local.x);

    if(maximum_speed.value() <= 0.0)
      ROS_WARN_STREAM_THROTTLE(1.0,"Hover maximum speed set to: " << maximum_speed.value());
        
    auto current_target_speed = 0.0;
    if (current_range >= maximum_distance.value())
      current_target_speed = maximum_speed.value();
    else if (current_range > minimum_distance.value())
    {
      float p = (current_range-minimum_distance.value())/(maximum_distance.value()-minimum_distance.value());
      current_target_speed = p*maximum_speed.value();
    }
    else
    {
      // in the zero speed zone so don't turn towards target if it's behind us
      if(abs(current_bearing.value()) > M_PI/2.0)
        current_bearing += M_PI;
    }

    ROS_DEBUG_STREAM("bearing: " << current_bearing.value() << " range: " << current_range << " target speed: " << current_target_speed << " max speed: " << maximum_speed.value());

    geometry_msgs::TwistStamped cmd_vel = cmd_vel_in.value();

    cmd_vel.twist.angular.z = current_bearing.value();
    cmd_vel.twist.linear.x = current_target_speed;
    setOutput("command_velocity", cmd_vel);
    return BT::NodeStatus::RUNNING;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(1.0,"HoverCommand::tick() " << ex.what() << " cmd_vel frame: " << cmd_vel_in.value().header.frame_id << " target_frame: " << goal_pose.value().header.frame_id);
  }
  return BT::NodeStatus::FAILURE;
}

void HoverCommand::onHalted()
{
  
}

} // namespace hover
