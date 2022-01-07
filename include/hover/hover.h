#ifndef HOVER_HOVER_H
#define HOVER_HOVER_H

#include "ros/ros.h"

#include "geographic_visualization_msgs/GeoVizItem.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "project11/utils.h"
#include "project11/tf2_utils.h"

class Hover
{
public:
  Hover();
  ~Hover();

protected:
  void setTarget(const geometry_msgs::Point& target);
  bool generateCommands(geometry_msgs::TwistStamped &cmd_vel, const std::string& map_frame);

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const tf2_ros::Buffer *tf);

  void clearDisplay();
  void updateDisplay(project11::LatLongDegrees target);
  void sendDisplay(bool dim=false);
  void setParameters(float min_distance, float max_distance, float max_speed);

  double current_range_;
  project11::AngleRadiansZeroCentered current_bearing_;
  float current_target_speed_;
private:

  geometry_msgs::Point target_;

  float minimum_distance_; // meters
  float maximum_distance_; // meters
  float maximum_speed_;    // m/s

  const tf2_ros::Buffer *tf_buffer_ = nullptr;

  // display
  geographic_visualization_msgs::GeoVizItem vis_display_;
  ros::Publisher display_pub_;

};


#endif
