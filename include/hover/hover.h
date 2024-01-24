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

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::shared_ptr<tf2_ros::Buffer> tf);

  void clearDisplay();
  void updateDisplay(project11::LatLongDegrees target);
  void updateDisplay(const std::string& map_frame);
  void sendDisplay(bool dim=false);
  void setParameters(float min_distance, float max_distance, float max_speed);

  double current_range_;
  project11::AngleRadiansZeroCentered current_bearing_;
  float current_target_speed_;
  float minimum_distance_ = 10.0; // meters
  float maximum_distance_ = 25.0; // meters
  geometry_msgs::Point target_;
  float maximum_speed_ = 2.0;    // m/s

private:

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // display
  geographic_visualization_msgs::GeoVizItem vis_display_;
  ros::Publisher display_pub_;

};


#endif
