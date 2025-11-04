// Copyright 2016 Open Source Robotics Foundation, Inc.
#ifndef SRC_CONTROL_INCLUDE_CONTROL_CONTROL_NODE_HPP_
#define SRC_CONTROL_INCLUDE_CONTROL_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
// #include "ros_msgs/msg/control_command.hpp"
#include "memory"
// #include "ros_msgs/msg/trajectory.hpp"
// #include "ros_msgs/msg/pose.hpp"
// #include "control/lat_controller.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
// #include "control_protos/controller_conf.pb.h"

namespace control {
class ControlNode : public rclcpp::Node {
 public:
  ControlNode();
  // void get_trajectory(ros_msgs::msg::Trajectory::SharedPtr msg);
  // void get_localization(ros_msgs::msg::Pose::SharedPtr msg);
  // void compute_lateral_command();
  void send_vehicle_command();

 private:
  size_t count_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr send_control_command_timer_;
  // rclcpp::Publisher<ros_msgs::msg::ControlCommand>::SharedPtr
  //   ego_vehicle_control_cmd_publisher_;
  // rclcpp::Subscription<ros_msgs::msg::Trajectory>::SharedPtr
  //   trajectory_subscriber_;
  // rclcpp::Subscription<ros_msgs::msg::Pose>::SharedPtr
  //   localization_subscriber_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr
      carla_vehicle_control_cmd_publisher_;

  // ros_msgs::msg::ControlCommand control_command_;
  // ros_msgs::msg::Trajectory trajectory_;
  // ros_msgs::msg::Pose pose_;
  carla_msgs::msg::CarlaEgoVehicleControl carla_vehicle_command_;
  // bool has_subscribed_trajectory_;
  // bool has_subscribed_pose_;

  // LatController lateral_controller_;

  // control::pb::ControllerConf controller_conf_;
};
}  // namespace control
#endif  // SRC_CONTROL_INCLUDE_CONTROL_CONTROL_NODE_HPP_

