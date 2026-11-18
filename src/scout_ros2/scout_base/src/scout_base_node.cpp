/*
 * scout_base_node.cpp
 *
 * Created on: Oct 15, 2021 16:20
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <memory>
#include <csignal>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "scout_base/scout_base_ros.hpp"

using namespace westonrobot;

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void RequestStop(int) {
  // Only set a signal-safe flag; CAN and ROS teardown run in the main thread.
  stop_requested = 1;
}
}  // namespace

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, RequestStop);
  std::signal(SIGTERM, RequestStop);

  auto robot = std::make_shared<ScoutBaseRos>("scout");
  int result = 0;
  try {
    if (!robot->Initialize()) {
      result = 1;
    } else if (!stop_requested) {
      std::cout << "Robot initialized, start running ..." << std::endl;
      robot->Run([] { return stop_requested == 0; });
    }
  } catch (const std::exception &exc) {
    std::cerr << "Scout base stopped: " << exc.what() << std::endl;
    result = 1;
  }
  robot->Stop();
  // Destroy publishers/subscriptions before invalidating the ROS context.
  robot.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
