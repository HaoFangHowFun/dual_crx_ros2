#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

namespace
{
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
constexpr double kPi = 3.141592653589793;

std::map<std::string, double> demo_target(const std::string & group)
{
  if (group == "left_arm") {
    return {
      {"left_J1", 0.35},
      {"left_J2", 0.0},
      {"left_J3", 0.0},
      {"left_J4", 0.0},
      {"left_J5", -0.35},
      {"left_J6", 0.0},
    };
  }

  if (group == "right_arm") {
    return {
      {"right_J1", kPi - 0.35},
      {"right_J2", 0.0},
      {"right_J3", 0.0},
      {"right_J4", 0.0},
      {"right_J5", -0.35},
      {"right_J6", kPi},
    };
  }

  if (group == "both_arms") {
    return {
      {"left_J1", 0.55},
      {"left_J2", 0.15},
      {"left_J3", 0.0},
      {"left_J4", 0.0},
      {"left_J5", -0.60},
      {"left_J6", 0.0},
      {"right_J1", kPi - 0.55},
      {"right_J2", 0.15},
      {"right_J3", 0.0},
      {"right_J4", 0.0},
      {"right_J5", -0.60},
      {"right_J6", kPi},
    };
  }

  return {};
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dual_crx_demo");
  const auto group = node->declare_parameter<std::string>("group", "left_arm");
  const auto execute = node->declare_parameter<bool>("execute", false);
  const auto planning_time = node->declare_parameter<double>("planning_time", 10.0);
  const auto velocity_scaling = node->declare_parameter<double>("velocity_scaling", 0.10);
  const auto acceleration_scaling =
    node->declare_parameter<double>("acceleration_scaling", 0.10);

  const auto target = demo_target(group);
  if (target.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Unsupported planning group '%s'. Choose left_arm, right_arm, or both_arms.",
      group.c_str());
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int result = 1;
  {
    MoveGroupInterface move_group(node, group);
    move_group.setPlanningTime(planning_time);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling);

    const auto current_state = move_group.getCurrentState(5.0);
    if (!current_state) {
      RCLCPP_ERROR(node->get_logger(), "No current robot state received within 5 seconds.");
    } else if (!move_group.setJointValueTarget(target)) {
      RCLCPP_ERROR(node->get_logger(), "The demo target for '%s' is invalid.", group.c_str());
    } else {
      move_group.setStartStateToCurrentState();
      RCLCPP_INFO(
        node->get_logger(),
        "Planning a collision-aware joint-space motion for '%s' (execute=%s).",
        group.c_str(), execute ? "true" : "false");

      MoveGroupInterface::Plan plan;
      const bool planned =
        move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

      if (!planned) {
        RCLCPP_ERROR(node->get_logger(), "Planning failed for '%s'.", group.c_str());
      } else if (!execute) {
        RCLCPP_INFO(
          node->get_logger(),
          "Planning succeeded for '%s'. Re-run with -p execute:=true to execute it.",
          group.c_str());
        result = 0;
      } else {
        RCLCPP_INFO(node->get_logger(), "Planning succeeded; executing '%s'.", group.c_str());
        const bool executed =
          move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (executed) {
          RCLCPP_INFO(node->get_logger(), "Execution succeeded for '%s'.", group.c_str());
          result = 0;
        } else {
          RCLCPP_ERROR(node->get_logger(), "Execution failed for '%s'.", group.c_str());
        }
      }
    }
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
