// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

#include <ompl/util/Console.h>

#include <prm_multigoal_planner/prm_multigoal_planner_action_server.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // OMPL prints an INFO line per query/setup: keep only warnings and errors.
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

  auto node = std::make_shared<rclcpp::Node>("prm_multigoal_planner");
  auto action_server = std::make_shared<prm_multigoal_planner::PrmMultigoalPlannerActionServer>(node);

  // Humble's rclcpp_action server is not safe under a MultiThreadedExecutor (fixed in later releases).
  // All blocking work runs in the detached execute() threads, so one executor thread is enough there.
#if RCLCPP_VERSION_MAJOR >= 28
  rclcpp::executors::MultiThreadedExecutor executor;
#else
  rclcpp::executors::SingleThreadedExecutor executor;
#endif
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "prm_multigoal_planner node spinning.");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
