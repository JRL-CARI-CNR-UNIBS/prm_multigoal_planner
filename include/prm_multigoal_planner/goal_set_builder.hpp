#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <moveit_msgs/msg/constraints.hpp>

#include <ik_solver_msgs/srv/get_ik_array.hpp>

namespace prm_multigoal_planner
{

/**
 * @brief Turns the (Cartesian XOR joint-space) elements of goal_constraints into a flat set of
 * candidate joint-space goal configurations.
 *
 * - Cartesian element (position_constraints + orientation_constraints): all the IK solutions returned
 *   by ik_solver's get_task_redundant_ik_array for the target pose.
 * - Joint-space element (joint_constraints): the configuration itself.
 *
 * No collision/bounds filtering is done here (the planner does it).
 *
 * Service calls use async_send_request() + future.wait_for(): the node must be spun by an executor
 * on another thread while this class is used.
 */
class GoalSetBuilder
{
public:
  GoalSetBuilder(rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client, rclcpp::Logger logger,
                 double service_timeout_s = 5.0);

  /**
   * @return false only on a hard error (malformed constraints, service unavailable); an element that
   * yields zero candidates (e.g. unreachable Cartesian target) is not an error.
   */
  bool buildGoalConfigurations(const std::vector<moveit_msgs::msg::Constraints>& goal_constraints,
                               const std::vector<std::string>& canonical_joint_names,
                               std::vector<Eigen::VectorXd>& goal_configurations, std::string& error);

private:
  bool solveCartesianGoal(const moveit_msgs::msg::Constraints& c, const std::vector<std::string>& canonical_joint_names,
                          std::vector<Eigen::VectorXd>& out_configs, std::string& error);

  rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client_;
  rclcpp::Logger logger_;
  double service_timeout_s_;
};

}  // namespace prm_multigoal_planner
