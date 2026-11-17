// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0

#include <prm_multigoal_planner/goal_set_builder.hpp>
#include <prm_multigoal_planner/joint_order_utils.hpp>

#include <ik_solver_msgs/msg/ik_target.hpp>

namespace prm_multigoal_planner
{

GoalSetBuilder::GoalSetBuilder(rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client,
                               rclcpp::Logger logger, double service_timeout_s)
  : ik_client_(std::move(ik_client)), logger_(std::move(logger)), service_timeout_s_(service_timeout_s)
{
}

bool GoalSetBuilder::solveCartesianGoal(const moveit_msgs::msg::Constraints& c,
                                        const std::vector<std::string>& canonical_joint_names,
                                        std::vector<Eigen::VectorXd>& out_configs, std::string& error)
{
  const auto& pc = c.position_constraints.front();
  const auto& oc = c.orientation_constraints.front();

  if (pc.constraint_region.primitive_poses.empty())
  {
    error = "position_constraints[0].constraint_region.primitive_poses is empty";
    return false;
  }

  if (!ik_client_->wait_for_service(std::chrono::duration<double>(service_timeout_s_)))
  {
    error = std::string("IK service '") + ik_client_->get_service_name() + "' is not available";
    return false;
  }

  ik_solver_msgs::msg::IkTarget target;
  target.pose.header = pc.header;
  target.pose.pose.position = pc.constraint_region.primitive_poses.front().position;
  target.pose.pose.orientation = oc.orientation;

  auto request = std::make_shared<ik_solver_msgs::srv::GetIkArray::Request>();
  request->targets.push_back(target);
  request->seed_joint_names = canonical_joint_names;

  RCLCPP_DEBUG(logger_, "IK request frame='%s' pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
               target.pose.header.frame_id.c_str(), target.pose.pose.position.x, target.pose.pose.position.y,
               target.pose.pose.position.z, target.pose.pose.orientation.x, target.pose.pose.orientation.y,
               target.pose.pose.orientation.z, target.pose.pose.orientation.w);

  auto future = ik_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) != std::future_status::ready)
  {
    error = "timeout waiting for the IK response";
    return false;
  }
  auto response = future.get();

  std::size_t added = 0;
  for (const auto& sol : response->solutions)
    for (const auto& cfg : sol.configurations)
    {
      Eigen::VectorXd q;
      std::string reorder_err;
      if (!reorderByJointName(response->joint_names, cfg.configuration, canonical_joint_names, q, reorder_err))
      {
        RCLCPP_WARN(logger_, "Skipping an IK solution: %s", reorder_err.c_str());
        continue;
      }
      out_configs.push_back(std::move(q));
      ++added;
    }

  if (added == 0)
    RCLCPP_WARN(logger_, "No IK solution for a Cartesian goal.");
  RCLCPP_DEBUG(logger_, "%zu IK solution(s) received.", added);
  return true;
}

bool GoalSetBuilder::buildGoalConfigurations(const std::vector<moveit_msgs::msg::Constraints>& goal_constraints,
                                             const std::vector<std::string>& canonical_joint_names,
                                             std::vector<Eigen::VectorXd>& goal_configurations, std::string& error)
{
  goal_configurations.clear();

  for (std::size_t i = 0; i < goal_constraints.size(); ++i)
  {
    const auto& c = goal_constraints[i];
    const bool has_joint_goal = !c.joint_constraints.empty();
    const bool has_pose_goal = !c.position_constraints.empty() && !c.orientation_constraints.empty();
    std::string local_err;

    if (has_joint_goal)
    {
      std::vector<std::string> names;
      std::vector<double> values;
      for (const auto& jc : c.joint_constraints)
      {
        names.push_back(jc.joint_name);
        values.push_back(jc.position);
      }
      Eigen::VectorXd q;
      if (!reorderByJointName(names, values, canonical_joint_names, q, local_err))
      {
        error = "goal_constraints[" + std::to_string(i) + "]: " + local_err;
        return false;
      }
      goal_configurations.push_back(std::move(q));
    }
    else if (has_pose_goal)
    {
      if (!solveCartesianGoal(c, canonical_joint_names, goal_configurations, local_err))
      {
        error = "goal_constraints[" + std::to_string(i) + "]: " + local_err;
        return false;
      }
    }
    else
    {
      error = "goal_constraints[" + std::to_string(i) +
              "]: must specify either joint_constraints, or both position_constraints and orientation_constraints";
      return false;
    }
  }
  return true;
}

}  // namespace prm_multigoal_planner
