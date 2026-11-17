// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <parallel_multigoal_planner_msgs/action/parallel_multigoal_plan.hpp>
#include <ik_solver_msgs/srv/get_ik_array.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <std_srvs/srv/trigger.hpp>

#if __has_include(<moveit/robot_model_loader/robot_model_loader.hpp>)
#include <moveit/robot_model_loader/robot_model_loader.hpp>  // MoveIt >= Jazzy
#else
#include <moveit/robot_model_loader/robot_model_loader.h>  // MoveIt Humble
#endif

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <prm_multigoal_planner/goal_set_builder.hpp>
#include <prm_multigoal_planner/moveit_state_validity_checker.hpp>
#include <prm_multigoal_planner/multigoal_lazy_prm_star.hpp>

namespace prm_multigoal_planner
{

/**
 * @brief Action server for parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan, based on a
 * persistent multi-goal Lazy PRM* (OMPL).
 *
 * One roadmap is kept per planning group. For every goal:
 *  1. the planning scene is fetched from @c planning_scene_service;
 *  2. if the scene differs (see planningSceneHash()) from the one the group's roadmap was built on, the
 *     roadmap is discarded and rebuilt on the new scene; otherwise it is reused as is;
 *  3. goal_constraints become candidate joint-space goals (IK for Cartesian elements), filtered by
 *     joint bounds and collisions;
 *  4. MultigoalLazyPRMstar grows/queries the roadmap until cost < (1 + utopia_tolerance) * utopia,
 *     utopia = min_i ||goal_i - start||, or the time budget ends (best solution so far is returned);
 *  5. the path is (optionally) shortcut and returned as a MotionPlanResponse.
 *
 * Requests for the same group are serialized (they share the roadmap); different groups run in parallel.
 */
class PrmMultigoalPlannerActionServer
{
public:
  using Action = parallel_multigoal_planner_msgs::action::ParallelMultigoalPlan;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  /**
   * @brief Declare parameters, load the robot model from robot_description, create clients/servers.
   * @throw std::runtime_error if the robot model cannot be loaded.
   */
  explicit PrmMultigoalPlannerActionServer(const rclcpp::Node::SharedPtr& node);

private:
  /// Roadmap and everything it depends on, for one planning group.
  struct GroupContext
  {
    std::timed_mutex mtx;  ///< held for a whole request
    std::string group_name;
    std::vector<std::string> joint_names;  ///< active joints, group order
    std::set<std::string> joint_name_set;
    std::shared_ptr<ompl::base::RealVectorStateSpace> space;

    // Valid after the first request; replaced when the scene changes.
    bool initialized{ false };
    std::uint64_t scene_hash{ 0 };
    double padding{ 0.0 };  ///< collision_padding the roadmap was built with
    planning_scene::PlanningScenePtr scene;
    ompl::base::SpaceInformationPtr si;
    std::shared_ptr<MoveItStateValidityChecker> checker;
    std::shared_ptr<MultigoalLazyPRMstar> planner;
    unsigned int queries{ 0 };  ///< queries answered on the current roadmap

    std::atomic<bool> reset_requested{ false };
  };
  using GroupContextPtr = std::shared_ptr<GroupContext>;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> gh);
  void handle_accepted(const std::shared_ptr<GoalHandle> gh);
  void execute(const std::shared_ptr<GoalHandle> gh);

  /// Get (creating it if needed) the context of a group. Null with @p error if the group is not plannable.
  GroupContextPtr getGroupContext(const std::string& group_name, std::string& error);

  /// Fetch the full planning scene message from move_group.
  bool fetchPlanningScene(moveit_msgs::msg::PlanningScene& scene_msg, std::string& error);

  /// Build a new scene / checker / space information / empty roadmap for @p ctx.
  bool resetRoadmap(GroupContext& ctx, const moveit_msgs::msg::PlanningScene& scene_msg, std::uint64_t hash,
                    std::string& error);

  void clearRoadmapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  moveit_msgs::msg::MotionPlanResponse pathToMotionPlanResponse(const moveit_msgs::msg::MotionPlanRequest& req,
                                                                 const std::vector<std::string>& joint_names,
                                                                 const std::vector<Eigen::VectorXd>& waypoints,
                                                                 double planning_time) const;

  static moveit_msgs::msg::MotionPlanResponse errorResponse(const moveit_msgs::msg::MotionPlanRequest& req,
                                                             int32_t error_code);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_roadmap_srv_;

  rclcpp::CallbackGroup::SharedPtr cb_group_ik_;
  rclcpp::CallbackGroup::SharedPtr cb_group_scene_;
  rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;

  std::unique_ptr<GoalSetBuilder> goal_builder_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;

  std::mutex contexts_mtx_;
  std::map<std::string, GroupContextPtr> contexts_;

  // Parameters read once at startup (the others are read at every request).
  double service_timeout_s_;
};

}  // namespace prm_multigoal_planner
