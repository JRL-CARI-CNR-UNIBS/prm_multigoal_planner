// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0

#include <prm_multigoal_planner/prm_multigoal_planner_action_server.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/version.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>

#include <prm_multigoal_planner/joint_order_utils.hpp>
#include <prm_multigoal_planner/scene_hash.hpp>

namespace prm_multigoal_planner
{

namespace ob = ompl::base;
namespace og = ompl::geometric;
using moveit_msgs::msg::MoveItErrorCodes;

namespace
{
#if RCLCPP_VERSION_MAJOR >= 28
rclcpp::QoS serviceQoS()
{
  return rclcpp::ServicesQoS();
}
#else
const rmw_qos_profile_t& serviceQoS()
{
  return rmw_qos_profile_services_default;
}
#endif

std::string vecToString(const Eigen::VectorXd& v)
{
  std::stringstream ss;
  ss << v.transpose();
  return ss.str();
}
}  // namespace

PrmMultigoalPlannerActionServer::PrmMultigoalPlannerActionServer(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  const std::string ik_solver_ns = node_->declare_parameter<std::string>("ik_solver_ns", "ik_solver");
  const std::string planning_scene_service =
      node_->declare_parameter<std::string>("planning_scene_service", "/get_planning_scene");
  const std::string action_name = node_->declare_parameter<std::string>("action_name", "prm_multigoal_plan");
  service_timeout_s_ = node_->declare_parameter<double>("service_timeout_s", 5.0);

  // Read at every roadmap (re)initialization.
  node_->declare_parameter<double>("collision_check_resolution", 0.02);
  // [m] added to the padding of the links moved by the group (robot-world checks only, not self-collisions)
  node_->declare_parameter<double>("collision_padding", 0.0);
  node_->declare_parameter<double>("scene_joint_tolerance", 1e-3);
  // Read at every request.
  node_->declare_parameter<double>("default_allowed_planning_time", 5.0);
  node_->declare_parameter<double>("utopia_tolerance", 0.01);
  node_->declare_parameter<bool>("try_direct_connection", true);
  node_->declare_parameter<double>("search_growth_ratio", 0.01);
  node_->declare_parameter<double>("informed_sampling_ratio", 0.7);
  node_->declare_parameter<bool>("simplify_solution", true);
  node_->declare_parameter<double>("simplify_max_time", 0.1);
  node_->declare_parameter<int>("max_roadmap_vertices", 100000);
  node_->declare_parameter<double>("feedback_period_s", 0.2);
  node_->declare_parameter<double>("waypoint_dt_s", 0.5);

  // Kinematics plugins are not needed: IK comes from the ik_solver node.
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", false);
  robot_model_ = robot_model_loader_->getModel();
  if (!robot_model_)
    throw std::runtime_error("prm_multigoal_planner: unable to load the robot model from robot_description");

  cb_group_ik_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_scene_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  ik_client_ = node_->create_client<ik_solver_msgs::srv::GetIkArray>(
      "/" + ik_solver_ns + "/get_task_redundant_ik_array", serviceQoS(), cb_group_ik_);
  scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(planning_scene_service, serviceQoS(),
                                                                           cb_group_scene_);

  goal_builder_ =
      std::make_unique<GoalSetBuilder>(ik_client_, node_->get_logger().get_child("goal_set_builder"), service_timeout_s_);

  using std::placeholders::_1;
  using std::placeholders::_2;
  clear_roadmap_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/clear_roadmap", std::bind(&PrmMultigoalPlannerActionServer::clearRoadmapCallback, this, _1, _2));

  server_ = rclcpp_action::create_server<Action>(
      node_, action_name, std::bind(&PrmMultigoalPlannerActionServer::handle_goal, this, _1, _2),
      std::bind(&PrmMultigoalPlannerActionServer::handle_cancel, this, _1),
      std::bind(&PrmMultigoalPlannerActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(node_->get_logger(), "'%s' action server ready (ik_solver_ns='%s', planning_scene_service='%s').",
              action_name.c_str(), ik_solver_ns.c_str(), planning_scene_service.c_str());
}

void PrmMultigoalPlannerActionServer::clearRoadmapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                           std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lk(contexts_mtx_);
  for (auto& [name, ctx] : contexts_)
    ctx->reset_requested = true;
  res->success = true;
  res->message = "Roadmap(s) of " + std::to_string(contexts_.size()) + " group(s) will be rebuilt at the next request";
}

PrmMultigoalPlannerActionServer::GroupContextPtr
PrmMultigoalPlannerActionServer::getGroupContext(const std::string& group_name, std::string& error)
{
  std::lock_guard<std::mutex> lk(contexts_mtx_);
  auto it = contexts_.find(group_name);
  if (it != contexts_.end())
    return it->second;

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  if (!jmg)
  {
    error = "planning group '" + group_name + "' not found in the robot model";
    return nullptr;
  }

  auto ctx = std::make_shared<GroupContext>();
  ctx->group_name = group_name;
  ctx->joint_names = jmg->getActiveJointModelNames();
  if (ctx->joint_names.empty() || jmg->getVariableCount() != ctx->joint_names.size())
  {
    error = "planning group '" + group_name + "' must contain only single-variable active joints";
    return nullptr;
  }
  ctx->joint_name_set.insert(ctx->joint_names.begin(), ctx->joint_names.end());

  ctx->space = std::make_shared<ob::RealVectorStateSpace>(static_cast<unsigned int>(ctx->joint_names.size()));
  ob::RealVectorBounds bounds(static_cast<unsigned int>(ctx->joint_names.size()));
  for (std::size_t i = 0; i < ctx->joint_names.size(); ++i)
  {
    const auto& b = robot_model_->getJointModel(ctx->joint_names[i])->getVariableBounds()[0];
    // Continuous joints: MoveIt gives [-pi, pi], used as is (no wrap-around).
    bounds.setLow(static_cast<unsigned int>(i), b.min_position_);
    bounds.setHigh(static_cast<unsigned int>(i), b.max_position_);
    RCLCPP_DEBUG(node_->get_logger(), "group '%s' joint[%zu] '%s' bounds=[%.4f, %.4f]", group_name.c_str(), i,
                 ctx->joint_names[i].c_str(), b.min_position_, b.max_position_);
  }
  ctx->space->setBounds(bounds);

  contexts_[group_name] = ctx;
  return ctx;
}

bool PrmMultigoalPlannerActionServer::fetchPlanningScene(moveit_msgs::msg::PlanningScene& scene_msg, std::string& error)
{
  if (!scene_client_->wait_for_service(std::chrono::duration<double>(service_timeout_s_)))
  {
    error = std::string("planning scene service '") + scene_client_->get_service_name() + "' not available";
    return false;
  }
  auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  using C = moveit_msgs::msg::PlanningSceneComponents;
  req->components.components = C::SCENE_SETTINGS | C::ROBOT_STATE | C::ROBOT_STATE_ATTACHED_OBJECTS |
                               C::WORLD_OBJECT_NAMES | C::WORLD_OBJECT_GEOMETRY | C::OCTOMAP | C::TRANSFORMS |
                               C::ALLOWED_COLLISION_MATRIX | C::LINK_PADDING_AND_SCALING;
  auto future = scene_client_->async_send_request(req);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) != std::future_status::ready)
  {
    error = "timeout waiting for the planning scene";
    return false;
  }
  scene_msg = future.get()->scene;
  return true;
}

bool PrmMultigoalPlannerActionServer::resetRoadmap(GroupContext& ctx, const moveit_msgs::msg::PlanningScene& scene_msg,
                                                   std::uint64_t hash, std::string& error)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  if (!scene->setPlanningSceneMsg(scene_msg))
  {
    error = "unable to apply the planning scene";
    return false;
  }

  // Safety margin: the edges are checked at discrete points, so a path that grazes an obstacle may touch it
  // between two checks. The padding grows the links moved by the group (start and goals included).
  const double padding = std::max(0.0, node_->get_parameter("collision_padding").as_double());
  if (padding > 0.0)
  {
    auto& env = scene->getCollisionEnvNonConst();
    for (const moveit::core::LinkModel* link :
         robot_model_->getJointModelGroup(ctx.group_name)->getUpdatedLinkModelsWithGeometry())
      env->setLinkPadding(link->getName(), env->getLinkPadding(link->getName()) + padding);
  }

  auto si = std::make_shared<ob::SpaceInformation>(ctx.space);
  auto checker = std::make_shared<MoveItStateValidityChecker>(si, scene, ctx.group_name);
  si->setStateValidityChecker(checker);
  const double resolution = node_->get_parameter("collision_check_resolution").as_double();
  // OMPL's resolution is a fraction of the space extent.
  si->setStateValidityCheckingResolution(std::clamp(resolution / ctx.space->getMaximumExtent(), 1e-6, 1.0));
  si->setup();

  auto planner = std::make_shared<MultigoalLazyPRMstar>(si);
  // LazyPRM::setup() needs a problem definition (it is replaced at every query).
  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
  planner->setProblemDefinition(pdef);
  planner->setup();

  ctx.scene = scene;
  ctx.si = si;
  ctx.checker = checker;
  ctx.planner = planner;
  ctx.scene_hash = hash;
  ctx.padding = padding;
  ctx.queries = 0;
  ctx.initialized = true;
  return true;
}

rclcpp_action::GoalResponse PrmMultigoalPlannerActionServer::handle_goal(const rclcpp_action::GoalUUID&,
                                                                         std::shared_ptr<const Action::Goal> goal)
{
  if (!goal)
    return rclcpp_action::GoalResponse::REJECT;
  if (goal->motion_plan_request.goal_constraints.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected: goal_constraints is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->motion_plan_request.group_name.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected: motion_plan_request.group_name is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PrmMultigoalPlannerActionServer::handle_cancel(const std::shared_ptr<GoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PrmMultigoalPlannerActionServer::handle_accepted(const std::shared_ptr<GoalHandle> gh)
{
  std::thread{ std::bind(&PrmMultigoalPlannerActionServer::execute, this, gh) }.detach();
}

void PrmMultigoalPlannerActionServer::execute(const std::shared_ptr<GoalHandle> gh)
{
  const auto goal = gh->get_goal();
  const auto& req = goal->motion_plan_request;
  const bool verbose = goal->verbose;
  const auto tic = std::chrono::steady_clock::now();
  auto elapsed = [&tic]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - tic).count(); };

  auto feedback = std::make_shared<Action::Feedback>();
  auto result = std::make_shared<Action::Result>();
  auto publish_feedback = [&](const std::string& status, float progress) {
    feedback->status = status;
    feedback->progress = progress;
    gh->publish_feedback(feedback);
  };
  auto abort = [&](int32_t code, const std::string& msg) {
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
    result->motion_plan_response = errorResponse(req, code);
    gh->abort(result);
  };
  auto cancel = [&]() {
    RCLCPP_INFO(node_->get_logger(), "Goal canceled.");
    result->motion_plan_response = errorResponse(req, MoveItErrorCodes::PREEMPTED);
    gh->canceled(result);
  };

  // Parameters read at every request
  const double utopia_tolerance = node_->get_parameter("utopia_tolerance").as_double();
  const double default_allowed_time = node_->get_parameter("default_allowed_planning_time").as_double();
  const bool try_direct = node_->get_parameter("try_direct_connection").as_bool();
  const double search_growth_ratio = node_->get_parameter("search_growth_ratio").as_double();
  const double informed_ratio = node_->get_parameter("informed_sampling_ratio").as_double();
  const bool simplify = node_->get_parameter("simplify_solution").as_bool();
  const double simplify_max_time = node_->get_parameter("simplify_max_time").as_double();
  const long max_vertices = node_->get_parameter("max_roadmap_vertices").as_int();
  const double feedback_period = node_->get_parameter("feedback_period_s").as_double();
  const double scene_joint_tol = node_->get_parameter("scene_joint_tolerance").as_double();
  const double allowed_time = (req.allowed_planning_time > 0.0) ? req.allowed_planning_time : default_allowed_time;

  std::string err;
  GroupContextPtr ctx = getGroupContext(req.group_name, err);
  if (!ctx)
    return abort(MoveItErrorCodes::INVALID_GROUP_NAME, err);

  // ---- Exclusive access to the group's roadmap ----
  publish_feedback("Waiting for the roadmap", 0.01f);
  std::unique_lock<std::timed_mutex> lock(ctx->mtx, std::defer_lock);
  while (!lock.try_lock_for(std::chrono::milliseconds(50)))
    if (gh->is_canceling())
      return cancel();

  // ---- Planning scene: reuse or rebuild the roadmap ----
  publish_feedback("Fetching planning scene", 0.05f);
  moveit_msgs::msg::PlanningScene scene_msg;
  if (!fetchPlanningScene(scene_msg, err))
    return abort(MoveItErrorCodes::FAILURE, "Cannot get the planning scene: " + err);
  const std::uint64_t hash = planningSceneHash(scene_msg, ctx->joint_name_set, scene_joint_tol);

  std::string reset_reason;
  if (!ctx->initialized)
    reset_reason = "first request";
  else if (ctx->reset_requested.exchange(false))
    reset_reason = "clear_roadmap requested";
  else if (hash != ctx->scene_hash)
    reset_reason = "planning scene changed";
  else if (std::max(0.0, node_->get_parameter("collision_padding").as_double()) != ctx->padding)
    reset_reason = "collision_padding changed";
  else if (max_vertices > 0 && ctx->planner->milestoneCount() > static_cast<unsigned long>(max_vertices))
    reset_reason = "roadmap exceeded max_roadmap_vertices";

  if (!reset_reason.empty())
  {
    if (!resetRoadmap(*ctx, scene_msg, hash, err))
      return abort(MoveItErrorCodes::FAILURE, err);
    RCLCPP_INFO(node_->get_logger(), "[%s] new roadmap (%s).", req.group_name.c_str(), reset_reason.c_str());
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "[%s] reusing roadmap: %lu vertices, %lu edges, %u previous queries.",
                req.group_name.c_str(), ctx->planner->milestoneCount(), ctx->planner->edgeCount(), ctx->queries);
  }
  ctx->checker->setVerbose(verbose);
  const auto& joint_names = ctx->joint_names;
  const auto& space = ctx->space;
  const auto& checker = ctx->checker;

  auto in_bounds = [&](const Eigen::VectorXd& q) {
    const auto& b = space->getBounds();
    for (Eigen::Index i = 0; i < q.size(); ++i)
      if (q(i) < b.low[i] || q(i) > b.high[i])
        return false;
    return true;
  };

  // ---- Start ----
  Eigen::VectorXd start_q;
  if (req.start_state.joint_state.name.empty())
  {
    // MoveIt convention: empty start state = current state.
    std::vector<double> v;
    ctx->scene->getCurrentState().copyJointGroupPositions(req.group_name, v);
    start_q = Eigen::Map<Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
  }
  else if (!reorderByJointName(req.start_state.joint_state.name, req.start_state.joint_state.position, joint_names,
                               start_q, err))
    return abort(MoveItErrorCodes::INVALID_ROBOT_STATE, "Invalid start_state: " + err);

  RCLCPP_DEBUG(node_->get_logger(), "start_q = [%s]", vecToString(start_q).c_str());
  if (!in_bounds(start_q))
    return abort(MoveItErrorCodes::INVALID_ROBOT_STATE, "Start state out of joint bounds: [" + vecToString(start_q) + "]");
  if (!checker->isValid(start_q))
    return abort(MoveItErrorCodes::START_STATE_IN_COLLISION,
                 ctx->padding > 0.0 ? "Start state in collision (collision_padding " + std::to_string(ctx->padding) +
                                          " m included)." :
                                      std::string("Start state in collision."));

  // ---- Goals ----
  publish_feedback("Building candidate goal set", 0.10f);
  std::vector<Eigen::VectorXd> raw_goals;
  if (!goal_builder_->buildGoalConfigurations(req.goal_constraints, joint_names, raw_goals, err))
    return abort(MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS, "Failed to build goal set: " + err);
  if (raw_goals.empty())
    return abort(MoveItErrorCodes::NO_IK_SOLUTION, "No candidate goal configuration (no IK solution).");

  std::vector<Eigen::VectorXd> goals;
  std::size_t out_of_bounds = 0, in_collision = 0;
  for (const auto& q : raw_goals)
  {
    if (!in_bounds(q))
      ++out_of_bounds;
    else if (!checker->isValid(q))
      ++in_collision;
    else
      goals.push_back(q);
  }
  if (verbose || goals.empty())
    RCLCPP_INFO(node_->get_logger(), "%zu candidate goal(s): %zu valid, %zu out of bounds, %zu in collision%s.",
                raw_goals.size(), goals.size(), out_of_bounds, in_collision,
                ctx->padding > 0.0 ? (" (collision_padding " + std::to_string(ctx->padding) + " m included)").c_str() :
                                     "");
  if (goals.empty())
    return abort(MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS, "No valid candidate goal configuration.");

  double utopia = std::numeric_limits<double>::infinity();
  std::size_t utopia_goal = 0;
  for (std::size_t i = 0; i < goals.size(); ++i)
  {
    const double d = (goals[i] - start_q).norm();
    if (d < utopia)
    {
      utopia = d;
      utopia_goal = i;
    }
  }

  // Start already at a goal: nothing to plan.
  if (utopia < 1e-9)
  {
    result->motion_plan_response = pathToMotionPlanResponse(req, joint_names, { start_q, goals[utopia_goal] }, elapsed());
    publish_feedback("Done (start is a goal)", 1.0f);
    gh->succeed(result);
    return;
  }

  // ---- Query ----
  publish_feedback("Planning", 0.15f);
  const auto& si = ctx->si;
  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  {
    ob::ScopedState<> s(space);
    for (Eigen::Index i = 0; i < start_q.size(); ++i)
      s[static_cast<unsigned int>(i)] = start_q(i);
    pdef->addStartState(s);
  }
  auto goal_states = std::make_shared<ob::GoalStates>(si);
  for (const auto& q : goals)
  {
    ob::ScopedState<> g(space);
    for (Eigen::Index i = 0; i < q.size(); ++i)
      g[static_cast<unsigned int>(i)] = q(i);
    goal_states->addState(g);
  }
  pdef->setGoal(goal_states);
  const double cost_threshold = (1.0 + utopia_tolerance) * utopia;
  auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
  objective->setCostThreshold(ob::Cost(cost_threshold));
  pdef->setOptimizationObjective(objective);

  auto& planner = ctx->planner;
  planner->setTryDirectConnection(try_direct);
  planner->setSearchGrowthRatio(search_growth_ratio);
  planner->setInformedSamplingRatio(informed_ratio);
  planner->setProblemDefinition(pdef);  // also clears the previous query (start/goal lists), not the roadmap

  const unsigned long vertices_before = planner->milestoneCount();
  const double plan_tic = elapsed();
  double next_feedback = plan_tic;
  bool canceled = false;
  ob::PlannerTerminationCondition ptc([&]() {
    if (gh->is_canceling())
    {
      canceled = true;
      return true;
    }
    const double t = elapsed();
    if (t - plan_tic >= allowed_time)
      return true;
    if (t >= next_feedback)
    {
      next_feedback = t + feedback_period;
      std::stringstream ss;
      ss << "Planning: " << planner->milestoneCount() << " vertices, best cost " << planner->bestCostValue()
         << " (threshold " << cost_threshold << ")";
      publish_feedback(ss.str(), 0.15f + 0.75f * static_cast<float>(std::min(1.0, (t - plan_tic) / allowed_time)));
    }
    return false;
  });

  const ob::PlannerStatus status = planner->solve(ptc);
  planner->clearQuery();
  ++ctx->queries;
  const double solve_time = elapsed() - plan_tic;

  if (canceled)
    return cancel();

  if (status != ob::PlannerStatus::EXACT_SOLUTION || !pdef->hasExactSolution())
  {
    if (status == ob::PlannerStatus::INVALID_START)
      return abort(MoveItErrorCodes::START_STATE_IN_COLLISION, "Planner rejected the start state.");
    if (status == ob::PlannerStatus::INVALID_GOAL)
      return abort(MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS, "Planner rejected all the goal states.");
    return abort(MoveItErrorCodes::PLANNING_FAILED,
                 "No solution found within " + std::to_string(allowed_time) + " s (roadmap: " +
                     std::to_string(planner->milestoneCount()) + " vertices).");
  }

  auto path = std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
  const double roadmap_cost = path->length();
  const bool satisfied = planner->querySatisfied();

  const auto& stats = planner->queryStats();
  RCLCPP_INFO(node_->get_logger(),
              "[%s] solved in %.3f s: cost %.4f, utopia %.4f (ratio %.4f, %s), %lu new samples (%lu informed), roadmap %lu -> %lu "
              "vertices, %lu searches (%lu retries, %.3f s).",
              req.group_name.c_str(), solve_time, roadmap_cost, utopia, roadmap_cost / utopia,
              satisfied ? "early termination" : "time limit", planner->queryIterations(), stats.informed_samples, vertices_before,
              planner->milestoneCount(), stats.searches, stats.retries, stats.search_time);

  if (simplify && path->getStateCount() > 2)
  {
    publish_feedback("Simplifying solution", 0.92f);
    og::PathSimplifier simplifier(si);
    simplifier.simplify(*path, simplify_max_time);
    if (verbose)
      RCLCPP_INFO(node_->get_logger(), "Simplified path: cost %.4f -> %.4f, %zu waypoints.", roadmap_cost,
                  path->length(), path->getStateCount());
  }

  std::vector<Eigen::VectorXd> waypoints;
  waypoints.reserve(path->getStateCount());
  const unsigned int dof = space->getDimension();
  for (std::size_t k = 0; k < path->getStateCount(); ++k)
  {
    const auto* st = path->getState(static_cast<unsigned int>(k))->as<ob::RealVectorStateSpace::StateType>();
    waypoints.emplace_back(Eigen::Map<const Eigen::VectorXd>(st->values, dof));
  }

  publish_feedback("Done", 1.0f);
  result->motion_plan_response = pathToMotionPlanResponse(req, joint_names, waypoints, elapsed());
  gh->succeed(result);
}

moveit_msgs::msg::MotionPlanResponse
PrmMultigoalPlannerActionServer::pathToMotionPlanResponse(const moveit_msgs::msg::MotionPlanRequest& req,
                                                          const std::vector<std::string>& joint_names,
                                                          const std::vector<Eigen::VectorXd>& waypoints,
                                                          double planning_time) const
{
  const double dt = node_->get_parameter("waypoint_dt_s").as_double();

  moveit_msgs::msg::MotionPlanResponse out;
  out.trajectory_start = req.start_state;
  out.group_name = req.group_name;
  out.planning_time = planning_time;

  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = joint_names;
  jt.points.reserve(waypoints.size());
  for (std::size_t k = 0; k < waypoints.size(); ++k)
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(waypoints[k].data(), waypoints[k].data() + waypoints[k].size());
    pt.time_from_start = rclcpp::Duration::from_seconds(dt * static_cast<double>(k));
    jt.points.push_back(std::move(pt));
  }
  out.trajectory.joint_trajectory = std::move(jt);
  out.error_code.val = MoveItErrorCodes::SUCCESS;
  return out;
}

moveit_msgs::msg::MotionPlanResponse
PrmMultigoalPlannerActionServer::errorResponse(const moveit_msgs::msg::MotionPlanRequest& req, int32_t error_code)
{
  moveit_msgs::msg::MotionPlanResponse out;
  out.trajectory_start = req.start_state;
  out.group_name = req.group_name;
  out.planning_time = 0.0;
  out.error_code.val = error_code;
  return out;
}

}  // namespace prm_multigoal_planner
