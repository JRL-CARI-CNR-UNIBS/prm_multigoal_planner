// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0

#include <prm_multigoal_planner/moveit_state_validity_checker.hpp>

#include <stdexcept>

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace prm_multigoal_planner
{

MoveItStateValidityChecker::MoveItStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                                                       const planning_scene::PlanningSceneConstPtr& scene,
                                                       const std::string& group_name)
  : ompl::base::StateValidityChecker(si)
  , scene_(scene)
  , group_name_(group_name)
  , jmg_(scene->getRobotModel()->getJointModelGroup(group_name))
  , state_(scene->getCurrentState())
{
  if (!jmg_)
    throw std::runtime_error("MoveItStateValidityChecker: unknown planning group '" + group_name + "'");
}

bool MoveItStateValidityChecker::isValid(const double* q) const
{
  // Group variables == active joints of the group (all single-variable, checked by the caller).
  state_.setJointGroupPositions(jmg_, q);
  state_.update();
  return !scene_->isStateColliding(state_, group_name_, verbose_);
}

bool MoveItStateValidityChecker::isValid(const ompl::base::State* state) const
{
  return isValid(state->as<ompl::base::RealVectorStateSpace::StateType>()->values);
}

bool MoveItStateValidityChecker::isValid(const Eigen::VectorXd& q) const
{
  if (static_cast<unsigned int>(q.size()) != jmg_->getVariableCount())
    return false;
  return isValid(q.data());
}

}  // namespace prm_multigoal_planner
