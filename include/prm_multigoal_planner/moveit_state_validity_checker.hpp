#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include <ompl/base/StateValidityChecker.h>

#if __has_include(<moveit/planning_scene/planning_scene.hpp>)
#include <moveit/planning_scene/planning_scene.hpp>  // MoveIt >= Jazzy
#else
#include <moveit/planning_scene/planning_scene.h>  // MoveIt Humble
#endif

namespace prm_multigoal_planner
{

/**
 * @brief OMPL state validity checker on a MoveIt planning scene snapshot.
 *
 * A state is the vector of the active joint positions of one planning group, in the order of
 * JointModelGroup::getActiveJointModelNames(). The other joints of the robot are kept at their
 * values in the scene's current state. A state is valid if the links moved by the group (and the
 * objects attached to them) are not in collision with the rest of the robot or the world, according
 * to the scene's ACM (PlanningScene::isStateColliding(state, group)). Collisions between links not
 * moved by the group (e.g. fixed cell elements) are ignored.
 *
 * Not thread safe (it reuses one RobotState): use it from one thread at a time.
 */
class MoveItStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  MoveItStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                             const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name);

  bool isValid(const ompl::base::State* state) const override;

  /// Same check on a configuration (active joints of the group, in group order).
  bool isValid(const Eigen::VectorXd& q) const;

  void setVerbose(bool verbose)
  {
    verbose_ = verbose;
  }

private:
  bool isValid(const double* q) const;

  planning_scene::PlanningSceneConstPtr scene_;
  std::string group_name_;
  const moveit::core::JointModelGroup* jmg_;
  mutable moveit::core::RobotState state_;
  bool verbose_{ false };
};

}  // namespace prm_multigoal_planner
