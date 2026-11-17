// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <set>
#include <string>

#include <moveit_msgs/msg/planning_scene.hpp>

namespace prm_multigoal_planner
{

/**
 * @brief Hash of the part of a planning scene that affects collision checking for a planning group.
 *
 * Two scenes with the same hash give the same collision result for every configuration of the group,
 * so a roadmap validated on one can be reused on the other. Ignored: the scene name, object colors,
 * header stamps, and the positions of the group's own joints (they are the planning variables).
 * Included: world objects and octomap, attached objects, ACM, link padding/scale, fixed frame
 * transforms, and the positions of all the joints NOT in the group (rounded to
 * @p joint_tolerance, so that sensor noise on e.g. a gripper does not reset the roadmap).
 *
 * Collision objects and fixed transforms are sorted by id first, so their order does not matter.
 *
 * @param scene Full planning scene (not a diff), as returned by /get_planning_scene.
 * @param group_joints Names of the joints that are planning variables (excluded from the hash).
 * @param joint_tolerance Quantization of the other joints' positions [rad or m].
 */
std::uint64_t planningSceneHash(moveit_msgs::msg::PlanningScene scene, const std::set<std::string>& group_joints,
                                double joint_tolerance);

}  // namespace prm_multigoal_planner
