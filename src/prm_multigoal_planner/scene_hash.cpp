// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0

#include <prm_multigoal_planner/scene_hash.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace prm_multigoal_planner
{

namespace
{

void clearStamp(std_msgs::msg::Header& h)
{
  h.stamp.sec = 0;
  h.stamp.nanosec = 0;
}

/// FNV-1a, 64 bit: stable across runs (std::hash is not required to be).
std::uint64_t fnv1a(const std::uint8_t* data, std::size_t size)
{
  std::uint64_t h = 1469598103934665603ULL;
  for (std::size_t i = 0; i < size; ++i)
  {
    h ^= data[i];
    h *= 1099511628211ULL;
  }
  return h;
}

}  // namespace

std::uint64_t planningSceneHash(moveit_msgs::msg::PlanningScene scene, const std::set<std::string>& group_joints,
                                double joint_tolerance)
{
  scene.name.clear();
  scene.object_colors.clear();

  // Robot state: keep only the joints that are not planning variables, quantized and sorted.
  auto& js = scene.robot_state.joint_state;
  std::vector<std::pair<std::string, double>> other_joints;
  for (std::size_t i = 0; i < js.name.size() && i < js.position.size(); ++i)
    if (group_joints.count(js.name[i]) == 0)
    {
      double p = js.position[i];
      if (joint_tolerance > 0.0)
        p = std::round(p / joint_tolerance) * joint_tolerance + 0.0;  // + 0.0: -0.0 -> +0.0 (same bytes)
      other_joints.emplace_back(js.name[i], p);
    }
  std::sort(other_joints.begin(), other_joints.end());
  js = sensor_msgs::msg::JointState();
  for (const auto& [name, pos] : other_joints)
  {
    js.name.push_back(name);
    js.position.push_back(pos);
  }
  clearStamp(scene.robot_state.multi_dof_joint_state.header);
  for (auto& a : scene.robot_state.attached_collision_objects)
    clearStamp(a.object.header);
  std::sort(scene.robot_state.attached_collision_objects.begin(), scene.robot_state.attached_collision_objects.end(),
            [](const auto& a, const auto& b) { return a.object.id < b.object.id; });

  for (auto& t : scene.fixed_frame_transforms)
    clearStamp(t.header);
  std::sort(scene.fixed_frame_transforms.begin(), scene.fixed_frame_transforms.end(),
            [](const auto& a, const auto& b) { return a.child_frame_id < b.child_frame_id; });

  for (auto& o : scene.world.collision_objects)
    clearStamp(o.header);
  std::sort(scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
            [](const auto& a, const auto& b) { return a.id < b.id; });
  clearStamp(scene.world.octomap.header);
  clearStamp(scene.world.octomap.octomap.header);

  // CDR does not write the alignment padding bytes: serialize into a zeroed buffer that is already
  // large enough (so it is not reallocated), otherwise equal messages can give different bytes.
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
  rclcpp::SerializedMessage sizing;
  serializer.serialize_message(&scene, &sizing);

  rclcpp::SerializedMessage serialized(sizing.size() + 64);
  auto& raw = serialized.get_rcl_serialized_message();
  std::memset(raw.buffer, 0, raw.buffer_capacity);
  serializer.serialize_message(&scene, &serialized);
  return fnv1a(raw.buffer, raw.buffer_length);
}

}  // namespace prm_multigoal_planner
