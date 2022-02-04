// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADAPTIVE_CRUISE_CONTROLLER__ADAPTIVE_CRUISE_CONTROL_HPP_
#define ADAPTIVE_CRUISE_CONTROLLER__ADAPTIVE_CRUISE_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>

namespace motion_planning
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
class AdaptiveCruiseControllerNode : public rclcpp::Node
{
public:
  explicit AdaptiveCruiseControllerNode(const rclcpp::NodeOptions & options);

private:
  struct Param
  {
  };

  Param param_;

  tier4_autoware_utils::SelfPoseListener self_pose_listener_;
};

}  // namespace motion_planning

#endif  // ADAPTIVE_CRUISE_CONTROLLER__ADAPTIVE_CRUISE_CONTROL_HPP_
