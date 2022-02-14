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

#ifndef ADAPTIVE_CRUISE_CONTROLLER__ADAPTIVE_CRUISE_CONTROL_CORE_HPP_
#define ADAPTIVE_CRUISE_CONTROLLER__ADAPTIVE_CRUISE_CONTROL_CORE_HPP_

#include <adaptive_cruise_controller/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace motion_planning
{

using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using nav_msgs::msg::Odometry;

struct AccParam
{
  double object_low_velocity_thresh;
  double object_velocity_hysteresis_margin;
  double reset_time_to_acc_state;
  double acc_min_acceleration;
  double acc_min_jerk;
  double stop_min_acceleration;
  double object_min_acceleration;
  double minimum_margin_distance;
  double idling_time;
  double breaking_delay_time;
  double p_term_in_velocity_pid;
};

enum State { NONE = 0, ACC = 1, STOP = 2 };

struct AccMotion
{
  double target_velocity;
  double target_acceleration;
  double target_jerk;
  TrajectoryPoints planned_trajectory;
  geometry_msgs::msg::Pose stop_pose;
  bool emergency;  // true when ACC makes a plan to collide with a car in front
};

struct AdaptiveCruiseInformation
{
  rclcpp::Time info_time;  // current time of pose stamped
  double current_ego_velocity;
  double current_object_velocity;
  double current_distance_to_object;
  double ideal_distance_to_object;
  PredictedObject target_object;
  TrajectoryPoints original_trajectory;
};

class AdaptiveCruiseControlCore
{
public:
  explicit AdaptiveCruiseControlCore(const double baselink2front, const AccParam & acc_param);
  AdaptiveCruiseInformation getAccInfo() { return *acc_info_ptr_; }

  void calcInformationForAdaptiveCruise(
    const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::PoseStamped & pose,
    const Odometry & odometry, const PredictedObject & object, const rclcpp::Time & object_time);

  void calculate();
  bool getTargetMotion(double & target_velocity, double & target_acc, double & target_jerk);
  TrajectoryPoints getAccTrajectory(bool & emergency_flag);
  State getState() { return current_state; };
  AccMotion getAccMotion() { return acc_motion_; };

private:
  // parameter
  double baselink2front_;
  AccParam acc_param_;

  // variables
  State current_state = State::NONE;
  double prev_target_velocity_;
  std::shared_ptr<AdaptiveCruiseInformation> acc_info_ptr_;
  std::shared_ptr<AdaptiveCruiseInformation> prev_acc_info_ptr_;
  AccMotion acc_motion_;

  // functions
  void updateState(const rclcpp::Time & current_time, const double obstacle_velocity);
  double getIdealDistanceToObject(const double current_velocity, const double object_velocity);
  double calcStoppingDistFromCurrentVel(const double current_velocity);
  void calculateTargetMotion();
  void calcTrajectoryWithStopPoints();
  TrajectoryPoints insertStopPoint(
    const double stop_distance, const TrajectoryPoints & trajectory_points,
    geometry_msgs::msg::Pose & stop_pose);
};

}  // namespace motion_planning

#endif  // ADAPTIVE_CRUISE_CONTROLLER__ADAPTIVE_CRUISE_CONTROL_CORE_HPP_
