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

#include <adaptive_cruise_controller/adaptive_cruise_pid_controller.hpp>

namespace motion_planning
{

AdaptiveCruisePIDController::AdaptiveCruisePIDController(
  const double baselink2front, const AccParam & acc_param)
: baselink2front_(baselink2front), acc_param_(acc_param)
{
  // TODO
}

void AdaptiveCruisePIDController::getInformationForAdaptiveCruise(
  const Trajectory & trajectory, const geometry_msgs::msg::PoseStamped & pose,
  const Odometry & odometry, const PredictedObject & object)
{
  // TODO
  // using dt ( = object_timestamp - pose_timstamp), do time compensation for object position
}

void AdaptiveCruisePIDController::calculate()
{
  if (!acc_info_ptr_) {
    return;
  }

  updateState(acc_info_ptr_->info_time, acc_info_ptr_->current_object_velocity);

  if (current_state == State::STOP) {
    calcTrajectoryWithStopPoints();
  } else if (current_state == State::ACC) {
    calculateTargetMotion();
  }

  prev_acc_info_ptr_ = acc_info_ptr_;
}

void AdaptiveCruisePIDController::calcTrajectoryWithStopPoints()
{
  // TODO(insert Stop Velocity to Trajectory)
  /*
  double stop_dist = acc_info_ptr_->current_distance_to_object;
  double minimum_stop_dist_with_acc_limit = calcStopDistance(acc_param_.stop_min_acceleration);
  if (stop_dist > minimum_stop_dist_with_acc_limit) {
    stop_dist = minimum_stop_dist_with_acc_limit;
    acc_motion_.emergency = true;
  }
  acc_motion_.planned_trajectory = insertStopVelocity(stop_dist);
  */
}

void AdaptiveCruisePIDController::calculateTargetMotion()
{
  // calculate target motion
}

bool AdaptiveCruisePIDController::getTargetMotion(
  double & target_velocity, double & target_acc, double & target_jerk)
{
  if (!acc_info_ptr_) {
    return false;
  }

  if (current_state != State::ACC) {
    // no target motion
    return false;
  }

  target_velocity = acc_motion_.target_velocity;
  target_acc = acc_motion_.target_acceleration;
  target_jerk = acc_motion_.target_jerk;

  return true;
}

Trajectory AdaptiveCruisePIDController::getAccTrajectory(bool & emergency_flag)
{
  if (!acc_info_ptr_) {
    emergency_flag = false;
    return acc_info_ptr_->original_trajectory;
  }

  if (current_state != State::STOP) {
    // no trajectory update
    emergency_flag = false;
    return acc_info_ptr_->original_trajectory;
  }

  emergency_flag = acc_motion_.emergency;
  return acc_motion_.planned_trajectory;
}

void AdaptiveCruisePIDController::updateState(
  const rclcpp::Time & current_time, const double obstacle_velocity)
{
  if (!prev_acc_info_ptr_) {
    current_state = State::NONE;
  }

  if (
    (current_time - prev_acc_info_ptr_->info_time).seconds() > acc_param_.reset_time_to_acc_state) {
    // reset current_state if previous acc_time is too old
    current_state = State::NONE;
  }

  double thresh_velocity = acc_param_.object_low_velocity_thresh;

  // provide a hysteresis in the state to prevent chattering
  if (current_state == State::STOP) {
    thresh_velocity += acc_param_.object_velocity_hysteresis_margin;
  }

  if (obstacle_velocity >= thresh_velocity) {
    current_state = State::ACC;
  } else {
    current_state = State::STOP;
  }
}

}  // namespace motion_planning