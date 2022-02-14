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

#include <adaptive_cruise_controller/adaptive_cruise_control_core.hpp>

namespace motion_planning
{

AdaptiveCruisePIDController::AdaptiveCruisePIDController(
  const double baselink2front, const AccParam & acc_param)
: baselink2front_(baselink2front), acc_param_(acc_param)
{
  // TODO
}

void AdaptiveCruisePIDController::calcInformationForAdaptiveCruise(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::PoseStamped & pose,
  const Odometry & odometry, const PredictedObject & object, const rclcpp::Time & object_time)
{
  const auto object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const double object_velocity = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  double object_diff_angle = 0.0;
  getDiffAngleWithTrajectory(object_pose, trajectory_points, object_diff_angle);

  /* calculate the object velocity along trajectory */
  const double object_velocity_along_traj = object_velocity * std::cos(object_diff_angle);

  /* calculate distance to object */
  // calculate raw distance to the center-position of object from the base_link of ego-car
  double dist_to_object = tier4_autoware_utils::calcSignedArcLength(
    trajectory_points, pose.pose.position, object_pose.position);

  // considering the size of ego-car and object
  dist_to_object -= (baselink2front_ + object.shape.dimensions.x / 2.0);

  // delay compensation ( add object's travling distance in the delay time )
  const double delay_time = (rclcpp::Time(pose.header.stamp) - object_time).seconds();
  const double running_distance_in_delay = delay_time * object_velocity_along_traj;
  dist_to_object += running_distance_in_delay;

  /* input acc information */
  acc_info_ptr_ = std::make_shared<AdaptiveCruiseInformation>();
  acc_info_ptr_->current_distance_to_object = dist_to_object;
  acc_info_ptr_->current_ego_velocity = odometry.twist.twist.linear.x;
  acc_info_ptr_->current_object_velocity = object_velocity_along_traj;
  acc_info_ptr_->ideal_distance_to_object =
    getIdealDistanceToObject(odometry.twist.twist.linear.x, object_velocity_along_traj);
  acc_info_ptr_->info_time = rclcpp::Time(pose.header.stamp);
  acc_info_ptr_->target_object = object;
  acc_info_ptr_->original_trajectory = trajectory_points;
}

double AdaptiveCruisePIDController::getIdealDistanceToObject(
  const double current_velocity, const double object_velocity)
{
  // ego current velocity ... v_ego
  // ego minimum deceleration ... a_ego
  // object velocity ... v_obj
  // object minimum deceleration ... a_obj
  // idling_time ... t_idling
  // ideal_distance ... D
  // minimum_margin_distance ... d_margin
  // D = d_margin + v_e * t_idling + v_ego^2/(2*|a_ego|) - v_obj^2/(2*|a_obj|)

  const double margin_distance = acc_param_.minimum_margin_distance;
  const double idle_traving_distance = current_velocity * acc_param_.idling_time;
  const double ego_braking_distance =
    (current_velocity * current_velocity) / (2.0 * std::fabs(acc_param_.acc_min_acceleration));
  const double object_braking_distance =
    (object_velocity * object_velocity) / (2.0 * std::fabs(acc_param_.object_min_acceleration));

  return margin_distance + idle_traving_distance + ego_braking_distance - object_braking_distance;
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
  const double original_stop_dist =
    acc_info_ptr_->current_distance_to_object - acc_param_.minimum_margin_distance;
  const double min_stop_dist = calcStoppingDistFromCurrentVel(acc_info_ptr_->current_ego_velocity);

  double target_stop_dist;
  if (original_stop_dist >= min_stop_dist) {
    target_stop_dist = original_stop_dist;
    acc_motion_.emergency = false;
  } else {
    target_stop_dist = min_stop_dist;
    acc_motion_.emergency = true;
  }

  acc_motion_.planned_trajectory =
    insertStopPoint(target_stop_dist, acc_info_ptr_->original_trajectory, acc_motion_.stop_pose);
}

void AdaptiveCruisePIDController::calculateTargetMotion()
{
  const double diff_distance_to_object =
    acc_info_ptr_->current_distance_to_object - acc_info_ptr_->ideal_distance_to_object;

  acc_motion_.target_velocity = acc_info_ptr_->current_ego_velocity +
                                acc_param_.p_term_in_velocity_pid * diff_distance_to_object;
  // TODO(tkimura4) calculate accel and jerk depends on current diff distance
  acc_motion_.target_acceleration = acc_param_.acc_min_acceleration;
  acc_motion_.target_jerk = acc_param_.acc_min_jerk;
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

TrajectoryPoints AdaptiveCruisePIDController::getAccTrajectory(bool & emergency_flag)
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

double AdaptiveCruisePIDController::calcStoppingDistFromCurrentVel(const double current_velocity)
{
  const double idling_travel_distance = current_velocity * acc_param_.breaking_delay_time;
  const double braking_distance =
    (current_velocity * current_velocity) / (2.0 * acc_param_.stop_min_acceleration);
  return idling_travel_distance + braking_distance;
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

TrajectoryPoints AdaptiveCruisePIDController::insertStopPoint(
  const double stop_distance, const TrajectoryPoints & trajectory_points,
  geometry_msgs::msg::Pose & stop_pose)
{
  TrajectoryPoints trajectory_with_stop_point = trajectory_points;

  if (trajectory_points.empty()) {
    return trajectory_points;
  }

  // calulcate stop index and stop point
  double accumulated_length = 0;
  double insert_idx = 0;
  for (size_t i = 1; i < trajectory_points.size(); i++) {
    const auto prev_pose = trajectory_points.at(i - 1).pose;
    const auto curr_pose = trajectory_points.at(i).pose;
    const double segment_length = tier4_autoware_utils::calcDistance3d(prev_pose, curr_pose);
    accumulated_length += segment_length;
    if (accumulated_length > stop_distance) {
      insert_idx = i;
      const double ratio = 1 - (accumulated_length - stop_distance) / segment_length;
      stop_pose = lerpByPose(prev_pose, curr_pose, ratio);
      break;
    }
  }

  // insert stop point
  TrajectoryPoint stop_point;
  stop_point.longitudinal_velocity_mps = 0.0;
  stop_point.lateral_velocity_mps = 0.0;
  stop_point.pose = stop_pose;
  trajectory_with_stop_point.insert(trajectory_with_stop_point.begin() + insert_idx, stop_point);
  // set 0 velocity to points after the stop point
  for (size_t i = insert_idx; i < trajectory_with_stop_point.size(); i++) {
    trajectory_with_stop_point.at(insert_idx).longitudinal_velocity_mps = 0.0;
    trajectory_with_stop_point.at(insert_idx).lateral_velocity_mps = 0.0;
  }
  return trajectory_with_stop_point;
}

}  // namespace motion_planning