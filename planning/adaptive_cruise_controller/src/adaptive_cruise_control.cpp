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

#include <adaptive_cruise_controller/adaptive_cruise_control.hpp>
#include <adaptive_cruise_controller/adaptive_cruise_pid_controller.hpp>
#include <adaptive_cruise_controller/debug_marker.hpp>

namespace motion_planning
{
AdaptiveCruiseControllerNode::AdaptiveCruiseControllerNode(const rclcpp::NodeOptions & options)
: Node("adaptive_cruise_controller", options), self_pose_listener_(this)
{
  using std::placeholders::_1;

  // set param
  // TODO

  // set acc param
  // TODO

  // vehicle parameters
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  // controller
  controller_ptr_ =
    std::make_shared<AdaptiveCruisePIDController>(vehicle_info_.front_overhang_m, acc_param_);

  // wait for self pose
  self_pose_listener_.waitForFirstPose();

  // publishers, subscribers
  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);

  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&AdaptiveCruiseControllerNode::onTrajectory, this, _1));
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&AdaptiveCruiseControllerNode::onOdometry, this, _1));
  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, std::bind(&AdaptiveCruiseControllerNode::onPredictedObjects, this, _1));
}

void AdaptiveCruiseControllerNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  current_pose_ = self_pose_listener_.getCurrentPose();
  if (!isDataReady()) {
    return;
  }

  TrajectoryPoints trajectory_points = tier4_autoware_utils::convertToTrajectoryPointArray(*msg);

  // trim trajectory from self pose
  const auto base_trajectory =
    trimTrajectoryWithIndexFromSelfPose(trajectory_points, *current_pose_);
  // extend trajectory to consider objects after the goal
  const auto extend_trajectory = extendTrajectory(base_trajectory, param_.trajectory_extend_length);
  // decimate trajectory for calculation cost
  const auto decimate_trajectory =
    decimateTrajectory(extend_trajectory, param_.decimate_step_length);

  if (decimate_trajectory.size() < 2) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "trajectory is too short.");
    pub_trajectory_->publish(*msg);
  }

  // create rough detection area
  std::vector<Polygon2d> rough_detection_area;
  createPolygonFromTrajectoryPoints(
    decimate_trajectory, param_.rough_detection_area_expand_length, rough_detection_area);
  // extract objects in rough detection area
  PredictedObjects objects_in_rough_detection_area;
  extractVehicleObjectsInPolygon(
    *current_objects_ptr_, rough_detection_area, objects_in_rough_detection_area);

  // create detection area
  std::vector<Polygon2d> detection_area;
  createPolygonFromTrajectoryPoints(
    decimate_trajectory, param_.detection_area_expand_length, detection_area);

  // extract target objects of adaptive cruise controller
  PredictedObjects target_objects;
  getTargetObjects(
    objects_in_rough_detection_area, detection_area, decimate_trajectory, target_objects);

  if (target_objects.objects.empty()) {
    // no target object
    pub_trajectory_->publish(*msg);
  }

  // get nearest object from target object
  PredictedObject nearest_target_object;
  getNearestObject(target_objects, decimate_trajectory, nearest_target_object);

  // send velocity / insert velocity to trajectory
  TrajectoryPoints output;
  {
    // get information for adaptive cruise control
    const auto object_time = rclcpp::Time(target_objects.header.stamp);
    controller_ptr_->calcInformationForAdaptiveCruise(
      trajectory_points, *current_pose_, *current_odometry_ptr_, nearest_target_object,
      object_time);

    // calculate target motion (target_velocity, and trajectory)
    controller_ptr_->calculate();

    // send target velocity
    double target_velocity;
    double target_acceleration;
    double target_jerk;
    if (controller_ptr_->getTargetMotion(target_velocity, target_acceleration, target_jerk)) {
      pub_veloity_limit_->publish(
        createVelocityLimitMsg(target_velocity, target_acceleration, target_jerk));
    }

    // get trajectory (If necessary, insert stop velocity.)
    bool emergency;
    output = controller_ptr_->getAccTrajectory(emergency);
  }

  // publish trajectory
  auto output_trajectory = tier4_autoware_utils::convertToTrajectory(output);
  output_trajectory.header = msg->header;
  pub_trajectory_->publish(output_trajectory);
}

void AdaptiveCruiseControllerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  current_odometry_ptr_ = msg;
}

void AdaptiveCruiseControllerNode::onPredictedObjects(const PredictedObjects::ConstSharedPtr msg)
{
  current_objects_ptr_ = msg;
}

bool AdaptiveCruiseControllerNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  if (!current_odometry_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "waiting for odometry msg...");
    return false;
  }

  if (!current_objects_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "waiting for objects msg...");
    return false;
  }

  return true;
}

VelocityLimit AdaptiveCruiseControllerNode::createVelocityLimitMsg(
  const double velocity, const double acceleration, const double jerk)
{
  VelocityLimit msg;
  msg.stamp = this->now();
  msg.sender = "adaptive_cruise_controller";
  msg.max_velocity = velocity;
  msg.use_constraints = true;
  msg.constraints.min_acceleration = std::min(0.0, acceleration);
  msg.constraints.min_jerk = std::min(0.0, jerk);
  msg.constraints.max_jerk = -msg.constraints.min_jerk;
  return msg;
}

VelocityLimitClearCommand AdaptiveCruiseControllerNode::createVelocityLimitClearCommandMsg()
{
  VelocityLimitClearCommand msg;
  msg.stamp = this->now();
  msg.sender = "adaptive_cruise_controller";
  msg.command = true;
  return msg;
}

void AdaptiveCruiseControllerNode::createPolygonFromTrajectoryPoints(
  const TrajectoryPoints & trajectory_points, const double expand_width,
  std::vector<Polygon2d> & polygons)
{
  std::vector<cv::Point2d> vehicle_corner_points;

  const auto & i = vehicle_info_;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0 + expand_width;
  const auto & back_m = i.rear_overhang_m;

  for (size_t i = 0; i < trajectory_points.size() - 1; i++) {
    std::vector<cv::Point2d> polygon;
    vehicle_corner_points.clear();
    // create convex hull polygon from trajectory points i and i+1
    for (size_t j = i; j <= i + 1; j++) {
      const auto point = trajectory_points.at(i);
      const auto yaw = tf2::getYaw(point.pose.orientation);
      vehicle_corner_points.push_back(cv::Point2d(
        point.pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
        point.pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
      vehicle_corner_points.push_back(cv::Point2d(
        point.pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
        point.pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
      vehicle_corner_points.push_back(cv::Point2d(
        point.pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
        point.pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
      vehicle_corner_points.push_back(cv::Point2d(
        point.pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
        point.pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
    }
    convexHull(vehicle_corner_points, polygon);
    // convert cv-polygon to boost-polygon
    Polygon2d boost_polygon;
    convertcvPointsToBoostPolygon(polygon, boost_polygon);
    polygons.emplace_back(boost_polygon);
  }
}

void AdaptiveCruiseControllerNode::extractVehicleObjectsInPolygon(
  const PredictedObjects & objects, const std::vector<Polygon2d> & target_polygons,
  PredictedObjects & objects_in_polygon)
{
  objects_in_polygon.header = objects.header;
  for (const auto object : objects.objects) {
    if (!isTargetObjectType(object)) {
      // the object is not target vehicle type
      continue;
    }

    Polygon2d object_polygon;
    convertObjectToBoostPolygon(object, object_polygon);
    if (isObjectWithinPolygon(target_polygons, object_polygon)) {
      objects_in_polygon.objects.emplace_back(object);
    }
  }
}

void AdaptiveCruiseControllerNode::getTargetObjects(
  const PredictedObjects & objects, const std::vector<Polygon2d> & target_polygons,
  const TrajectoryPoints & target_trajectory, PredictedObjects & target_objects)
{
  target_objects.header = objects.header;

  for (const auto object : objects.objects) {
    // check if the object is in the target area
    // and if the object angle is aligned with the trajectory angle.
    Polygon2d object_polygon;
    convertObjectToBoostPolygon(object, object_polygon);
    bool object_in_target_area = isObjectWithinPolygon(target_polygons, object_polygon);
    bool align_object_angle = isAngleAlignedWithTrajectory(
      getObjectPose(object), target_trajectory, param_.object_threshold_angle);

    if (object_in_target_area && align_object_angle) {
      target_objects.objects.emplace_back(object);
      continue;
    }

    // check if the object velocity is high
    // and if the predicted_path and trajectory overlap for a certain period of time.
    bool object_velocity_high = isObjectVelocityHigh(object);
    double threshold_dist_of_path_and_trajectory = vehicle_info_.vehicle_width_m / 2.0 +
                                                   object.shape.dimensions.y / 2.0 +
                                                   param_.detection_area_expand_length;
    bool overlap_predicted_path_with_trajectory = isPathNearTrajectory(
      getHighestConfidencePathFromObject(object), target_trajectory,
      threshold_dist_of_path_and_trajectory, param_.predicted_path_threshold_angle,
      param_.mininum_overlap_time_of_predicted_path);

    if (object_velocity_high && overlap_predicted_path_with_trajectory) {
      target_objects.objects.emplace_back(object);
      continue;
    }
  }
}

void AdaptiveCruiseControllerNode::getNearestObject(
  const PredictedObjects & objects, const TrajectoryPoints & trajectory_points,
  PredictedObject & nearest_object)
{
  double minimum_length_to_object = std::numeric_limits<double>::max();

  for (const auto object : objects.objects) {
    const auto object_point = object.kinematics.initial_pose_with_covariance.pose.position;

    const auto length_to_object = tier4_autoware_utils::calcSignedArcLength(
      trajectory_points, current_pose_->pose.position, object_point);

    if (length_to_object < minimum_length_to_object) {
      minimum_length_to_object = length_to_object;
      nearest_object = object;
    }
  }
}

PredictedPath AdaptiveCruiseControllerNode::getHighestConfidencePathFromObject(
  const PredictedObject & object)
{
  double highest_confidence = 0;
  PredictedPath highest_confidence_path;

  for (const auto path : object.kinematics.predicted_paths) {
    if (highest_confidence < path.confidence) {
      highest_confidence = path.confidence;
      highest_confidence_path = path;
    }
  }
  return highest_confidence_path;
}

geometry_msgs::msg::Pose AdaptiveCruiseControllerNode::getObjectPose(const PredictedObject & object)
{
  if (object.kinematics.initial_twist_with_covariance.twist.linear.x >= 0) {
    return object.kinematics.initial_pose_with_covariance.pose;
  }

  // If the object velocity is negative, invert yaw-angle
  auto obj_pose = object.kinematics.initial_pose_with_covariance.pose;
  double yaw, pitch, roll;
  tf2::getEulerYPR(obj_pose.orientation, yaw, pitch, roll);
  tf2::Quaternion inv_q;
  inv_q.setRPY(roll, pitch, yaw + M_PI);
  obj_pose.orientation = tf2::toMsg(inv_q);
  return obj_pose;
}

bool AdaptiveCruiseControllerNode::isTargetObjectType(const PredictedObject & object)
{
  using Label = ObjectClassification;

  const auto object_label = object.classification.front().label;
  if (
    object_label == Label::BUS || object_label == Label::CAR || object_label == Label::TRAILER ||
    object_label == Label::MOTORCYCLE || object_label == Label::TRUCK) {
    return true;
  }
  return false;
}

bool AdaptiveCruiseControllerNode::isObjectVelocityHigh(const PredictedObject & object)
{
  const auto object_velocity = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  return object_velocity > acc_param_.object_low_velocity_thresh;
}

bool AdaptiveCruiseControllerNodeisClockWise(const Polygon2d & polygon)
{
  const auto n = polygon.outer().size();

  const double x_offset = polygon.outer().at(0).x();
  const double y_offset = polygon.outer().at(0).y();
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.outer().size(); ++i) {
    sum +=
      (polygon.outer().at(i).x() - x_offset) * (polygon.outer().at((i + 1) % n).y() - y_offset) -
      (polygon.outer().at(i).y() - y_offset) * (polygon.outer().at((i + 1) % n).x() - x_offset);
  }
  return sum < 0.0;
}

Polygon2d AdaptiveCruiseControllerNodeinverseClockWise(const Polygon2d & polygon)
{
  const auto & poly = polygon.outer();
  Polygon2d inverted_polygon;
  inverted_polygon.outer().reserve(poly.size());
  std::reverse_copy(poly.begin(), poly.end(), std::back_inserter(inverted_polygon.outer()));
  return inverted_polygon;
}

}  // namespace motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::AdaptiveCruiseControllerNode)