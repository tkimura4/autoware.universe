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

  TrajectoryPoints output_trajectory_points =
    tier4_autoware_utils::convertToTrajectoryPointArray(*msg);

  // trim trajectory from self pose
  const auto base_trajectory =
    trimTrajectoryWithIndexFromSelfPose(output_trajectory_points, *current_pose_);
  // extend trajectory to consider objects after the goal
  const auto extend_trajectory = extendTrajectory(base_trajectory, param_.trajectory_extend_length);
  // decimate trajectory for calculation cost
  const auto decimate_trajectory =
    decimateTrajectory(extend_trajectory, param_.decimate_step_length);

  if (decimate_trajectory.size() < 2) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "trajectory is too short.");
    pub_trajectory_->publish(*msg);
  }

  // create rough polygon
  std::vector<Polygon2d> rough_detection_area;
  createPolygonFromTrajectoryPoints(
    decimate_trajectory, param_.rough_detection_area_expand_length, rough_detection_area);
  // extract objects in rough detection area
  PredictedObjects objects_in_rough_detection_area;
  extractObjectsinPolygon(*current_objects_ptr_, objects_in_rough_detection_area);

  // extract target objects of adaptive cruise controller
  PredictedObjects target_objects;
  getTargetObjects(objects_in_rough_detection_area, target_objects);

  if (target_objects.objects.empty()) {
    // no target object
    pub_trajectory_->publish(*msg);
  }

  // get nearest object from target object
  PredictedObject nearest_target_object;
  getNearestObject(target_objects, nearest_target_object);

  // send velocity / insert velocity to trajectory
  Trajectory output;
  {
    // get information for adaptive cruise control
    controller_ptr_->getInformationForAdaptiveCruise(
      *msg, *current_pose_, *current_odometry_ptr_, nearest_target_object);

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
  pub_trajectory_->publish(output);
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

void extractObjectsinPolygon(
  const PredictedObjects & objects, PredictedObjects & objects_in_polygon)
{
  // TODO
}

void getTargetObjects(const PredictedObjects & objects, PredictedObjects & target_objects)
{
  // TODO
  // if target is in detection area & diff angle with trajectory is low(30deg)? -> true

  // false then,

  // if velocity is high and predicted path is near the trajectory and diff angle with trajectory is
  // low(5deg)?
}

void getNearestObject(const PredictedObjects & objects, PredictedObject & nearest_object)
{
  // TODO
  // calc distance to object using trajectory
  // hear, rough distance is calculted by object-center-position.
  // (object-corner is not used here.)
}

void AdaptiveCruiseControllerNode::convexHull(
  const std::vector<cv::Point2d> & points, std::vector<cv::Point2d> & polygon_points)
{
  cv::Point2d centroid;
  centroid.x = 0;
  centroid.y = 0;
  for (const auto & point : points) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x = centroid.x / static_cast<double>(points.size());
  centroid.y = centroid.y / static_cast<double>(points.size());

  std::vector<cv::Point> normalized_points;
  std::vector<cv::Point> normalized_polygon_points;
  for (size_t i = 0; i < points.size(); ++i) {
    // 1mm-order
    normalized_points.push_back(
      cv::Point((points.at(i).x - centroid.x) * 1000.0, (points.at(i).y - centroid.y) * 1000.0));
  }
  cv::convexHull(normalized_points, normalized_polygon_points);

  for (size_t i = 0; i < normalized_polygon_points.size(); ++i) {
    cv::Point2d polygon_point;
    polygon_point.x = (normalized_polygon_points.at(i).x / 1000.0 + centroid.x);
    polygon_point.y = (normalized_polygon_points.at(i).y / 1000.0 + centroid.y);
    polygon_points.push_back(polygon_point);
  }
}

bool AdaptiveCruiseControllerNode::withinPolygon(
  const std::vector<Polygon2d> & trajectory_polygons, const Polygon2d & object_polygon)
{
  Polygon2d boost_trajectory_polygon;

  for (const auto & trajectory_polygon : trajectory_polygons) {
    if (bg::within(trajectory_polygon, object_polygon)) {
      return true;
    }
  }
  return false;
}

void AdaptiveCruiseControllerNode::convertObjectToBoostPolygon(
  const PredictedObject & object, Polygon2d object_polygon)
{
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const double yaw = tier4_autoware_utils::normalizeRadian(tf2::getYaw(pose.orientation));
    Eigen::Matrix2d rotation;
    rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    Eigen::Vector2d offset0, offset1, offset2, offset3;
    offset0 = rotation *
              Eigen::Vector2d(object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
    offset1 = rotation *
              Eigen::Vector2d(object.shape.dimensions.x * 0.5f, -object.shape.dimensions.y * 0.5f);
    offset2 = rotation *
              Eigen::Vector2d(-object.shape.dimensions.x * 0.5f, -object.shape.dimensions.y * 0.5f);
    offset3 = rotation *
              Eigen::Vector2d(-object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
    object_polygon.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset0.x(), pose.position.y + offset0.y()));
    object_polygon.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset1.x(), pose.position.y + offset1.y()));
    object_polygon.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset2.x(), pose.position.y + offset2.y()));
    object_polygon.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
      pose.position.x + offset3.x(), pose.position.y + offset3.y()));
    object_polygon.outer().push_back(object_polygon.outer().front());
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    const auto & center = object.kinematics.initial_pose_with_covariance.pose;
    const auto & radius = object.shape.dimensions.x * 0.5;
    constexpr int n = 6;
    for (int i = 0; i < n; ++i) {
      Eigen::Vector2d point;
      point.x() = std::cos(
                    (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                    M_PI / static_cast<double>(n)) *
                    radius +
                  center.position.x;
      point.y() = std::sin(
                    (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                    M_PI / static_cast<double>(n)) *
                    radius +
                  center.position.y;
      object_polygon.outer().push_back(
        boost::geometry::make<tier4_autoware_utils::Point2d>(point.x(), point.y()));
    }
    object_polygon.outer().push_back(object_polygon.outer().front());
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    for (const auto & point : object.shape.footprint.points) {
      object_polygon.outer().push_back(boost::geometry::make<tier4_autoware_utils::Point2d>(
        pose.position.x + point.x, pose.position.y + point.y));
    }
    object_polygon.outer().push_back(object_polygon.outer().front());
  }
  object_polygon = isClockWise(object_polygon) ? object_polygon : inverseClockWise(object_polygon);
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

void AdaptiveCruiseControllerNode::convertcvPointsToBoostPolygon(
  const std::vector<cv::Point2d> & points, Polygon2d boost_polygon)
{
  for (const auto & point : points) {
    boost_polygon.outer().push_back(bg::make<Point2d>(point.x, point.y));
  }
  boost_polygon.outer().push_back(bg::make<Point2d>(points.front().x, points.front().y));
}

}  // namespace motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::AdaptiveCruiseControllerNode)