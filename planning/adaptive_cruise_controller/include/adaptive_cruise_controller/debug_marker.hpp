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
#ifndef ADAPTIVE_CRUISE_CONTROLLER__DEBUG_MARKER_HPP_
#define ADAPTIVE_CRUISE_CONTROLLER__DEBUG_MARKER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace motion_planning
{

using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using visualization_msgs::msg::MarkerArray;

class DebugValues
{
public:
  enum class TYPE {
    CURRENT_VEL = 0,
    CURRENT_ACC = 1,
    CURRENT_OBJECT_VEL_ = 2,
    CURRENT_OBJECT_VEL_POSITION_DIFF = 3,
    CURRENT_OBJECT_ACC = 4,
    CURRENT_OBJECT_DISTANCE = 5,
    IDEAL_OBJECT_DISTANCE = 6,
    RSS_OBJECT_DISTANCE = 7,
    FLAG_CUTIN_OBJECT = 8,
    FLAG_CUTOUT_OBJECT = 9,
    FLAG_FIND_OBJECT = 10,
    FLAG_ADAPTIVE_CRUISE = 11,
    FLAG_STOP = 12,
    CURRENT_TARGET_VELOCITY = 13,
    SIZE
  };

  /**
   * @brief get the index corresponding to the given value TYPE
   * @param [in] type the TYPE enum for which to get the index
   * @return index of the type
   */
  int getValuesIdx(const TYPE type) const { return static_cast<int>(type); }
  /**
   * @brief get all the debug values as an std::array
   * @return array of all debug values
   */
  std::array<double, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }
  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void setValues(const TYPE type, const double val) { values_.at(static_cast<int>(type)) = val; }
  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void setValues(const int type, const double val) { values_.at(type) = val; }

private:
  static constexpr int num_debug_values_ = static_cast<int>(TYPE::SIZE);
  std::array<double, static_cast<int>(TYPE::SIZE)> values_;
};

class AdaptiveCruiseControllerDebugNode
{
public:
  explicit AdaptiveCruiseControllerDebugNode(rclcpp::Node * node);
  void setDebugValues(const DebugValues::TYPE type, const double val)
  {
    debug_values_.setValues(type, val);
  }
  void setStopLine(const geometry_msgs::msg::Pose & stop_pose, const double baselink2front);
  void setPolygon(const std::vector<cv::Point2d> & points);
  void publishMarker();
  void publishDebugValues();

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_value_;
  DebugValues debug_values_;
};

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_
