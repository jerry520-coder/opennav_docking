// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING__CONTROLLER_HPP_
#define OPENNAV_DOCKING__CONTROLLER_HPP_

#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace opennav_docking
{
/**
 * @class opennav_docking::Controller
 * @brief Default control law for approaching a dock target 【默认控制率 Default control law 】
 */
class Controller
{
public:
  /**
   * @brief Create a controller instance. Configure ROS 2 parameters.
   */
  explicit Controller(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /**
   * @brief Compute a velocity command using control law.
   * @param pose Target pose, in robot centric coordinates.
   * @param cmd Command velocity.
   * @param backward If true, robot will drive backwards to goal.
   * @returns True if command is valid, false otherwise.
   */

  /**
 * @brief 使用控制定律计算速度命令。
 * @param pose 目标姿态，以机器人中心坐标系表示。
 * @param cmd 命令速度。
 * @param backward 如果为真，机器人将向后行驶到达目标。
 * @returns 如果命令有效，则返回真，否则返回假。
 */
  bool computeVelocityCommand(
    const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd,
    bool backward = false);

protected:
  std::unique_ptr<nav2_graceful_controller::SmoothControlLaw> control_law_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__CONTROLLER_HPP_
