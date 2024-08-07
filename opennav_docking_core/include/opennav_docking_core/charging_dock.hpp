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

#ifndef OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_
#define OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"


namespace opennav_docking_core
{

/**
 * @class ChargingDock
 * @brief Abstract interface for a charging dock for the docking framework
 */
class ChargingDock
{
public:
  using Ptr = std::shared_ptr<ChargingDock>;

  /**
   * @brief Virtual destructor
   */
  virtual ~ChargingDock() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive Behavior and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */

  /**
 * @brief 获取充电桩的预设姿态的方法。此方法通常应该使用TF（坐标变换）和充电桩的姿态信息，以从静态或参数化的预设姿态中计算出相对于充电姿态的预设姿态。
 * @param pose 充电桩姿态
 * @param frame 充电桩的姿态坐标系
 * @return 指定坐标系中预设姿态的 PoseStamped 对象
 */
  virtual geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame) = 0;

  /**
   * @brief Method to obtain the refined pose of the dock, usually based on sensors
   * @param pose The initial estimate of the dock pose.
   * @param frame The frame of the initial estimate.
   */

  /**
 * @brief 获取充电底座精确位置的方法，通常基于传感器
 * @param pose 充电底座位置的初步估计。
 * @param frame 初步估计的位置框架。
 */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose) = 0;

  /**
   * @brief Have we made contact with dock? This can be implemented in a variety
   * of ways: by establishing communications with the dock, by monitoring the
   * the drive motor efforts, etc.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */

  /**
 * @brief 我们与充电底座接触了吗？这可以通过多种方式实现：
 * 通过与充电底座建立通信，监测驱动电机的工作情况等。
 *
 * 注意：此函数预期应快速返回。在此处阻塞将会阻塞对接控制器的循环。
 */
  virtual bool isDocked() = 0;

  /**
   * @brief Are we charging? If a charge dock requires any sort of negotiation
   * to begin charging, that should happen inside this function as this function
   * will be called repeatedly after the docking loop to check if successful.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */

  /**
 * @brief 我们正在充电吗？如果充电底座需要进行任何形式的协商以开始充电，
 * 这些操作应在此函数内进行，因为在对接循环后会反复调用此函数以检查是否成功。
 *
 * 注意：此函数预期应快速返回。在此处阻塞将会阻塞对接控制器的循环。
 */
  virtual bool isCharging() = 0;

  /**
   * @brief Undocking while current is still flowing can damage a charge dock
   * so some charge docks provide the ability to disable charging before the
   * robot physically disconnects. The undocking action will not command the
   * robot to move until this returns true.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */

  /**
 * @brief 在电流仍在流动时进行脱离操作可能会损坏充电底座，
 * 因此某些充电底座提供了在机器人物理断开连接之前禁用充电的功能。
 * 只有当此函数返回true时，脱离操作才会命令机器人移动。
 *
 * 注意：此函数预期应快速返回。在此处阻塞将会阻塞对接控制器的循环。
 */
  virtual bool disableCharging() = 0;

  /**
   * @brief Similar to isCharging() but called when undocking.
   */

  /**
 * @brief 类似于isCharging()，但在脱离操作时调用。
 */
  virtual bool hasStoppedCharging() = 0;

  std::string getName() {return name_;}

protected:
  std::string name_;
};

}  // namespace opennav_docking_core

#endif  // OPENNAV_DOCKING_CORE__CHARGING_DOCK_HPP_
