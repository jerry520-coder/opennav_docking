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

#include "opennav_docking/pose_filter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace opennav_docking
{

PoseFilter::PoseFilter(double coef, double timeout)
{
  coef_ = coef;
  timeout_ = timeout;
  pose_.header.stamp = rclcpp::Time(0);
}

geometry_msgs::msg::PoseStamped
PoseFilter::update(const geometry_msgs::msg::PoseStamped & measurement)
{
  // 如果滤波系数小于等于 0.0，不进行滤波，直接返回测量值
  if (coef_ <= 0.0) {
    return measurement;
  }

  // 如果测量时间戳与上次保存的姿态时间戳的差值超过了设定的超时阈值，则直接使用新的测量值
  if ((rclcpp::Time(measurement.header.stamp) - pose_.header.stamp).seconds() > timeout_) {
    pose_ = measurement;
  } else if (pose_.header.frame_id != measurement.header.frame_id) {
    // 如果测量的坐标系与当前姿态的坐标系不同，直接使用新的测量值
    pose_ = measurement;
  } else {
    // 否则，进行滤波处理

    // 复制头部信息
    pose_.header = measurement.header;

    // 对位置进行滤波处理
    filter(pose_.pose.position.x, measurement.pose.position.x);
    filter(pose_.pose.position.y, measurement.pose.position.y);
    filter(pose_.pose.position.z, measurement.pose.position.z);

    // 对方向进行滤波处理
    tf2::Quaternion f_quat, m_quat;
    tf2::fromMsg(measurement.pose.orientation, m_quat); // 将测量的方向转换为四元数
    tf2::fromMsg(pose_.pose.orientation, f_quat);      // 将当前姿态的方向转换为四元数
    f_quat = f_quat.slerp(m_quat, coef_);             // 使用球面线性插值进行方向的滤波
    pose_.pose.orientation = tf2::toMsg(f_quat);      // 将滤波后的四元数转换回消息格式
  }

  return pose_; // 返回滤波后的姿态
}

void PoseFilter::filter(double & filt, double meas)
{
  filt = (1 - coef_) * filt + coef_ * meas;
}

}  // namespace opennav_docking
