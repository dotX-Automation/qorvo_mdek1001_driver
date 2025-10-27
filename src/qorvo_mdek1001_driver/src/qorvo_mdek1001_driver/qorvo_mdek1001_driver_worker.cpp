/**
 * Qorvo MDEK1001 Driver worker thread routine.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 *
 * July 23, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <qorvo_mdek1001_driver/qorvo_mdek1001_driver.hpp>

namespace qorvo_mdek1001_driver
{

/**
 * @brief Tag sampling thread routine: parses and publishes data produced by the tag.
 *
 * @throws std::runtime_error if the tag device cannot be opened or it fails to respond.
 */
void QorvoMDEK1001Driver::tag_routine()
{
  // Open serial device
  serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_));
  try {
    serial_device_ = std::make_shared<serial::Serial>(
      serial_port_,
      115200U,
      timeout);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open serial device: %s", e.what());
    throw std::runtime_error("Failed to open serial device: " + std::string(e.what()));
  }

  size_t ret = 0;
  bool data_ready = false;
  const uint8_t * shell_mode = reinterpret_cast<const uint8_t *>("\x0D\x0D");
  const uint8_t * csv_mode = reinterpret_cast<const uint8_t *>("lec\n");
  const uint8_t * reset = reinterpret_cast<const uint8_t *>("reset\n");

  // Set shell mode and CSV output format
  ret = serial_device_->write(shell_mode, 2);
  if (ret != 2) {
    RCLCPP_FATAL(this->get_logger(), "Failed to set shell mode");
    throw std::runtime_error("Failed to set shell mode");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  ret = serial_device_->write(csv_mode, 4);
  if (ret != 4) {
    RCLCPP_FATAL(this->get_logger(), "Failed to set CSV mode");
    throw std::runtime_error("Failed to set CSV mode");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  RCLCPP_INFO(this->get_logger(), "Serial device ready");

  // Wait for output to start
  while (true) {
    data_ready = serial_device_->waitReadable();
    if (data_ready) {
      std::string line = serial_device_->readline(65535, "\r\n");
      if (line.substr(0, 4) == "DIST") {
        break;
      }
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for output to start...");
  }

  RCLCPP_WARN(this->get_logger(), "Tag sampling thread started");

  // Main loop
  while (running_.load(std::memory_order_acquire)) {
    data_ready = serial_device_->waitReadable();
    if (!data_ready) {
      continue;
    }
    std::string line = serial_device_->readline(65535, "\r\n");

    UWBTag tag_msg{};
    rclcpp::Time stamp = this->get_clock()->now();
    tag_msg.header.set__stamp(stamp);
    tag_msg.header.set__frame_id(frame_prefix_ + "uwb_" + std::to_string(tag_id_) + "_link");

    // Parse CSV line: start from anchor data
    std::vector<std::string> splits = split(line, ',');
    int n_anchors = std::stoi(splits[1]);
    tag_msg.set__n_anchors(n_anchors);
    splits.erase(splits.begin(), splits.begin() + 2);
    for (int i = 0; i < n_anchors; i++) {
      UWBAnchor anchor_msg{};
      anchor_msg.header.set__stamp(stamp);
      anchor_msg.header.set__frame_id(global_frame_);

      std::vector<std::string> anchor_splits(splits.begin() + i * 6, splits.begin() + (i + 1) * 6);

      anchor_msg.set__id(std::stoi(anchor_splits[1], 0, 16));
      anchor_msg.set__id_str(anchor_splits[1]);
      anchor_msg.set__distance(std::stod(anchor_splits[5]));
      anchor_msg.position.set__x(std::stod(anchor_splits[2]));
      anchor_msg.position.set__y(std::stod(anchor_splits[3]));
      anchor_msg.position.set__z(std::stod(anchor_splits[4]));

      tag_msg.anchors.push_back(anchor_msg);
    }

    // Parse tag data, if present
    tag_msg.tag_position.header.set__stamp(stamp);
    tag_msg.tag_position.header.set__frame_id(global_frame_);
    if (splits.size() > static_cast<size_t>(n_anchors * 6)) {
      int i = n_anchors * 6 + 1;
      tag_msg.tag_position.point.set__x(std::stod(splits[i]));
      tag_msg.tag_position.point.set__y(std::stod(splits[i + 1]));
      tag_msg.tag_position.point.set__z(std::stod(splits[i + 2]));
      tag_msg.set__quality_factor(std::stoi(splits[i + 3]));

      // Publish tag pose
      PoseWithCovarianceStamped pose_msg{};
      pose_msg.header.set__stamp(stamp);
      pose_msg.header.set__frame_id(global_frame_);
      pose_msg.pose.pose.set__position(tag_msg.tag_position.point);
      pose_msg.pose.covariance[0] = position_variance_[0];
      pose_msg.pose.covariance[7] = position_variance_[1];
      pose_msg.pose.covariance[14] = position_variance_[2];
      pose_pub_->publish(pose_msg);
    } else {
      tag_msg.set__quality_factor(-1);
    }

    // Publish tag data
    uwb_tag_pub_->publish(tag_msg);
  }

  // Reset UWB device and close serial device
  ret = serial_device_->write(reset, 6);
  if (ret != 6) {
    RCLCPP_FATAL(this->get_logger(), "Failed to reset UWB device");
    throw std::runtime_error("Failed to reset UWB device");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  serial_device_->close();
  serial_device_.reset();

  RCLCPP_WARN(this->get_logger(), "Tag sampling thread stopped");
}

} // namespace qorvo_mdek1001_driver
