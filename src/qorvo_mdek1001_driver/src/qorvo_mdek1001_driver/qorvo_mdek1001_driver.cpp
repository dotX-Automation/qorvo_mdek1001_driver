/**
 * Qorvo MDEK1001 Driver node implementation.
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
 * @brief Node constructor.
 *
 * @param opts Node options.
 */
QorvoMDEK1001Driver::QorvoMDEK1001Driver(const rclcpp::NodeOptions & opts)
: NodeBase("qorvo_mdek1001_driver", opts, true)
{
  init_parameters();
  init_publishers();
  init_services();

  RCLCPP_INFO(this->get_logger(), "Node initialized");

  // Start tag sampling thread, if requested
  if (autostart_)
  {
    running_.store(true, std::memory_order_release);
    tag_thread_ = std::thread(
      &QorvoMDEK1001Driver::tag_routine,
      this);
  }
}

/**
 * @brief Node destructor.
 */
QorvoMDEK1001Driver::~QorvoMDEK1001Driver()
{
  // Stop the tag sampling thread if it is active
  bool is_running = true;
  if (running_.compare_exchange_strong(
      is_running,
      false,
      std::memory_order_release,
      std::memory_order_acquire))
  {
    tag_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Tag thread joined");
  }
}

/**
 * @brief Initializes publishers.
 */
void QorvoMDEK1001Driver::init_publishers()
{
  // tag_data
  uwb_tag_pub_ = this->create_publisher<UWBTag>(
    "~/tag_data",
    dua_qos::Reliable::get_datum_qos());

  // tag_pose
  pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/tag_pose",
    dua_qos::Reliable::get_datum_qos());
}

/**
 * @brief Initializes services.
 */
void QorvoMDEK1001Driver::init_services()
{
  // enable
  enable_srv_ = this->create_service<SetBool>(
    "~/enable",
    std::bind(
      &QorvoMDEK1001Driver::enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

} // namespace qorvo_mdek1001_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(qorvo_mdek1001_driver::QorvoMDEK1001Driver)
