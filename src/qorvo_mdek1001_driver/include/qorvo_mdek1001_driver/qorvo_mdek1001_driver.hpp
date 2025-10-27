/**
 * Qorvo MDEK1001 Driver node definition.
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

#ifndef QORVO_MDEK1001_DRIVER__QORVO_MDEK1001_DRIVER_HPP
#define QORVO_MDEK1001_DRIVER__QORVO_MDEK1001_DRIVER_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <serial/serial.h>

#include <rclcpp/rclcpp.hpp>

#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <dua_hardware_interfaces/msg/uwb_anchor.hpp>
#include <dua_hardware_interfaces/msg/uwb_tag.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace dua_hardware_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace std_msgs::msg;

using namespace std_srvs::srv;

namespace qorvo_mdek1001_driver
{

/**
 * Drives a Qorvo MDEK1001 UWB module.
 */
class QorvoMDEK1001Driver : public dua_node::NodeBase
{
public:
  QorvoMDEK1001Driver(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~QorvoMDEK1001Driver();

private:
  /* Node initialization routines. */
  void init_parameters();
  void init_publishers();
  void init_services();

  /* Topic publishers. */
  rclcpp::Publisher<UWBTag>::SharedPtr uwb_tag_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;

  /* Service callbacks. */
  void enable_callback(
    const SetBool::Request::SharedPtr req,
    const SetBool::Response::SharedPtr res);

  /* Internal state and data. */
  std::atomic<bool> running_{false};
  std::shared_ptr<serial::Serial> serial_device_ = nullptr;

  /* Tag sampling thread and routine. */
  std::thread tag_thread_;
  void tag_routine();

  /* Node parameters. */
  bool autostart_ = false;
  std::string frame_prefix_ = "";
  std::string global_frame_ = "";
  std::array<double, 3> position_variance_ = {0.0, 0.0, 0.0};
  std::string serial_port_ = "";
  int64_t tag_id_ = 0;
  int64_t timeout_ = 0;

  /* Auxiliary routines. */
  std::vector<std::string> split(const std::string & str, char delimiter);
  bool validate_position_variance(const rclcpp::Parameter & p);
};

} // namespace qorvo_mdek1001_driver

#endif // QORVO_MDEK1001_DRIVER__QORVO_MDEK1001_DRIVER_HPP
