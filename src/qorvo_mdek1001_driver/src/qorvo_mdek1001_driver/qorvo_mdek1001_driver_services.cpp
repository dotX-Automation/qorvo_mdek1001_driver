/**
 * Qorvo MDEK1001 Driver node service callbacks.
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
 * @brief Enable service callback.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void QorvoMDEK1001Driver::enable_callback(
  const SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    bool expected = false;
    if (running_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      // Start camera sampling thread
      tag_thread_ = std::thread(
        &QorvoMDEK1001Driver::tag_routine,
        this);
    }
  } else {
    bool expected = true;
    if (running_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      tag_thread_.join();
      RCLCPP_INFO(this->get_logger(), "Tag thread joined");
    }
  }
  resp->set__success(true);
  resp->set__message("");
}

} // namespace qorvo_mdek1001_driver
