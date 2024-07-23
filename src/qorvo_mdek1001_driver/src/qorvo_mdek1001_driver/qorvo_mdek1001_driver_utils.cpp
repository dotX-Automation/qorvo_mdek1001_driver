/**
 * Qorvo MDEK1001 Driver auxiliary utilities.
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
 * @brief Splits a string into tokens.
 *
 * @param str String to split.
 * @param delimiter Delimiter character.
 * @return Vector of string tokens.
 */
std::vector<std::string> QorvoMDEK1001Driver::split(const std::string & str, char delimiter)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream token_stream(str);

  while (std::getline(token_stream, token, delimiter)) {
    tokens.push_back(token);
  }

  return tokens;
}

/**
 * @brief Validates the position_variance parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool QorvoMDEK1001Driver::validate_position_variance(const rclcpp::Parameter & p)
{
  // Check that the parameter is a 3-element vector
  if (p.as_double_array().size() != 3) {
    return false;
  }

  // Update entries one by one to avoid copies and memory access violations
  for (int i = 0; i < 3; i++) {
    position_variance_[i] = p.as_double_array()[i];
  }
  return true;
}

} // namespace qorvo_mdek1001_driver
