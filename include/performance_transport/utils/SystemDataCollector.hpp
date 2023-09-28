// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef PERFORMANCE_TRANSPORT__UTILS__SYSTEMDATACOLLECTOR_HPP_
#define PERFORMANCE_TRANSPORT__UTILS__SYSTEMDATACOLLECTOR_HPP_

#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace performance_transport
{
/// This class blocks destruction when in use
class SystemDataCollector
{
public:
  SystemDataCollector() = default;
  explicit SystemDataCollector(
    const std::string & _filename, rclcpp::Clock::SharedPtr _clock);
  ~SystemDataCollector();
  void Close();
  void loop();

private:
  std::string filename_;
  rclcpp::Clock::SharedPtr clock_;
  bool stop_{false};
  std::thread thread;
};
}  // namespace performance_transport
#endif  // PERFORMANCE_TRANSPORT__UTILS__SYSTEMDATACOLLECTOR_HPP_
