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

#include "performance_transport/utils/SystemDataCollector.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>
#include "performance_transport/utils/DataCollector.hpp"
#include "performance_transport/utils/ProcessInfo.hpp"

namespace performance_transport
{
SystemDataCollector::SystemDataCollector(
  const std::string & _filename, rclcpp::Clock::SharedPtr _clock)
: filename_(_filename), clock_(_clock)
{
  this->outputfile_.open(this->filename_);
  thread = std::thread(std::bind(&SystemDataCollector::loop, this));
}

void SystemDataCollector::loop()
{
  performance_transport::DataCollector dataCollector(this->filename_);
  dataCollector.WriteLine("timestamp, uptime, cpuusage, memory, anonmemory, vm");
  rclcpp::WallRate loop_rate(1);
  ProcessInfo pinfo(getpid());

  while (rclcpp::ok() && !this->stop_) {
    loop_rate.sleep();
    pinfo.GetProcessMemoryUsed();
    dataCollector.WriteLine(
      std::to_string(this->clock_->now().seconds()) + "," +
      std::to_string(pinfo.GetProcessUptime()) + "," +
      std::to_string(pinfo.GetProcessCPUUsage()) + "," +
      std::to_string(pinfo.GetMemUsed()) + "," +
      std::to_string(pinfo.GetMemAnonUsed()) + "," +
      std::to_string(pinfo.GetMemVmUsed()));
  }
  dataCollector.Close();
}

SystemDataCollector::~SystemDataCollector()
{
  this->Close();
}

void SystemDataCollector::Close()
{
  this->stop_ = true;
  thread.join();
  this->outputfile_.close();
}

}  // namespace performance_transport
