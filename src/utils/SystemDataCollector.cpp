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
  thread = std::thread(std::bind(&SystemDataCollector::loop, this));
}

void SystemDataCollector::loop()
{
  performance_transport::DataCollector dataCollector(this->filename_);
  dataCollector.WriteLine("timestamp, uptime, cpuusage, memory, anonmemory, vm, "
                          "rmbytes, tmbytes, rpackets, tpackets");
  rclcpp::WallRate loop_rate(1);
  ProcessInfo pinfo(getpid());

  while (rclcpp::ok() && !this->stop_) {
    loop_rate.sleep();
    pinfo.GetProcessMemoryUsed();
    pinfo.GetNetworkStats();
    dataCollector.WriteLine(
      std::to_string(this->clock_->now().seconds()) + "," +
      std::to_string(pinfo.GetProcessUptime()) + "," +
      std::to_string(pinfo.GetProcessCPUUsage()) + "," +
      std::to_string(pinfo.GetMemUsed()) + "," +
      std::to_string(pinfo.GetMemAnonUsed()) + "," +
      std::to_string(pinfo.GetMemVmUsed()) + "," +
      std::to_string(pinfo.GetReceivedMbytes()) + "," +
      std::to_string(pinfo.GetTransmitedMbytes()) + "," +
      std::to_string(pinfo.GetReceivedPackets()) + "," +
      std::to_string(pinfo.GetTransmitedPackets()));
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
  if (thread.joinable())
    thread.join();
}

}  // namespace performance_transport

// int num_bytes_per_sec = 0;
// int max_bytes_per_sec = 0;
// long int max_bytes_per_sec_timestamp = 0;
// long int ref_time = 0;
// char msg_data[30000];

// while (!feof(fp))
//     {
//         // read next message size
//         int memsz;
//         read = fread (&memsz, sizeof(int), 1, fp);
//         if (read == 0)
//         {
//             if (ferror(fp))
//                 printf("error reading. %d\n", read);
//             break;
//         }
//         assert(read == 1);

//         // read the message
//         read = fread (msg_data, sizeof(char), memsz, fp);
//         assert(read = memsz);

//         // the foo method extracts the event time from the message, in ms
//         time_t event_time = 0;
//         foo (msg_data, &event_time);

//         // track bandwidth
//         if (ref_time == 0 || event_time - ref_time > 1000)
//         {
//             if (num_bytes_per_sec > max_bytes_per_sec)
//             {
//                 max_bytes_per_sec = num_bytes_per_sec;
//                 max_bytes_per_sec_timestamp = event_time;
//             }
//             num_bytes_per_sec = 0;
//             ref_time = event_time;
//         }
//         num_bytes_per_sec += sizeof(char) * strlen(msg_data);
// }