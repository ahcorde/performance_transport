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

#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "performance_transport/image_transport/PublisherImageTransport.hpp"
#include "performance_transport/utils/DataCollector.hpp"
#include "performance_transport/utils/ProcessInfo.hpp"

using namespace std::chrono_literals;

bool stop = false;

void statistics(std::string _filename, rclcpp::Clock::SharedPtr clock)
{
  performance_transport::DataCollector dataCollector(_filename);
  dataCollector.WriteLine("timestamp, uptime, cpuusage, memory, AnonMemory, VM");
  rclcpp::WallRate loop_rate(1);
  ProcessInfo pinfo(getpid());

  while (rclcpp::ok() && !stop) {
    loop_rate.sleep();
    pinfo.GetProcessMemoryUsed();
    dataCollector.WriteLine(
      std::to_string(clock->now().seconds()) + "," +
      std::to_string(pinfo.GetProcessUptime()) + "," +
      std::to_string(pinfo.GetProcessCPUUsage()) + "," +
      std::to_string(pinfo.GetMemUsed()) + "," +
      std::to_string(pinfo.GetMemAnonUsed()) + "," +
      std::to_string(pinfo.GetMemVmUsed()));
  }
  dataCollector.Close();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  if (argc < 2) {
    std::cout << "Usage publish_image <filename> " << std::endl;
    return -1;
  }

  std::string filename = argv[1];

  std::shared_ptr<performance_transport::PublisherImageTransport> pit =
    std::make_shared<performance_transport::PublisherImageTransport>(options, filename);

  pit->Initialize();

  int loop_time = 1;  // seconds

  rclcpp::WallRate loop_rate(30);

  int size = std::atoi(argv[2]);

  pit->SetImageSize(size, size);

  int compress = 95;

  pit->SetCompressJpeg(compress);

  performance_transport::DataCollector dataCollector("publisher_data" + std::string("_") +
    std::to_string(size) + std::string("_") +
    std::to_string(size) + ".csv");

  auto start = std::chrono::high_resolution_clock::now();

  std::thread t1(statistics,
    "publisher_data_cpu_mem" + std::string("_") +
    std::to_string(size) + std::string("_") +
    std::to_string(size) + ".csv",
    pit->get_clock());

  while (rclcpp::ok()) {
    loop_rate.sleep();

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = finish - start;

    if (elapsed.count() > loop_time) {
      double fps = static_cast<double>(pit->GetNumberOfImagesPublished()) /
        static_cast<double>(elapsed.count());
      std::cout << size << " " << size << " published in " << loop_time << " seconds: " << fps <<
        " " << elapsed.count() << " compress " << compress << std::endl;

      dataCollector.WriteLine(
        std::to_string(fps) + std::string(",") +
        std::to_string(compress));

      compress -= 10;
      if (compress < 0) {
        break;
      }
      pit->SetCompressJpeg(compress);
      start = finish;
    }
    rclcpp::spin_some(pit);
  }
  stop = true;
  pit->SetCompressJpeg(-5);
  t1.join();

  rclcpp::shutdown();
}
