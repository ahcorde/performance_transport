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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/times.h>

#include <memory>
#include <string>

#include "performance_transport/point_cloud_transport/PublisherPointCloudTransport.hpp"

#include <rclcpp/rclcpp.hpp>

#include "performance_transport/utils/DataCollector.hpp"
#include "performance_transport/utils/SystemDataCollector.hpp"
#include "performance_transport/utils/ProcessInfo.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  if (argc < 2) {
    std::cout << "Usage publish_point_cloud <filename> " << std::endl;
    return -1;
  }
  std::string filename = argv[1];

  std::shared_ptr<performance_transport::PublisherPointCloudTransport> ppc =
    std::make_shared<performance_transport::PublisherPointCloudTransport>(options, filename);
  ppc->Initialize();

  rclcpp::WallRate loop_rate(30);

  int size = ppc->GetSize();

  performance_transport::SystemDataCollector systemDataCollector =
    performance_transport::SystemDataCollector(
    "publisher_point_cloud_data_cpu_mem" + std::string("_") +
    std::to_string(size) + ".csv",
    ppc->get_clock());

  performance_transport::DataCollector dataCollector(
    "publisher_point_cloud_data" + std::string("_") +
    std::to_string(size) + ".csv");

  int loop_time = 20;  // seconds

  auto start_loop = std::chrono::high_resolution_clock::now();
  auto start = std::chrono::high_resolution_clock::now();

  while (rclcpp::ok()) {
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = finish - start;
    if (elapsed.count() > 1) {
      double fps = static_cast<double>(ppc->GetNumberOfImagesPublished()) /
        static_cast<double>(elapsed.count());
      dataCollector.WriteLine(
        std::to_string(fps));
      start = finish;
    }

    finish = std::chrono::high_resolution_clock::now();
    elapsed = finish - start_loop;

    if (elapsed.count() > loop_time) {
      // break;
    }

    loop_rate.sleep();
    rclcpp::spin_some(ppc);
  }

  std::cout << "exit loop" << std::endl;

  systemDataCollector.Close();
  dataCollector.Close();
  std::cout << "exit loop2" << std::endl;

  ppc.reset();
  std::cout << "exit loop3" << std::endl;

  rclcpp::shutdown();
  std::cout << "exit loop4" << std::endl;

  return 0;
}
