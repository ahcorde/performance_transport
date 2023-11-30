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
#include "performance_transport/utils/SystemDataCollector.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  if (argc < 2) {
    std::cout << "Usage publish_image <filename> " << std::endl;
    return -1;
  }

  std::shared_ptr<performance_transport::PublisherImageTransport> pit =
    std::make_shared<performance_transport::PublisherImageTransport>(
    options);

  pit->declare_parameter("filename", "");
  std::string filename;
  pit->get_parameter("filename", filename);
  pit->SetFilename(filename);

  pit->declare_parameter("rosbag_topic", "");
  std::string rosbag_topic;
  pit->get_parameter("rosbag_topic", rosbag_topic);
  pit->SetRosBag(rosbag_topic);

  pit->declare_parameter("output_name", "");
  std::string output_name;
  pit->get_parameter("output_name", output_name);

  pit->declare_parameter("size", 4096);
  int size = 0;
  pit->get_parameter("size", size);

  pit->declare_parameter("compress", 0);
  int compress = 0;
  pit->get_parameter("compress", compress);

  pit->declare_parameter("compress_type", "");
  std::string compress_type{""};
  pit->get_parameter("compress_type", compress_type);
  pit->SetCompressType(compress_type);

  pit->declare_parameter("loop_time", 300);
  int loop_time{300};
  pit->get_parameter("loop_time", loop_time);

  pit->declare_parameter("transport_hint", "raw");
  std::string transport_hint{""};
  pit->get_parameter("transport_hint", transport_hint);

  std::cout << "Filename " << filename << std::endl;
  std::cout << "size " << size << std::endl;
  std::cout << "compress " << compress << std::endl;
  std::cout << "compress_type " << compress_type << std::endl;
  std::cout << "transport_hint " << transport_hint << std::endl;
  std::cout << "loop_time " << loop_time << std::endl;

  pit->Initialize();

  rclcpp::WallRate loop_rate(30);

  pit->SetImageSize(size, size);
  pit->SetCompressJpeg(compress);

  std::string filenameStats = "publisher_data" + std::string("_") +
    output_name + std::string("_") +
    transport_hint;

  std::string filenameSystemData = "publisher_data_cpu_mem" + std::string("_") +
    output_name + std::string("_") +
    transport_hint;

  if (transport_hint == "compressed") {
    filenameStats += std::string("_") + compress_type;
    filenameStats += std::string("_") + std::to_string(compress);

    filenameSystemData += std::string("_") + compress_type;
    filenameSystemData += std::string("_") + std::to_string(compress);
  }

  if (transport_hint == "zstd" || transport_hint == "avif") {
    filenameStats += std::string("_") + std::to_string(compress);
    filenameSystemData += std::string("_") + std::to_string(compress);
  }

  filenameStats += ".csv";
  filenameSystemData += ".csv";

  performance_transport::DataCollector dataCollector(filenameStats);

  performance_transport::SystemDataCollector systemDataCollector =
    performance_transport::SystemDataCollector(
    filenameSystemData,
    pit->get_clock());

  auto start_loop = std::chrono::high_resolution_clock::now();
  auto start = std::chrono::high_resolution_clock::now();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pit);

  while (rclcpp::ok()) {
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> elapsed = finish - start;

    if (elapsed.count() > 1) {
      double fps = static_cast<double>(pit->GetNumberOfImagesPublished()) /
        static_cast<double>(elapsed.count());
      std::cout << "fps: " << fps << std::endl;
      // std::cout << size << " " << size << " published in " << loop_time << " seconds: "
      //           << fps << " " << elapsed.count() << " compress " << compress << std::endl;
      dataCollector.WriteLine(std::to_string(fps));
      start = finish;
    }

    finish = std::chrono::high_resolution_clock::now();
    elapsed = finish - start_loop;

    if (elapsed.count() > loop_time) {
      break;
    }
    loop_rate.sleep();
    executor.spin_some();
  }

  systemDataCollector.Close();
  dataCollector.Close();

  pit->Destroy();

  rclcpp::shutdown();
  return 0;
}
