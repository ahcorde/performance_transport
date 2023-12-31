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
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "performance_transport/image_transport/SubscriberImageTransport.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<performance_transport::SubscriberImageTransport> node =
    std::make_shared<performance_transport::SubscriberImageTransport>(options);

  node->declare_parameter("transport_hint", "raw");
  std::string transport_hint{""};
  node->get_parameter("transport_hint", transport_hint);
  node->SetTransportHint(transport_hint);

  node->declare_parameter("output_name", "");
  std::string output_name;
  node->get_parameter("output_name", output_name);
  node->SetOutputName(output_name);

  node->declare_parameter("compress_type", "");
  std::string compress_type{""};
  node->get_parameter("compress_type", compress_type);
  node->SetCompressType(compress_type);

  node->declare_parameter("loop_time", 300);
  int loop_time{300};
  node->get_parameter("loop_time", loop_time);
  node->SetLoopTime(loop_time);

  std::cout << "transport_hint " << transport_hint << std::endl;
  std::cout << "compress_type " << compress_type << std::endl;

  node->Initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread([&]() {executor.spin();});

  rclcpp::WallRate loop_rate(10);

  while (rclcpp::ok() && !node->IsFinished()) {
    loop_rate.sleep();
  }

  node->Destroy();

  rclcpp::shutdown();

  return 0;
}
