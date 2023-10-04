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

#include "rclcpp/rclcpp.hpp"
#include "performance_transport/point_cloud_transport/SubscriberPointCloudTransport.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<performance_transport::SubscriberPointCloudTransport> node =
    std::make_shared<performance_transport::SubscriberPointCloudTransport>(
    options);

  node->declare_parameter("transport_hint", "raw");
  std::string transport_hint{""};
  node->get_parameter("transport_hint", transport_hint);
  node->SetTransportHint(transport_hint);

  node->declare_parameter("loop_time", 300);
  int loop_time{300};
  node->get_parameter("loop_time", loop_time);
  node->SetLoopTime(loop_time);

  node->Initialize();

  rclcpp::WallRate loop_rate(30);

  while (rclcpp::ok() && !node->IsFinished()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  node->Destroy();

  rclcpp::shutdown();

  return 0;
}
