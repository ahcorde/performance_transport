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

#ifndef PERFORMANCE_TRANSPORT__POINT_CLOUD_TRANSPORT__PUBLISHERPOINTCLOUDTRANSPORT_HPP_
#define PERFORMANCE_TRANSPORT__POINT_CLOUD_TRANSPORT__PUBLISHERPOINTCLOUDTRANSPORT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <point_cloud_transport/point_cloud_transport.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace performance_transport
{
class PublisherPointCloudTransport : public rclcpp::Node
{
public:
  PublisherPointCloudTransport(
    const rclcpp::NodeOptions & _options,
    const std::string & _filename);
  void Initialize();
  void PublishMessage();

private:
  std::shared_ptr<point_cloud_transport::PointCloudTransport> pc_;
  point_cloud_transport::Publisher pub_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string filename_;
};
}  // namespace performance_transport

#endif  // PERFORMANCE_TRANSPORT__POINT_CLOUD_TRANSPORT__PUBLISHERPOINTCLOUDTRANSPORT_HPP_
