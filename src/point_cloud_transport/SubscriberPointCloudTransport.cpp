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

#include "performance_transport/point_cloud_transport/SubscriberPointCloudTransport.hpp"

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;
namespace performance_transport
{
SubscriberPointCloudTransport::SubscriberPointCloudTransport(
  const rclcpp::NodeOptions & _options,
  const std::string & _transport_hint)
: Node("subscriber_point_cloud_transport", _options), transport_hint_(_transport_hint)
{
}

SubscriberPointCloudTransport::~SubscriberPointCloudTransport()
{
  this->dataCollector_->Close();
}

inline double timeToSec(const builtin_interfaces::msg::Time & time_msg)
{
  auto ns = std::chrono::duration<double, std::nano>(time_msg.nanosec);
  auto s = std::chrono::duration<double>(time_msg.sec);
  return (s + std::chrono::duration_cast<std::chrono::duration<double>>(ns)).count();
}

void SubscriberPointCloudTransport::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
}

void SubscriberPointCloudTransport::Initialize()
{
  this->pc = std::make_shared<point_cloud_transport::PointCloudTransport>(this->shared_from_this());
  auto transport_hint = std::make_shared<point_cloud_transport::TransportHints>(
    this->transport_hint_);
  this->sub = this->pc->subscribe(
    "pct/point_cloud", 100,
    std::bind(&SubscriberPointCloudTransport::pointCloudCallback, this, _1),
    {}, transport_hint.get());
}
}  // namespace performance_transport
