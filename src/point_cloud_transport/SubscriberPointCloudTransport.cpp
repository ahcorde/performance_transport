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
#include "performance_transport/utils/utils.hpp"

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
  if (this->dataCollector_ != nullptr) {
    this->dataCollector_->Close();
  }

  if (systemDataCollector != nullptr) {
    this->systemDataCollector->Close();
  }
}

void SubscriberPointCloudTransport::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  this->count_++;
  if (systemDataCollector == nullptr) {
    int size = msg->width * msg->height * msg->fields.size();
    this->systemDataCollector = std::make_shared<SystemDataCollector>(
      "subscriber_point_cloud_data_cpu_mem" + std::string("_") +
      std::to_string(size) + ".csv",
      this->get_clock());
    this->dataCollector_ = std::make_shared<DataCollector>(
      "subscriber_point_cloud_data" + std::string("_") +
      std::to_string(size) + ".csv");
    this->start = std::chrono::high_resolution_clock::now();
  }

  auto msg_time_stamp_seconds = timeToSec(msg->header.stamp);
  auto current_time_stamp_seconds = timeToSec(this->now());

  this->diff_time_sim_ += current_time_stamp_seconds - msg_time_stamp_seconds;

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> elapsed = finish - this->start;
  if (elapsed.count() > 1) {
    double fps = static_cast<double>(this->count_) / static_cast<double>(elapsed.count());
    double response_time = static_cast<double>(this->diff_time_sim_) /
      static_cast<double>(this->count_);
    this->count_ = 0;
    this->diff_time_sim_ = 0;
    this->start = std::chrono::high_resolution_clock::now();
    if (dataCollector_ != nullptr) {
      dataCollector_->WriteLine(
        std::to_string(fps) + "," +
        std::to_string(response_time));
    }
  }
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
