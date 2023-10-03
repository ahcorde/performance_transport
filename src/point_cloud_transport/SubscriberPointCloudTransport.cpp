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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "performance_transport/utils/utils.hpp"

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace performance_transport
{
SubscriberPointCloudTransport::SubscriberPointCloudTransport(
  const rclcpp::NodeOptions & _options)
: Node("subscriber_point_cloud_transport", _options)
{
}

SubscriberPointCloudTransport::~SubscriberPointCloudTransport()
{
  this->Destroy();
}

void SubscriberPointCloudTransport::Destroy()
{
  if (this->dataCollector_ != nullptr) {
    this->dataCollector_->Close();
  }

  if (systemDataCollector != nullptr) {
    this->systemDataCollector->Close();
  }
  this->timer_->cancel();

  this->sub.shutdown();
  this->pc.reset();
  this->timer_->cancel();
}

bool SubscriberPointCloudTransport::IsFinished()
{
  return this->stop_;
}

void SubscriberPointCloudTransport::checkSubscribers()
{
  auto current_time_stamp_seconds = timeToSec(this->now());
  auto diff = current_time_stamp_seconds - this->last_update;

  if (diff > 5) {
    this->stop_ = true;
  }
}

void SubscriberPointCloudTransport::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  this->count_++;
  if (systemDataCollector == nullptr) {
    int size = msg->width * msg->height * msg->fields.size();

    std::string filename = "subscriber_point_cloud_data" + std::string("_") +
      std::to_string(size) + std::string("_") +
      this->transport_hint_;

    std::string filenameSystemData = "subscriber_point_cloud_data_cpu_mem" + std::string("_") +
      std::to_string(size) + std::string("_") +
      this->transport_hint_;

    if (this->transport_hint_ != "raw") {
      filename += std::string("_") + std::to_string(this->compress_);

      filenameSystemData += std::string("_") + std::to_string(this->compress_);
    }
    filename += ".csv";
    filenameSystemData += ".csv";

    this->systemDataCollector = std::make_shared<SystemDataCollector>(
      filenameSystemData,
      this->get_clock());
    this->dataCollector_ = std::make_shared<DataCollector>(filename);
    this->timer_ = this->create_wall_timer(
      1s,
      std::bind(&SubscriberPointCloudTransport::checkSubscribers, this));
    this->start = std::chrono::high_resolution_clock::now();
  }

  auto msg_time_stamp_seconds = timeToSec(msg->header.stamp);
  auto current_time_stamp_seconds = timeToSec(this->now());

  this->diff_time_sim_ += current_time_stamp_seconds - msg_time_stamp_seconds;

  this->last_update = current_time_stamp_seconds;

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
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    this, "publisher_point_cloud_transport");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  std::stringstream ss;

  std::string param_name;
  if (this->transport_hint_ == "draco") {
    param_name = "encode_speed";
  } else if (this->transport_hint_ == "zlib") {
    param_name = "encode_level";
  } else if (this->transport_hint_ == "zstd") {
    param_name = "zstd_encode_level";
  }

  for (
    auto & parameter : parameters_client->get_parameters({param_name}))
  {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string();
    this->compress_ = std::atoi(parameter.value_to_string().c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Parameter: %s", ss.str().c_str());

  this->pc = std::make_shared<point_cloud_transport::PointCloudTransport>(this->shared_from_this());
  auto transport_hint = std::make_shared<point_cloud_transport::TransportHints>(
    this->transport_hint_);
  this->sub = this->pc->subscribe(
    "pct/point_cloud", rmw_qos_profile_sensor_data,
    std::bind(&SubscriberPointCloudTransport::pointCloudCallback, this, _1),
    {}, transport_hint.get());
}

void SubscriberPointCloudTransport::SetTransportHint(
  const std::string & _transport_hint)
{
  this->transport_hint_ = _transport_hint;
}
}  // namespace performance_transport
