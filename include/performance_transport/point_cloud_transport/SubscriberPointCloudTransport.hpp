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

#ifndef PERFORMANCE_TRANSPORT__POINT_CLOUD_TRANSPORT__SUBSCRIBERPOINTCLOUDTRANSPORT_HPP_
#define PERFORMANCE_TRANSPORT__POINT_CLOUD_TRANSPORT__SUBSCRIBERPOINTCLOUDTRANSPORT_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/subscriber.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "performance_transport/utils/DataCollector.hpp"
#include "performance_transport/utils/SystemDataCollector.hpp"

namespace performance_transport
{
class SubscriberPointCloudTransport : public rclcpp::Node
{
public:
  SubscriberPointCloudTransport(
    const rclcpp::NodeOptions & _options);
  ~SubscriberPointCloudTransport();
  void SetTransportHint(const std::string & _transport_hint);
  void Initialize();
  void checkSubscribers();
  void Destroy();
  bool IsFinished();
  void SetLoopTime(int _loop_time);
  void SetOutputName(const std::string & _output_name);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  std::shared_ptr<point_cloud_transport::PointCloudTransport> pc;
  point_cloud_transport::Subscriber sub;
  std::string transport_hint_;
  std::shared_ptr<DataCollector> dataCollector_;
  std::chrono::time_point<std::chrono::system_clock> start;
  std::shared_ptr<SystemDataCollector> systemDataCollector;
  rclcpp::TimerBase::SharedPtr timer_;

  double diff_time_sim_{0};
  int count_{0};

  std::atomic<bool> stop_{false};
  std::string output_name_;

  int loop_time_{300};

  double initial_time{0};
  int compress_{0};
};
}  // namespace performance_transport

#endif  // PERFORMANCE_TRANSPORT__POINT_CLOUD_TRANSPORT__SUBSCRIBERPOINTCLOUDTRANSPORT_HPP_
