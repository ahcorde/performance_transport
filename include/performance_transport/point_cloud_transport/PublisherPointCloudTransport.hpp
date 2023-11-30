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

#include <rosbag2_cpp/reader.hpp>

namespace performance_transport
{
class PublisherPointCloudTransport : public rclcpp::Node
{
public:
  PublisherPointCloudTransport(
    const rclcpp::NodeOptions & _options);
  ~PublisherPointCloudTransport();
  void Initialize();
  void SetFilename(const std::string & _filename);
  void SetCompressType(const std::string & _compress_type);
  void SetCompress(int _value);
  void SetCompressParameter();
  void SetTransportHint(const std::string & _transport_hint);
  void SetRosBag(const std::string & _rosbag_topic);
  void PublishMessage();
  int GetSize();
  int GetNumberOfImagesPublished();
  void Destroy();

private:
  std::shared_ptr<point_cloud_transport::PointCloudTransport> pc_;
  point_cloud_transport::Publisher pub_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string filename_;
  std::string rosbag_topic_;
  std::mutex mutex_;
  int count_{0};
  int compress_{0};
  std::string compress_type_;
  std::string transport_hint_;

  rosbag2_cpp::readers::SequentialReader reader;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> image_serialization;
};
}  // namespace performance_transport

#endif  // PERFORMANCE_TRANSPORT__POINT_CLOUD_TRANSPORT__PUBLISHERPOINTCLOUDTRANSPORT_HPP_
