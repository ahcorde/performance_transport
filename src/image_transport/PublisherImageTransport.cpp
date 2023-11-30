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

#include "performance_transport/image_transport/PublisherImageTransport.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/filesystem_helper.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>

using namespace std::chrono_literals;

namespace performance_transport
{
PublisherImageTransport::PublisherImageTransport(
  const rclcpp::NodeOptions & _options)
: Node("publisher_image_transport", _options)
{
  timer_ = this->create_wall_timer(
    33ms,
    std::bind(&PublisherImageTransport::PublishMessage, this));
}

void PublisherImageTransport::SetFilename(
  const std::string & _filename)
{
  this->filename_ = _filename;
}

void PublisherImageTransport::SetCompressType(
  const std::string & _compress_type)
{
  this->compress_type_ = _compress_type;
}

void PublisherImageTransport::SetRosBag(const std::string & _rosbag_topic)
{
  this->rosbag_topic_ = _rosbag_topic;
}

PublisherImageTransport::~PublisherImageTransport()
{
  this->Destroy();
}

void PublisherImageTransport::Destroy()
{
  this->pub.shutdown();
  this->it.reset();
  this->timer_->cancel();
}

void PublisherImageTransport::SetImageSize(int _width, int _height)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (cap.isOpened()) {
    return;
  }
  try {
    if (reader.has_next()) {
      return;
    }
  } catch (std::runtime_error &) {
  }

  std_msgs::msg::Header hdr;
  cv::Rect rect = cv::Rect(0, 0, _width, _height);
  cv::Mat image = cv::Mat(this->image_, rect);
  this->msg_ = cv_bridge::CvImage(hdr, "bgr8", image).toImageMsg();
}

int PublisherImageTransport::Height()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return this->image_.rows;
}

int PublisherImageTransport::Width()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return this->image_.cols;
}

void PublisherImageTransport::PublishMessage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (cap.isOpened()) {
    cap >> this->image_;
    if (this->image_.empty()) {
      exit(0);
    }
    std_msgs::msg::Header hdr;
    this->msg_ = cv_bridge::CvImage(hdr, "bgr8", this->image_).toImageMsg();
  }

  try {
    if (reader.has_next()) {
      auto serialized_message = reader.read_next();

      while (serialized_message->topic_name != this->rosbag_topic_) {
        // deserialize and convert to ros2 message
        serialized_message = reader.read_next();
        if (!reader.has_next()) {
          reader.seek(0);
        }
      }

      rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
      if (serialized_message->topic_name == this->rosbag_topic_) {
        image_serialization.deserialize_message(&extracted_serialized_msg, &(*this->msg_));
      }
    }
  } catch (std::runtime_error &) {

  }

  this->msg_->header.stamp = this->now();
  this->pub.publish(this->msg_);

  if (this->count == 0) {
    SetCompressJpegParameter();
    SetCompressType();
  }

  count++;
}

int PublisherImageTransport::GetNumberOfImagesPublished()
{
  std::lock_guard<std::mutex> lock(mutex_);
  int result = count;
  count = 0;
  return result;
}

void PublisherImageTransport::Initialize()
{
  this->it = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());
  this->pub = this->it->advertise(
    "camera/image", rclcpp::SensorDataQoS().keep_last(
      100).reliable().get_rmw_qos_profile());
  rcpputils::fs::path bag_file(this->filename_);
  if (this->filename_.find("mp4") != std::string::npos) {
    cap = cv::VideoCapture(this->filename_);
    if (!cap.isOpened()) {
      std::cout << "Error opening video stream or file" << std::endl;
      exit(-1);
    }
    std::cout << "Reading file " << this->filename_ << std::endl;

    cap >> this->image_;
  } else if (rcpputils::fs::exists(bag_file) && bag_file.is_directory()) {
    std::cout << "Openning bagfile " << bag_file << std::endl;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = this->filename_;
    storage_options.storage_id = "mcap";
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    // open the rosbag
    reader.open(storage_options, converter_options);

    auto serialized_message = reader.read_next();

    while (serialized_message->topic_name != this->rosbag_topic_) {
      // deserialize and convert to ros2 message
      serialized_message = reader.read_next();
    }

    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    if (serialized_message->topic_name == this->rosbag_topic_) {
      this->msg_ = std::make_shared<sensor_msgs::msg::Image>();
      image_serialization.deserialize_message(&extracted_serialized_msg, &(*this->msg_));
    }
  } else {
    this->image_ = cv::imread(this->filename_, cv::IMREAD_COLOR);
  }
}

void PublisherImageTransport::SetCompressJpeg(int _value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  this->compress_ = _value;
}

void PublisherImageTransport::SetCompressJpegParameter()
{
  if (this->compress_type_ == "jpeg") {
    this->set_parameter(rclcpp::Parameter("camera.image.compressed.jpeg_quality", this->compress_));
  } else if (this->compress_type_ == "png") {
    this->set_parameter(rclcpp::Parameter("camera.image.compressed.png_level", this->compress_));
  } else if (this->compress_type_ == "zstd") {
    this->set_parameter(rclcpp::Parameter("camera.image.zstd.zstd_level", this->compress_));
  } else if (this->compress_type_ == "avif") {
    this->set_parameter(rclcpp::Parameter("camera.image.avif.quality", this->compress_));
  }
}

void PublisherImageTransport::SetCompressType()
{
  if (this->compress_type_ == "jpeg" || this->compress_type_ == "png") {
    this->set_parameter(rclcpp::Parameter("camera.image.compressed.format", this->compress_type_));
  }
}

}  // namespace performance_transport
