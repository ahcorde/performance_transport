cmake_minimum_required(VERSION 3.8)
project(performance_transport)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS io)
find_package(pcl_conversions REQUIRED)

include_directories(include)

add_library(performance_transport_utils SHARED
  src/utils/DataCollector.cpp
  src/utils/ProcessInfo.cpp
)

# add the publisher example
add_executable(publish_image
  src/publish_image.cpp
  src/image_transport/PublisherImageTransport.cpp
)
ament_target_dependencies(publish_image
  cv_bridge
  image_transport
  rclcpp
)
target_link_libraries(publish_image
  image_transport::image_transport
  opencv_imgcodecs
  rclcpp::rclcpp
  performance_transport_utils
)

# add the subscriber example
add_executable(subscribe_image
  src/image_transport/SubscriberImageTransport.cpp
  src/subscribe_image.cpp
)
ament_target_dependencies(subscribe_image
  cv_bridge
  image_transport
  rclcpp
)
target_link_libraries(subscribe_image
  performance_transport_utils
)

## Point cloud
add_executable(publish_point_cloud
  src/publish_point_cloud.cpp
  src/point_cloud_transport/PublisherPointCloudTransport.cpp
)
ament_target_dependencies(publish_point_cloud
  pcl_conversions
  point_cloud_transport
  rclcpp
)
target_link_libraries(publish_point_cloud
  ${PCL_LIBRARIES}
  performance_transport_utils
)

add_executable(subcribe_point_cloud
  src/subscribe_point_cloud.cpp
  src/point_cloud_transport/SubscriberPointCloudTransport.cpp
)
ament_target_dependencies(subcribe_point_cloud
  point_cloud_transport
  rclcpp
)
target_link_libraries(subcribe_point_cloud
  performance_transport_utils
)

# Install executables
install(
  TARGETS
    publish_image
    subscribe_image
    publish_point_cloud
    subcribe_point_cloud
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS performance_transport_utils EXPORT performance_transport_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  add_subdirectory(test)

  install(DIRECTORY
    resources
    DESTINATION
      share/${PROJECT_NAME}
  )
endif()


ament_package()