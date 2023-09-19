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
find_package(rclcpp REQUIRED)

include_directories(include)

# add the publisher example
add_executable(publish_image
  src/publish_image.cpp
  src/DataCollector.cpp
  src/PublisherImageTransport.cpp
)
target_link_libraries(publish_image PRIVATE
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_imgcodecs
  rclcpp::rclcpp
)

# add the subscriber example
add_executable(my_subscriber
  src/DataCollector.cpp
  src/SubscriberImageTransport.cpp
  src/subscriber.cpp
)
target_link_libraries(my_subscriber
  cv_bridge::cv_bridge
  image_transport::image_transport
  opencv_highgui
  rclcpp::rclcpp
)

# Install executables
install(
  TARGETS
    publish_image
    my_subscriber
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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