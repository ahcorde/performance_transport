# Performance test

The goal of this repository is to measure the performance of the `image_transport` and `point_cloud_transport`

# Compile the repository

```bash
mkdir test_transport_ws/src -p
cd test_transport_ws/src
git clone https://github.com/ahcorde/performance_transport
cd ..
source /opt/ros/rolling/setup.bash
colcon build --merge-install --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=1
```

# image_transport

If you run the test you will see the results for: 4096x4096, 2048x2048, 1024x1024, 512x512

```bash
colcon test --merge-install --event-handlers console_direct+ --ctest-args -R test_raw
```

## Run then manually

Run in one terminal the subscriber:

```bash
ros2 run performance_transport my_subscriber compressed --ros-args -p qos_overrides./parameter_events.publisher.reliability:=best_effort
```

In another one the publisher:

```bash
ros2 run performance_transport publish_image <path_to_image> 4096 --ros-args -p camera.image.enable_pub_plugins:=['image_transport/compressed'] -p qos_overrides./parameter_events.publisher.reliability:=best_effort
```

# Plot the results
```bash
gnuplot
plot './build/performance_transport/test/publisher_data_4096_4096.csv' with linespoints, './build/performance_transport/test/publisher_data_2048_2048.csv' with linespoints, './build/performance_transport/test/publisher_data_1024_1024.csv' with linespoints, './build/performance_transport/test/publisher_data_512_512.csv' with linespoints
plot './build/performance_transport/test/subscriber_data_4096_4096.csv' with linespoints, './build/performance_transport/test/publisher_data_2048_2048.csv' with linespoints, './build/performance_transport/test/publisher_data_1024_1024.csv' with linespoints, './build/performance_transport/test/publisher_data_512_512.csv' with linespoints

# Or all together

plot './build/performance_transport/test/publisher_data_4096_4096.csv' with linespoints, './build/performance_transport/test/publisher_data_2048_2048.csv' with linespoints, './build/performance_transport/test/publisher_data_1024_1024.csv' with linespoints, './build/performance_transport/test/publisher_data_512_512.csv' with linespoints,'./build/performance_transport/test/subscriber_data_4096_4096.csv' with linespoints, './build/performance_transport/test/publisher_data_2048_2048.csv' with linespoints, './build/performance_transport/test/publisher_data_1024_1024.csv' with linespoints, './build/performance_transport/test/publisher_data_512_512.csv' with linespoints
```