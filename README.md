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
performance_transport/subscribe_image --ros-args -p transport_hint:=compressed -p compress_type:=jpeg
```

In another one the publisher:

performance_transport/publish_image --ros-args -p filename:=<path_to_image> -p transport_hint:=compressed -p compress:=50 -p compress_type:=jpeg -p size:=512 -p camera.image.enable_pub_plugins:=['image_transport/compressed']
```

# Plot the results

Under the folder `scripts` there are some scripts to visualize the data:

```bash

```