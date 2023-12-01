set(SIZES 4096 2048 1024 512)
set(COMPRESS_VALUES 90 50 20)
set(COMPRESS_TYPE jpeg)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(SIZE IN LISTS SIZES)
        message("Created test: " test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_image_transport.py
            TARGET test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE}
            ARGS "size:=${SIZE}" "transport_type:=compressed" "compress_type:=${COMPRESS_TYPE}" "compress:=${COMPRESS_VALUE}" "output_name:=${SIZE}x${SIZE}"
            TIMEOUT 330
        )
    endforeach()
endforeach()

set(COMPRESS_VALUES 3 6 9)
set(COMPRESS_TYPE png)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(SIZE IN LISTS SIZES)
        message("Created test: " test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_image_transport.py
            TARGET test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE}
            ARGS "size:=${SIZE}" "transport_type:=compressed" "compress_type:=${COMPRESS_TYPE}" "compress:=${COMPRESS_VALUE}" "output_name:=${SIZE}x${SIZE}"
            TIMEOUT 330
        )
    endforeach()
endforeach()

set(COMPRESS_VALUES 3 6 9)
set(COMPRESS_TYPE zstd)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(SIZE IN LISTS SIZES)
        message("Created test: " test_image_transport_${SIZE}_${COMPRESS_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_image_transport.py
            TARGET test_image_transport_${SIZE}_${COMPRESS_TYPE}_${COMPRESS_VALUE}
            ARGS "size:=${SIZE}" "transport_type:=zstd" "compress_type:=${COMPRESS_TYPE}" "compress:=${COMPRESS_VALUE}" "output_name:=${SIZE}x${SIZE}"
            TIMEOUT 330
        )
    endforeach()
endforeach()

set(COMPRESS_VALUES 90 50 20)
set(COMPRESS_TYPE avif)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(SIZE IN LISTS SIZES)
        message("Created test: " test_image_transport_${SIZE}_${COMPRESS_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_image_transport.py
            TARGET test_image_transport_${SIZE}_${COMPRESS_TYPE}_${COMPRESS_VALUE}
            ARGS "size:=${SIZE}" "transport_type:=avif" "compress_type:=${COMPRESS_TYPE}" "compress:=${COMPRESS_VALUE}" "output_name:=${SIZE}x${SIZE}"
            TIMEOUT 330
        )
    endforeach()
endforeach()

foreach(SIZE IN LISTS SIZES)
    message("Created test: " test_image_transport_${SIZE}_raw)
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_${SIZE}_raw
        ARGS "size:=${SIZE}" "transport_type:=raw" "output_name:=${SIZE}x${SIZE}"
        TIMEOUT 330
    )
endforeach()

set(COMPRESS_VALUES 3 6 9)
set(PointCloudFiles "table_scene_lms400" "office_chair_scene_360" "office_chair_model")
set(TRANSPORT_TYPES draco zlib zstd)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(TRANSPORT_TYPE IN LISTS TRANSPORT_TYPES)
        foreach(PointCloudFile IN LISTS PointCloudFiles)
            message("Created test: " test_point_cloud_transport_${PointCloudFile}_${TRANSPORT_TYPE}_${COMPRESS_VALUE})
            add_launch_test(test_point_cloud.py
                    TARGET test_point_cloud_transport_${PointCloudFile}_${TRANSPORT_TYPE}_${COMPRESS_VALUE}
                    ARGS
                        "filename:=${PointCloudFile}.pcd"
                        "compress_type:=${TRANSPORT_TYPE}"
                        "transport_type:=${TRANSPORT_TYPE}"
                        "compress:=${COMPRESS_VALUE}"
                        "output_name:=${PointCloudFiles}"
                    TIMEOUT 330
            )
        endforeach()
    endforeach()
endforeach()

foreach(PointCloudFile IN LISTS PointCloudFiles)
    message("Created test: " test_point_cloud_transport_${PointCloudFile}_raw)
    add_launch_test(test_point_cloud.py
        TARGET test_point_cloud_transport_${PointCloudFile}_raw
        ARGS "filename:=${PointCloudFile}.pcd" "transport_type:=raw" "output_name:=${PointCloudFiles}"
        TIMEOUT 330
    )
endforeach()

# indoor
set(TRANSPORT_TYPES draco zlib zstd)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(TRANSPORT_TYPE IN LISTS TRANSPORT_TYPES)
        message("Created test: " test_point_cloud_transport_rosbag_indoor_${TRANSPORT_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_point_cloud.py
            TARGET test_point_cloud_transport_rosbag_indoor_${TRANSPORT_TYPE}_${COMPRESS_VALUE}
            ARGS
                "filename:=/home/ahcorde/TRI/kitti/files/r2b_lounge_mcap/"
                "rosbag_topic:=pandar_xt_32_0_lidar"
                "compress_type:=${TRANSPORT_TYPE}"
                "compress:=${COMPRESS_VALUE}"
                "transport_type:=${TRANSPORT_TYPE}"
                "output_name:=indoor"
            TIMEOUT 330
        )
    endforeach()
endforeach()

# outdoor
set(TRANSPORT_TYPES draco zlib zstd)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(TRANSPORT_TYPE IN LISTS TRANSPORT_TYPES)
        message("Created test: " test_point_cloud_transport_rosbag_outdoor_${TRANSPORT_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_point_cloud.py
            TARGET test_point_cloud_transport_rosbag_outdoor_${TRANSPORT_TYPE}_${COMPRESS_VALUE}
            ARGS
                "filename:=/home/ahcorde/TRI/kitti/ros2_output"
                "rosbag_topic:=/kitti/velo/pointcloud"
                "compress_type:=${TRANSPORT_TYPE}"
                "compress:=${COMPRESS_VALUE}"
                "transport_type:=${TRANSPORT_TYPE}"
                "output_name:=outdoor"
            TIMEOUT 330
        )
    endforeach()
endforeach()

# indoor
set(COMPRESS_VALUES 90 50 20)
set(COMPRESS_TYPE jpeg)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    message("Created test: " test_image_transport_rosbag_indoor_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_rosbag_indoor_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE}
        ARGS
        "filename:=/home/ahcorde/TRI/kitti/files/r2b_lounge_mcap/"
        "rosbag_topic:=d455_1_rgb_image"
        "output_name:=indoor"
        "transport_type:=compressed"
        "compress_type:=${COMPRESS_TYPE}"
        "compress:=${COMPRESS_VALUE}"
        TIMEOUT 330
    )
endforeach()

set(COMPRESS_VALUES 3 6 9)
set(COMPRESS_TYPE png)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    message("Created test: " test_image_transport_rosbag_indoor_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_rosbag_indoor_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE}
        ARGS
        "transport_type:=compressed"
        "compress_type:=${COMPRESS_TYPE}"
        "compress:=${COMPRESS_VALUE}"
        "filename:=/home/ahcorde/TRI/kitti/files/r2b_lounge_mcap/"
        "rosbag_topic:=d455_1_rgb_image"
        "output_name:=indoor"
        TIMEOUT 330
    )
endforeach()

set(COMPRESS_VALUES 3 6 9)
set(COMPRESS_TYPE zstd)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    message("Created test: " test_image_transport_rosbag_indoor_${COMPRESS_TYPE}_${COMPRESS_VALUE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_rosbag_indoor_${COMPRESS_TYPE}_${COMPRESS_VALUE}
        ARGS
        "filename:=/home/ahcorde/TRI/kitti/files/r2b_lounge_mcap/"
        "rosbag_topic:=d455_1_rgb_image"
        "output_name:=indoor"
        "transport_type:=zstd"
        "compress_type:=${COMPRESS_TYPE}"
        "compress:=${COMPRESS_VALUE}"
        TIMEOUT 330
    )
endforeach()

# outdoor
set(COMPRESS_VALUES 90 50 20)
set(COMPRESS_TYPE jpeg)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    message("Created test: " test_image_transport_rosbag_outdoor_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_rosbag_outdoor_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE}
        ARGS
        "filename:=/home/ahcorde/TRI/kitti/ros2_output"
        "output_name:=outdoor"
        "rosbag_topic:=/kitti/camera_color_left/image_raw"
        "transport_type:=compressed"
        "compress_type:=${COMPRESS_TYPE}"
        "compress:=${COMPRESS_VALUE}"
        TIMEOUT 330
    )
endforeach()

set(COMPRESS_VALUES 3 6 9)
set(COMPRESS_TYPE png)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    message("Created test: " test_image_transport_rosbag_outdoor_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_rosbag_outdoor_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE}
        ARGS
        "transport_type:=compressed"
        "compress_type:=${COMPRESS_TYPE}"
        "compress:=${COMPRESS_VALUE}"
        "filename:=/home/ahcorde/TRI/kitti/ros2_output"
        "rosbag_topic:=/kitti/camera_color_left/image_raw"
        "output_name:=outdoor"
        TIMEOUT 330
    )
endforeach()

set(COMPRESS_VALUES 3 6 9)
set(COMPRESS_TYPE zstd)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    message("Created test: " test_image_transport_rosbag_outdoor_${COMPRESS_TYPE}_${COMPRESS_VALUE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_rosbag_outdoor_${COMPRESS_TYPE}_${COMPRESS_VALUE}
        ARGS
        "filename:=/home/ahcorde/TRI/kitti/ros2_output"
        "output_name:=outdoor"
        "rosbag_topic:=/kitti/camera_color_left/image_raw"
        "transport_type:=zstd"
        "compress_type:=${COMPRESS_TYPE}"
        "compress:=${COMPRESS_VALUE}"
        TIMEOUT 330
    )
endforeach()

set(COMPRESS_VALUES 90 50 20)
set(COMPRESS_TYPE avif)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    message("Created test: " test_image_transport_rosbag_outdoor_${COMPRESS_TYPE}_${COMPRESS_VALUE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_rosbag_outdoor_${COMPRESS_TYPE}_${COMPRESS_VALUE}
        ARGS
        "filename:=/home/ahcorde/TRI/kitti/ros2_output"
        "output_name:=outdoor"
        "rosbag_topic:=/kitti/camera_color_left/image_raw"
        "transport_type:=${COMPRESS_TYPE}"
        "compress_type:=${COMPRESS_TYPE}"
        "compress:=${COMPRESS_VALUE}"
        TIMEOUT 330
    )
endforeach()
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    message("Created test: " test_image_transport_rosbag_indoor_${COMPRESS_TYPE}_${COMPRESS_VALUE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_rosbag_indoor_${COMPRESS_TYPE}_${COMPRESS_VALUE}
        ARGS
        "filename:=/home/ahcorde/TRI/kitti/files/r2b_lounge_mcap/"
        "output_name:=indoor"
        "rosbag_topic:=d455_1_rgb_image"
        "transport_type:=${COMPRESS_TYPE}"
        "compress_type:=${COMPRESS_TYPE}"
        "compress:=${COMPRESS_VALUE}"
        TIMEOUT 330
    )
endforeach()

message("Created test: " test_image_transport_indoor_raw)
add_launch_test(test_image_transport.py
    TARGET test_image_transport_indoor_raw
    ARGS
    "transport_type:=raw"
    "filename:=/home/ahcorde/TRI/kitti/files/r2b_lounge_mcap/"
    "rosbag_topic:=d455_1_rgb_image"
    "output_name:=indoor"
    TIMEOUT 330
)

message("Created test: " test_image_transport_outdoor_raw)
add_launch_test(test_image_transport.py
    TARGET test_image_transport_outdoor_raw
    ARGS
    "transport_type:=raw"
    "filename:=/home/ahcorde/TRI/kitti/ros2_output"
    "rosbag_topic:=/kitti/camera_color_left/image_raw"
    "output_name:=outdoor"
    TIMEOUT 330
)

message("Created test: " test_image_transport_outdoor_svtav1)
add_launch_test(test_image_transport.py
    TARGET test_image_transport_outdoor_svtav1
    ARGS
    "filename:=/home/ahcorde/TRI/kitti/files/r2b_lounge_mcap/"
    "output_name:=indoor"
    "rosbag_topic:=d455_1_rgb_image"
    "transport_type:=svtav1"
    "compress_type:=svtav1"
    "compress:=10"
    TIMEOUT 330
)
