SET(SIZES 4096 2048 1024 512)
SET(COMPRESS_VALUES 90 50 20)
SET(COMPRESS_TYPE jpeg)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(SIZE IN LISTS SIZES)
    message("Created test: " test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_image_transport.py
            TARGET test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE}
            ARGS "size:=${SIZE}" "transport_type:=compressed" "compress_type:=${COMPRESS_TYPE}" "compress:=${COMPRESS_VALUE}"
            TIMEOUT 330
        )
    endforeach()
endforeach()

SET(COMPRESS_VALUES 3 6 9)
SET(COMPRESS_TYPE png)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(SIZE IN LISTS SIZES)
    message("Created test: " test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_image_transport.py
            TARGET test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE}
            ARGS "size:=${SIZE}" "transport_type:=compressed" "compress_type:=${COMPRESS_TYPE}" "compress:=${COMPRESS_VALUE}"
            TIMEOUT 330
        )
    endforeach()
endforeach()

SET(COMPRESS_VALUES 3 6 9)
SET(COMPRESS_TYPE zstd)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(SIZE IN LISTS SIZES)
    message("Created test: " test_image_transport_${SIZE}_compressed_${COMPRESS_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_image_transport.py
            TARGET test_image_transport_${SIZE}_${COMPRESS_TYPE}_${COMPRESS_VALUE}
            ARGS "size:=${SIZE}" "transport_type:=zstd" "compress_type:=${COMPRESS_TYPE}" "compress:=${COMPRESS_VALUE}"
            TIMEOUT 330
        )
    endforeach()
endforeach()

foreach(SIZE IN LISTS SIZES)
    message("Created test: " test_image_transport_${SIZE}_raw)
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_${SIZE}_raw
        ARGS "size:=${SIZE}" "transport_type:=raw"
        TIMEOUT 330
    )
endforeach()

SET(COMPRESS_VALUES 3 6 9)
SET(PointCloudFiles "table_scene_lms400" "office_chair_scene_360" "office_chair_model")
SET(TRANSPORT_TYPES draco zlib zstd)
foreach(COMPRESS_VALUE IN LISTS COMPRESS_VALUES)
    foreach(TRANSPORT_TYPE IN LISTS TRANSPORT_TYPES)
        foreach(PointCloudFile IN LISTS PointCloudFiles)
        message("Created test: " test_point_cloud_transport_${PointCloudFile}_${TRANSPORT_TYPE}_${COMPRESS_VALUE})
        add_launch_test(test_point_cloud.py
                TARGET test_point_cloud_transport_${PointCloudFile}_${TRANSPORT_TYPE}_${COMPRESS_VALUE}
                ARGS "filename:=${PointCloudFile}.pcd" "compress_type:=${TRANSPORT_TYPE}" "transport_type:=${TRANSPORT_TYPE}" "compress:=${COMPRESS_VALUE}"
                TIMEOUT 330
            )
        endforeach()
    endforeach()
endforeach()

foreach(PointCloudFile IN LISTS PointCloudFiles)
    message("Created test: " test_point_cloud_transport_${PointCloudFile}_raw)
    add_launch_test(test_point_cloud.py
        TARGET test_point_cloud_transport_${PointCloudFile}_raw
        ARGS "filename:=${PointCloudFile}.pcd" "transport_type:=raw"
        TIMEOUT 330
    )
endforeach()
