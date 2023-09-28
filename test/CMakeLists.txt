SET(SIZES 4096 2048 1024 512)
SET(COMPRESS_VALUES 90 70 40 30 10)
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

SET(COMPRESS_VALUES 1 3 5 7 9)
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

SET(COMPRESS_VALUES 1 3 5 7 9)
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

# SET(PointCloudFiles "table_scene_lms400.pcd" "office_chair_scene_360.pcd" "office_chair_model.pcd")
# foreach(PointCloudFile IN LISTS PointCloudFiles)
#     add_launch_test(test_point_cloud.py
#         TARGET test_point_cloud_transport_${PointCloudFile}_draco
#         ARGS "pcd_name:=${PointCloudFile}" "transport_type:=draco"
#         TIMEOUT 330
#     )
# endforeach()

