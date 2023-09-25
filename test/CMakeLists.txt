SET(SIZES 4096 2048 1024 512)
foreach(SIZE IN LISTS SIZES)
#    message(${SIZE})
    add_launch_test(test_image_transport.py
        TARGET test_image_transport_${SIZE}_compressed
        ARGS "size:=${SIZE}" "transport_type:=compressed"
        TIMEOUT 330
    )
endforeach()

SET(PointCloudFiles "table_scene_lms400.pcd" "office_chair_scene_360.pcd" "office_chair_model.pcd")
foreach(PointCloudFile IN LISTS PointCloudFiles)
    add_launch_test(test_point_cloud.py
        TARGET test_point_cloud_transport_${PointCloudFile}_draco
        ARGS "pcd_name:=${PointCloudFile}" "transport_type:=draco"
        TIMEOUT 330
    )
endforeach()

