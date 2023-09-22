SET(SIZES 4096 2048 1024 512)
foreach(SIZE IN LISTS SIZES)
    message(${SIZE})
    add_launch_test(test_raw.py
        TARGET test_raw_${SIZE}_compressed
        ARGS "size:=${SIZE}" "transport_type:=compressed"
        TIMEOUT 330
    )

endforeach()
