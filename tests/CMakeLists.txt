include(GoogleTest)

function(add_worker_test_target tgt_name)
    add_executable(${tgt_name} ${ARGN})
    target_link_libraries(${tgt_name}
        PUBLIC GTest::gtest_main
        PUBLIC GTest::gmock
        PUBLIC com_worker
        PUBLIC sensor_processing_lane_worker
        PUBLIC mqtt_worker
        PUBLIC blood_oxygen_processor
        PUBLIC accel_gravity_measure_model
        PUBLIC finger_abduction_processor
        PUBLIC force_processing
        PUBLIC joint_rom_processor
        PUBLIC wrist_orientation_processor
        PUBLIC paho_mqtt_client
        PUBLIC Threads::Threads
    )
    gtest_discover_tests(${tgt_name})
endfunction()

add_worker_test_target(workers_unit_tests workers_unit_tests.cpp)
add_worker_test_target(workers_threaded_tests workers_threaded_tests.cpp)
