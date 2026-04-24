cmake_minimum_required(VERSION 3.21)
project(CrapCode LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


include(FetchContent)
include(CTest)

enable_testing()

# Already included by the Inekf
# FetchContent_Declare(
# eigen
#  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
#  GIT_TAG 5.0.0  
#)

set(PAHO_WITH_MQTT_C ON CACHE BOOL "" FORCE)
set(PAHO_BUILD_DOCUMENTATION ON CACHE BOOL "" FORCE)


FetchContent_Declare(
  pahomqtt
  GIT_REPOSITORY https://github.com/eclipse-paho/paho.mqtt.cpp.git
  GIT_TAG v1.6.0 
)

FetchContent_Declare(
    simpleble
    GIT_REPOSITORY https://github.com/simpleble/simpleble.git
    GIT_TAG v0.12.1
    SOURCE_SUBDIR simpleble
)

FetchContent_Declare(
    inekf
    GIT_REPOSITORY https://bitbucket.org/frostlab/inekf.git
    GIT_TAG v0.1.0
)

if(BUILD_TESTING)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.16.0
)
endif()

FetchContent_MakeAvailable(pahomqtt simpleble inekf)

if(BUILD_TESTING)
FetchContent_MakeAvailable(googletest)
endif()


add_subdirectory(core)

find_package(Threads REQUIRED)

add_executable(app app/main.cpp)
target_include_directories(app PUBLIC core/include/
                               PUBLIC core/include/Messaging
                               PUBLIC core/include/Processing)
target_link_libraries(app PUBLIC blood_oxygen_processor 
                          PUBLIC accel_gravity_measure_model
                          PUBLIC bluetooth 
                          PUBLIC paho_mqtt_client
                          PUBLIC finger_abduction_processor 
                          PUBLIC force_processing
                          PUBLIC joint_rom_processor 
                          PUBLIC wrist_orientation_processor
                          PUBLIC com_worker
                          PUBLIC sensor_processing_lane_worker
                          PUBLIC mqtt_worker
                          PUBLIC simpleble::simpleble
                          PUBLIC InEKF::Core 
                          PUBLIC InEKF::SE2Models 
                          PUBLIC InEKF::Inertial
                          PUBLIC PahoMqttCpp::paho-mqttpp3
                          PUBLIC Threads::Threads)

add_executable(bluetooth_connection_test app/bluetooth_connection_test.cpp)
target_include_directories(bluetooth_connection_test
    PUBLIC core/include/
    PUBLIC core/include/Messaging
)
target_link_libraries(bluetooth_connection_test
    PUBLIC bluetooth
    PUBLIC simpleble::simpleble
    PUBLIC Threads::Threads
)

add_executable(mqtt_connection_test app/mqtt_connection_test.cpp)
target_include_directories(mqtt_connection_test
    PUBLIC core/include/
    PUBLIC core/include/Messaging
    PUBLIC core/include/Processing
)
target_link_libraries(mqtt_connection_test
    PUBLIC blood_oxygen_processor
    PUBLIC accel_gravity_measure_model
    PUBLIC bluetooth
    PUBLIC paho_mqtt_client
    PUBLIC finger_abduction_processor
    PUBLIC force_processing
    PUBLIC joint_rom_processor
    PUBLIC wrist_orientation_processor
    PUBLIC com_worker
    PUBLIC sensor_processing_lane_worker
    PUBLIC mqtt_worker
    PUBLIC simpleble::simpleble
    PUBLIC InEKF::Core
    PUBLIC InEKF::SE2Models
    PUBLIC InEKF::Inertial
    PUBLIC PahoMqttCpp::paho-mqttpp3
    PUBLIC Threads::Threads
)

if(BUILD_TESTING)
add_subdirectory(tests)
endif()
