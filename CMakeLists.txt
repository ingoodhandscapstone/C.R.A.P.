cmake_minimum_required(VERSION 3.21)
project(CrapCode LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


include(FetchContent)

# Already included by the Inekf
# FetchContent_Declare(
# eigen
#  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
#  GIT_TAG 5.0.0  
#)

set(PAHO_WITH_MQTT_C ON CACHE BOOL "" FORCE)


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


FetchContent_MakeAvailable(pahomqtt simpleble inekf)


add_subdirectory(core)


add_executable(app app/main.cpp)
target_include_directories(app PUBLIC core/include/)
target_link_libraries(app PUBLIC blood_oxygen_processor 
                          PUBLIC bluetooth 
                          PUBLIC finger_abduction_processor 
                          PUBLIC joint_rom_processor 
                          PUBLIC wrist_orientation_processor
                          PUBLIC simpleble::simpleble
                          PUBLIC InEKF::Core 
                          PUBLIC InEKF::SE2Models 
                          PUBLIC InEKF::Inertial
                          PUBLIC PahoMqttCpp::paho-mqttpp3)
