add_library(blood_oxygen_processor OBJECT src/Processing/BloodOxygenProcessor.cpp)
add_library(accel_gravity_measure_model OBJECT src/Processing/AccelGravityMeasureModel.cpp)
add_library(bluetooth OBJECT src/Messaging/Bluetooth.cpp)
add_library(paho_mqtt_client OBJECT src/Messaging/PahoMQTTClient.cpp)
add_library(finger_abduction_processor OBJECT src/Processing/FingerAbductionProcessor.cpp)
add_library(force_processing OBJECT src/Processing/ForceProcessing.cpp)
add_library(joint_rom_processor OBJECT src/Processing/JointRomProcessor.cpp)
add_library(wrist_orientation_processor OBJECT src/Processing/WristOrientationProcessor.cpp)
add_library(com_worker OBJECT src/Workers/ComWorker.cpp)
add_library(sensor_processing_lane_worker OBJECT src/Workers/SensorProcessingLaneWorker.cpp)
add_library(mqtt_worker OBJECT src/Workers/MQTTWorker.cpp)



target_include_directories(blood_oxygen_processor PUBLIC include/Processing)
target_include_directories(accel_gravity_measure_model PUBLIC include/Processing)
target_include_directories(bluetooth PUBLIC include/Messaging)
target_include_directories(paho_mqtt_client PUBLIC include/Messaging)
target_include_directories(finger_abduction_processor PUBLIC include/Processing)
target_include_directories(force_processing PUBLIC include/Processing)
target_include_directories(joint_rom_processor PUBLIC include/Processing)
target_include_directories(wrist_orientation_processor PUBLIC include/Processing
                                                       PUBLIC include/Workers)
target_include_directories(com_worker PUBLIC include/Messaging PUBLIC include/Workers PUBLIC include/Processing)

target_include_directories(sensor_processing_lane_worker PUBLIC include/Processing
                                                        PUBLIC include/Workers
                                                        PUBLIC include/Messaging)
target_include_directories(mqtt_worker PUBLIC include/Processing
                                      PUBLIC include/Workers
                                      PUBLIC include/Messaging)


target_link_libraries(bluetooth PUBLIC simpleble::simpleble)
target_link_libraries(paho_mqtt_client PUBLIC PahoMqttCpp::paho-mqttpp3)
target_link_libraries(accel_gravity_measure_model PUBLIC InEKF::Core)
target_link_libraries(finger_abduction_processor PUBLIC InEKF::Core PUBLIC InEKF::SE2Models PUBLIC InEKF::Inertial)
target_link_libraries(wrist_orientation_processor PUBLIC InEKF::Core PUBLIC InEKF::SE2Models PUBLIC InEKF::Inertial)
target_link_libraries(mqtt_worker PUBLIC PahoMqttCpp::paho-mqttpp3)
target_link_libraries(force_processing PUBLIC PUBLIC InEKF::Core)
target_link_libraries(joint_rom_processor PUBLIC PUBLIC InEKF::Core)
target_link_libraries(sensor_processing_lane_worker PUBLIC PUBLIC InEKF::Core )

# May have MQTT related class later


# External targets used here:
# - simpleble::simpleble
# - InEKF::Core, InEKF::SE2Models, InEKF::Inertial
# - PahoMqttCpp::paho-mqttpp3
