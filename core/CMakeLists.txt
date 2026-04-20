add_library(blood_oxygen_processor OBJECT src/Processing/BloodOxygenProcessor.cpp)
add_library(bluetooth OBJECT src/Messaging/Bluetooth.cpp)
add_library(finger_abduction_processor OBJECT src/Processing/FingerAbductionProcessor.cpp)
add_library(force_processing OBJECT src/Processing/ForceProcessing.cpp)
add_library(joint_rom_processor OBJECT src/Processing/JointRomProcessor.cpp)
add_library(wrist_orientation_processor OBJECT src/Processing/WristOrientationProcessor.cpp)
add_library(com_worker OBJECT src/Workers/ComWorker.cpp)
add_library(sensor_processing_worker OBJECT src/Workers/SensorProcessingWorker.cpp)
add_library(mqtt_worker OBJECT src/Workers/MQTTWorker.cpp)



target_include_directories(blood_oxygen_processor PUBLIC include/Processing
                                                  PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/inekf-src/include)
target_include_directories(bluetooth PUBLIC include/Messaging
                                     PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/simpleble-src/simpleble/include
                                     PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/simpleble-src/dependencies/external
                                     PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/simpleble-build/export)
target_include_directories(finger_abduction_processor PUBLIC include/Processing
                                                      PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/inekf-src/include)
target_include_directories(force_processing PUBLIC include/Processing
                                              PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/inekf-src/include)
target_include_directories(joint_rom_processor PUBLIC include/Processing
                                               PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/inekf-src/include)
target_include_directories(wrist_orientation_processor PUBLIC include/Processing
                                                       PUBLIC include/Workers
                                                       PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/inekf-src/include)
target_include_directories(com_worker PUBLIC include/Messaging)
target_include_directories(sensor_processing_worker PUBLIC include/Processing
                                                    PUBLIC include/Workers)
target_include_directories(mqtt_worker PUBLIC include/Processing
                                      PUBLIC include/Workers
                                      PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/pahomqtt-src/include
                                      PUBLIC /home/jason-gomez/InGoodHandsCapstoneCode/CrapCode/build/_deps/pahomqtt-src/externals/paho.mqtt.c/src)


target_link_libraries(bluetooth PUBLIC simpleble::simpleble)
target_link_libraries(finger_abduction_processor PUBLIC InEKF::Core PUBLIC InEKF::SE2Models PUBLIC InEKF::Inertial)
target_link_libraries(wrist_orientation_processor PUBLIC InEKF::Core PUBLIC InEKF::SE2Models PUBLIC InEKF::Inertial)
target_link_libraries(mqtt_worker PUBLIC PahoMqttCpp::paho-mqttpp3)
# May have MQTT related class later


# External targets used here:
# - simpleble::simpleble
# - InEKF::Core, InEKF::SE2Models, InEKF::Inertial
# - PahoMqttCpp::paho-mqttpp3
