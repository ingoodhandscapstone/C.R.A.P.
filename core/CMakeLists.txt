add_library(blood_oxygen_processor OBJECT src/BloodOxygenProcessor.cpp)
add_library(bluetooth OBJECT src/Bluetooth.cpp)
add_library(finger_abduction_processor OBJECT src/FingerAbductionProcessor.cpp)
add_library(joint_rom_processor OBJECT src/JointRomProcessor.cpp)
add_library(wrist_orientation_processor OBJECT src/WristOrientationProcessor.cpp)




target_include_directories(blood_oxygen_processor PUBLIC include/)
target_include_directories(bluetooth PUBLIC include/)
target_include_directories(finger_abduction_processor PUBLIC include/)
target_include_directories(joint_rom_processor PUBLIC include/)
target_include_directories(wrist_orientation_processor PUBLIC include/)


target_link_libraries(bluetooth PUBLIC simpleble::simpleble)
target_link_libraries(finger_abduction_processor PUBLIC InEKF::Core PUBLIC InEKF::SE2Models PUBLIC InEKF::Inertial)
target_link_libraries(wrist_orientation_processor PUBLIC InEKF::Core PUBLIC InEKF::SE2Models PUBLIC InEKF::Inertial)
# May have MQTT related class later


# External targets used here:
# - simpleble::simpleble
# - InEKF::Core, InEKF::SE2Models, InEKF::Inertial
# - PahoMqttCpp::paho-mqttpp3
