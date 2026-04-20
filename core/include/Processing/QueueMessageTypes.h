#ifndef QUEUE_MESSAGE_TYPES_H
#define QUEUE_MESSAGE_TYPES_H

enum class SensorType{
   IMU_GYRO,
   IMU_ACCEL,
   FLEX,
   SPO2
};

enum class SensorID{
    WRIST_SPO2,
    HAND_IMU,
    POINTER_IMU,
    MIDDLE_IMU,
    THUMB_IMU,
    RING_IMU,
    PINKY_IMU,
    POINTER_MCP_FLEX,
    POINTER_PIP_FLEX,
    POINTER_DIP_FLEX,
    MIDDLE_MCP_FLEX,
    MIDDLE_PIP_FLEX,
    MIDDLE_DIP_FLEX,
    RING_MCP_FLEX,
    RING_PIP_FLEX,
    RING_DIP_FLEX,
    PINKY_MCP_FLEX,
    PINKY_PIP_FLEX,
    PINKY_DIP_FLEX,
    THUMB_MCP_FLEX,
    THUMB_PIP_FLEX
};

struct dataOutputElement {
    SensorID id;
    std::string data; 
};




#endif