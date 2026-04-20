#ifndef COMMUNICAION_H
#define COMMUNICATION_H

#include <cstdint>


enum class Endpoint {
    IMU_GYRO_CHAR,
    IMU_ACCEL_CHAR,
    FLEX_CHAR,
    SPO2_CHAR,
    COMMAND_CHAR
};


class Communication {

    public:

        virtual bool read(const Endpoint& endpoint, uint8_t * message, uint16_t size) = 0;
        virtual bool write(const Endpoint& endpoint, uint8_t * message, uint16_t size ) = 0;
        virtual bool isConnected() = 0;
};




#endif