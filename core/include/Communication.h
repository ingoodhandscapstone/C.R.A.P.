#ifndef COMMUNICAION_H
#define COMMUNICATION_H

#include <cstdint>


enum class Endpoint {

};


class Communication {
    virtual bool read(uint8_t * message, uint16_t size) = 0;
    virtual bool write(const Endpoint& endpoint, uint8_t * message, uint16_t size ) = 0;
    virtual bool isConnected() = 0;
};




#endif