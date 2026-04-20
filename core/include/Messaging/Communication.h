#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <cstdint>
#include <vector>

enum class Endpoints;

class Communication {
  public:
    virtual ~Communication() = default;

    virtual bool read(const Endpoints& endpoint, std::vector<uint8_t>& message) = 0;
    virtual bool write(const Endpoints& endpoint, std::vector<uint8_t>& message) = 0;
    virtual bool isConnected() = 0;
};

#endif
