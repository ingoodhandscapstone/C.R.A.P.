#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <functional>
#include <string>


class MQTTClient {
    public:
        using MessageHandler = std::function<void(const std::string& topic, const std::string& payload)>;

        virtual ~MQTTClient() = default;

        virtual bool connect() = 0;
        virtual bool subscribe(const std::string& topic, int qos) = 0;
        virtual bool publish(const std::string& topic, const std::string& payload, int qos) = 0;
        virtual bool isConnected() = 0;
        virtual void setMessageHandler(MessageHandler handler) = 0;
};


#endif
