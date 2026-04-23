#ifndef PAHO_MQTT_CLIENT_H
#define PAHO_MQTT_CLIENT_H

#include "MQTTClient.h"
#include "mqtt/async_client.h"

#include <mutex>
#include <string>


class PahoMQTTClient : public MQTTClient {

    class Callback : public virtual mqtt::callback {
        PahoMQTTClient * mqttClient;

        public:
            Callback();
            Callback(PahoMQTTClient * mqttClient);

        private:
            void connected(const std::string& cause) override;
            void connection_lost(const std::string& cause) override;
            void message_arrived(mqtt::const_message_ptr message) override;
            void delivery_complete(mqtt::delivery_token_ptr token) override;
    };

    mqtt::connect_options connectOptions;
    mqtt::async_client client;
    Callback callback;

    std::mutex clientAccessMutex;
    std::mutex messageHandlerMutex;
    MessageHandler messageHandler;

    void onMessageArrived(const std::string& topic, const std::string& payload);
    void onConnectionLost();

    public:
        PahoMQTTClient(const std::string& serverURI, const std::string& clientID);

        bool connect() override;
        bool subscribe(const std::string& topic, int qos) override;
        bool publish(const std::string& topic, const std::string& payload, int qos) override;
        bool isConnected() override;
        void setMessageHandler(MessageHandler handler) override;
};


#endif
