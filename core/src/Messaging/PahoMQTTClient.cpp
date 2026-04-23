#include "PahoMQTTClient.h"

#include <utility>


PahoMQTTClient::Callback::Callback() : mqttClient(nullptr) {};


PahoMQTTClient::Callback::Callback(PahoMQTTClient * mqttClient) : mqttClient(mqttClient) {};


void PahoMQTTClient::Callback::connected(const std::string& cause) {}


void PahoMQTTClient::Callback::delivery_complete(mqtt::delivery_token_ptr token) {}


PahoMQTTClient::PahoMQTTClient(const std::string& serverURI, const std::string& clientID) :
    connectOptions(),
    client(serverURI, clientID),
    callback(this),
    clientAccessMutex(),
    messageHandlerMutex(),
    messageHandler(nullptr) {

    connectOptions.set_mqtt_version(MQTTVERSION_3_1_1);
    connectOptions.set_clean_session(false);
    connectOptions.set_connect_timeout(3000);
    connectOptions.set_automatic_reconnect(false);

    client.set_callback(callback);
}


bool PahoMQTTClient::connect() {
    std::lock_guard guard(clientAccessMutex);

    try {
        client.connect(connectOptions)->wait();
    } catch (...) {
        return false;
    }

    return client.is_connected();
}


bool PahoMQTTClient::subscribe(const std::string& topic, int qos) {
    std::lock_guard guard(clientAccessMutex);

    if(!client.is_connected()){
        return false;
    }

    try {
        client.subscribe(topic, qos)->wait();
    } catch (...) {
        return false;
    }

    return true;
}


bool PahoMQTTClient::publish(const std::string& topic, const std::string& payload, int qos) {
    std::lock_guard guard(clientAccessMutex);

    if(!client.is_connected()){
        return false;
    }

    try {
        client.publish(topic, payload.data(), payload.size(), qos, false)->wait();
    } catch (...) {
        return false;
    }

    return true;
}


bool PahoMQTTClient::isConnected() {
    std::lock_guard guard(clientAccessMutex);
    return client.is_connected();
}


void PahoMQTTClient::setMessageHandler(MessageHandler handler) {
    std::lock_guard guard(messageHandlerMutex);
    messageHandler = std::move(handler);
}


void PahoMQTTClient::onMessageArrived(const std::string& topic, const std::string& payload){
    MessageHandler handler;
    {
        std::lock_guard guard(messageHandlerMutex);
        handler = messageHandler;
    }

    if(handler){
        handler(topic, payload);
    }
}


void PahoMQTTClient::onConnectionLost() {
    // Reconnect policy lives outside the transport wrapper.
}


void PahoMQTTClient::Callback::connection_lost(const std::string& cause){
    if(mqttClient == nullptr){
        return;
    }

    mqttClient->onConnectionLost();
}


void PahoMQTTClient::Callback::message_arrived(mqtt::const_message_ptr message){
    if(mqttClient == nullptr || message == nullptr){
        return;
    }

    mqttClient->onMessageArrived(message->get_topic(), message->to_string());
}
