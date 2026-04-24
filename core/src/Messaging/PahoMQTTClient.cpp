#include "PahoMQTTClient.h"
#include "Logger.h"

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

    Logger::instance().info("PahoMQTTClient", "Attempting MQTT transport connect.", false);
    try {
        client.connect(connectOptions)->wait();
    } catch (const mqtt::exception& ex) {
        Logger::instance().error("PahoMQTTClient",
                                 std::string("MQTT transport connect threw mqtt::exception: ") + ex.what(),
                                 false);
        return false;
    } catch (const std::exception& ex) {
        Logger::instance().error("PahoMQTTClient",
                                 std::string("MQTT transport connect threw std::exception: ") + ex.what(),
                                 false);
        return false;
    } catch (...) {
        Logger::instance().error("PahoMQTTClient",
                                 "MQTT transport connect threw unknown exception.",
                                 false);
        return false;
    }

    const bool connected = client.is_connected();
    if(connected){
        Logger::instance().info("PahoMQTTClient", "MQTT transport connect successful.", false);
    } else {
        Logger::instance().warn("PahoMQTTClient",
                                "MQTT transport connect returned without exception but client is not connected.",
                                false);
    }

    return connected;
}


bool PahoMQTTClient::subscribe(const std::string& topic, int qos) {
    std::lock_guard guard(clientAccessMutex);

    if(!client.is_connected()){
        Logger::instance().warn("PahoMQTTClient",
                                "Subscribe failed because MQTT transport is not connected. topic=" + topic,
                                false);
        return false;
    }

    try {
        client.subscribe(topic, qos)->wait();
    } catch (const mqtt::exception& ex) {
        Logger::instance().error("PahoMQTTClient",
                                 "Subscribe failed for topic '" + topic +
                                     "' mqtt::exception: " + ex.what(),
                                 false);
        return false;
    } catch (const std::exception& ex) {
        Logger::instance().error("PahoMQTTClient",
                                 "Subscribe failed for topic '" + topic +
                                     "' std::exception: " + ex.what(),
                                 false);
        return false;
    } catch (...) {
        Logger::instance().error("PahoMQTTClient",
                                 "Subscribe failed for topic '" + topic + "' unknown exception.",
                                 false);
        return false;
    }

    Logger::instance().info("PahoMQTTClient",
                            "Subscribe successful for topic '" + topic + "'.",
                            false);
    return true;
}


bool PahoMQTTClient::publish(const std::string& topic, const std::string& payload, int qos) {
    std::lock_guard guard(clientAccessMutex);

    if(!client.is_connected()){
        Logger::instance().warn("PahoMQTTClient",
                                "Publish failed because MQTT transport is not connected. topic=" + topic,
                                false);
        return false;
    }

    try {
        client.publish(topic, payload.data(), payload.size(), qos, false)->wait();
    } catch (const mqtt::exception& ex) {
        Logger::instance().error("PahoMQTTClient",
                                 "Publish failed for topic '" + topic +
                                     "' mqtt::exception: " + ex.what(),
                                 false);
        return false;
    } catch (const std::exception& ex) {
        Logger::instance().error("PahoMQTTClient",
                                 "Publish failed for topic '" + topic +
                                     "' std::exception: " + ex.what(),
                                 false);
        return false;
    } catch (...) {
        Logger::instance().error("PahoMQTTClient",
                                 "Publish failed for topic '" + topic + "' unknown exception.",
                                 false);
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


void PahoMQTTClient::onConnectionLost(const std::string& cause) {
    // Reconnect policy lives outside the transport wrapper.
    Logger::instance().warn("PahoMQTTClient",
                            "MQTT connection lost. cause=" + cause,
                            false);
}


void PahoMQTTClient::Callback::connection_lost(const std::string& cause){
    if(mqttClient == nullptr){
        return;
    }

    mqttClient->onConnectionLost(cause);
}


void PahoMQTTClient::Callback::message_arrived(mqtt::const_message_ptr message){
    if(mqttClient == nullptr || message == nullptr){
        return;
    }

    mqttClient->onMessageArrived(message->get_topic(), message->to_string());
}
