#ifndef MQTT_WORKER_H
#define MQTT_WORKER_H

#include "mqtt/async_client.h"

#include "QueueMessageTypes.h"

#include <queue>
#include <mutex>
#include <chrono>
#include <thread>
#include <string>
#include <unordered_map>


class MQTTWorker{

    class Callback : public virtual mqtt::callback, public virtual mqtt::iaction_listener {


            MQTTWorker * worker;

            public:

                Callback() : worker(nullptr) {};
                Callback(MQTTWorker * worker) : worker(worker) {};

            
            private:

                void connected(const std::string& cause) override; 
                void connection_lost(const std::string& cause) override; // Attempt to reconnect
                void message_arrived(mqtt::const_message_ptr message) override; // 
                void delivery_complete(mqtt::delivery_token_ptr) override {}; // Do Nothing
                void on_success(const mqtt::token& asyncActionToken) override {}; // Connection Success Do Nothing
                void on_failure(const mqtt::token& asyncActionToken) override; // Reconnect failure

                void reconnect();

    };

    class PublisherActionListener : public virtual mqtt::iaction_listener {

        MQTTWorker * worker;

        int republishAttempts;

        public:

            PublisherActionListener () : worker(nullptr), republishAttempts(0) {};
            PublisherActionListener (MQTTWorker * worker) : worker(worker), republishAttempts(0) {};

        void on_success(const mqtt::token& asyncActionToken) override; // On the success
        void on_failure(const mqtt::token& asyncActionToken) override; // On the failure of publishing

    };

    class SubscriberActionListener : public virtual mqtt::iaction_listener {

        MQTTWorker * worker;

        public:

            SubscriberActionListener () : worker(nullptr) {};
            SubscriberActionListener (MQTTWorker * worker) : worker(worker) {};

        void on_success(const mqtt::token& asyncActionToken) override {}; // Literally on success on subscribing
        void on_failure(const mqtt::token& asyncActionToken) override; // Literally failed subscribing

    };

    static const int MAX_RECONNECT_ATTEMPTS;
    static const int MAX_PUBLISH_ATTEMPTS;
    static const int QOS;

    static const std::string SERVER_URI;
    static const std::string CLIENT_ID;

    static const std::string TOPIC_JOINT_POINTER_MCP;
    static const std::string TOPIC_JOINT_POINTER_PIP;
    static const std::string TOPIC_JOINT_POINTER_DIP;
    static const std::string TOPIC_JOINT_MIDDLE_MCP;
    static const std::string TOPIC_JOINT_MIDDLE_PIP;
    static const std::string TOPIC_JOINT_MIDDLE_DIP;
    static const std::string TOPIC_JOINT_RING_MCP;
    static const std::string TOPIC_JOINT_RING_PIP;
    static const std::string TOPIC_JOINT_RING_DIP;
    static const std::string TOPIC_JOINT_PINKY_MCP;
    static const std::string TOPIC_JOINT_PINKY_PIP;
    static const std::string TOPIC_JOINT_PINKY_DIP;
    static const std::string TOPIC_JOINT_THUMB_MCP;
    static const std::string TOPIC_JOINT_THUMB_PIP;
    static const std::string TOPIC_ABDUCTION_POINTER;
    static const std::string TOPIC_ABDUCTION_MIDDLE;
    static const std::string TOPIC_ABDUCTION_RING;
    static const std::string TOPIC_ABDUCTION_PINKY;
    static const std::string TOPIC_ABDUCTION_THUMB;
    static const std::string TOPIC_ORIENTATION_WRIST_X;
    static const std::string TOPIC_ORIENTATION_WRIST_Y;
    static const std::string TOPIC_SPO2_WRIST;

    int connectionAttemptCount;
    bool subFailed;
    bool pubFailed;

    std::queue<uint8_t> * mqttForwardCommandQueue;
    std::queue <DataOutputElement> * flexSPO2ForwardMQTTQueue;
    std::queue<DataOutputElement> * imuForceForwardMQTTQueue;

    std::mutex * mqttForwardCommandMutex;
    std::mutex * flexSPO2ForwardMQTTMutex;
    std::mutex * imuForceForwardMQTTMutex;

    std::mutex callbackStateMutex; // For callback-shared member state (flags/counters/maps).
    std::mutex clientAccessMutex; // For mqtt::async_client calls shared across threads.

    mqtt::connect_options connect_opts;
    mqtt::async_client client;

    Callback cb; // Used for reconnection attempts on disconnect 
    SubscriberActionListener subAL; 
    PublisherActionListener pubAL; // Use this to verify message actually get published (certain amount of attempts)

    std::unordered_map<int, mqtt::delivery_token_ptr> lastPublishedTokenMap;

    // This assumes SensorID will be apart of of both queue elements
    std::string getTopic(SensorID id);
  
    void publishMessage(std::mutex * queueMut, std::queue<DataOutputElement> * elemQueue);

    public:
        MQTTWorker() :
            mqttForwardCommandQueue(nullptr),
            flexSPO2ForwardMQTTQueue(nullptr),
            imuForceForwardMQTTQueue(nullptr),
            mqttForwardCommandMutex(nullptr),
            flexSPO2ForwardMQTTMutex(nullptr),
            imuForceForwardMQTTMutex(nullptr),
            connectionAttemptCount(0),
            connect_opts(),
            client(SERVER_URI, CLIENT_ID),
            cb(),
            subAL(),
            pubAL(),
            callbackStateMutex(),
            clientAccessMutex(),
            subFailed(false),
            pubFailed(false),
            lastPublishedTokenMap() {};

        bool initialize(std::queue<uint8_t> * mqttForwardCommandQueue,
                        std::queue<DataOutputElement> * flexSPO2ForwardMQTTQueue,
                        std::queue<DataOutputElement> * imuForceForwardMQTTQueue,
                        std::mutex * mqttForwardCommandMutex,
                        std::mutex * flexSPO2ForwardMQTTMutex,
                        std::mutex * imuForceForwardMQTTMutex);

        void run();

};


#endif
