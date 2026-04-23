#ifndef MQTT_WORKER_H
#define MQTT_WORKER_H

#include "MQTTClient.h"
#include "QueueMessageTypes.h"
#include "SessionCommand.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <stop_token>
#include <string>
#include <thread>


class MQTTWorker{
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
    static const std::string TOPIC_FORCE_POINTER;
    static const std::string TOPIC_FORCE_MIDDLE;
    static const std::string TOPIC_FORCE_THUMB;
    static const std::string TOPIC_FORCE_RING;
    static const std::string TOPIC_FORCE_PINKY;
    static const std::string TOPIC_ORIENTATION_WRIST_X;
    static const std::string TOPIC_ORIENTATION_WRIST_Y;
    static const std::string TOPIC_SPO2_WRIST;
    static const std::string TOPIC_SYSTEM_COMMAND;
    static const std::string TOPIC_SYSTEM_CALIBRATION_STATUS;

    int connectionAttemptCount;
    bool subFailed;
    bool pubFailed;

    std::queue<SessionCommand> * mqttForwardCommandQueue;
    std::queue<DataOutputElement> * flexSPO2ForwardMQTTQueue;
    std::queue<DataOutputElement> * imuForceForwardMQTTQueue;
    std::queue<CalibrationStatusMessage> * calibrationStatusQueue;

    std::mutex * mqttForwardCommandMutex;
    std::mutex * flexSPO2ForwardMQTTMutex;
    std::mutex * imuForceForwardMQTTMutex;
    std::mutex * calibrationStatusMutex;

    std::mutex callbackStateMutex;
    std::mutex clientAccessMutex;

    std::unique_ptr<MQTTClient> ownedMqttClient;
    MQTTClient * mqttClient;

    bool calibrationEpochActive;
    uint32_t currentCalibrationEpoch;
    uint32_t calibrationMessagesReceived;
    uint32_t calibrationMessagesRequired;

    std::string getTopic(SensorID id);
    void onMessageArrived(const std::string& topic, const std::string& payload);
    bool connectAndSubscribe();
    bool publishWithRetries(const std::string& topic, const std::string& payload);

    void publishData(std::mutex * queueMut, std::queue<DataOutputElement> * elemQueue);
    void publishCalibrationStatus();

    public:
        MQTTWorker();
        explicit MQTTWorker(MQTTClient * mqttClient);

        bool initialize(std::queue<SessionCommand> * mqttForwardCommandQueue,
                        std::queue<DataOutputElement> * flexSPO2ForwardMQTTQueue,
                        std::queue<DataOutputElement> * imuForceForwardMQTTQueue,
                        std::queue<CalibrationStatusMessage> * calibrationStatusQueue,
                        std::mutex * mqttForwardCommandMutex,
                        std::mutex * flexSPO2ForwardMQTTMutex,
                        std::mutex * imuForceForwardMQTTMutex,
                        std::mutex * calibrationStatusMutex);

        void run(std::stop_token stopToken);
};


#endif
