#include "MQTTWorker.h"

#include "Logger.h"
#include "PahoMQTTClient.h"


const int MQTTWorker::MAX_RECONNECT_ATTEMPTS = 10;
const int MQTTWorker::MAX_PUBLISH_ATTEMPTS = 10;
const int MQTTWorker::QOS = 1;

const std::string MQTTWorker::SERVER_URI = "mqtt://localhost:1883";
const std::string MQTTWorker::CLIENT_ID = "esp32_glove_main";

const std::string MQTTWorker::TOPIC_JOINT_POINTER_MCP = "joint/POINTER_MCP";
const std::string MQTTWorker::TOPIC_JOINT_POINTER_PIP = "joint/POINTER_PIP";
const std::string MQTTWorker::TOPIC_JOINT_POINTER_DIP = "joint/POINTER_DIP";
const std::string MQTTWorker::TOPIC_JOINT_MIDDLE_MCP = "joint/MIDDLE_MCP";
const std::string MQTTWorker::TOPIC_JOINT_MIDDLE_PIP = "joint/MIDDLE_PIP";
const std::string MQTTWorker::TOPIC_JOINT_MIDDLE_DIP = "joint/MIDDLE_DIP";
const std::string MQTTWorker::TOPIC_JOINT_RING_MCP = "joint/RING_MCP";
const std::string MQTTWorker::TOPIC_JOINT_RING_PIP = "joint/RING_PIP";
const std::string MQTTWorker::TOPIC_JOINT_RING_DIP = "joint/RING_DIP";
const std::string MQTTWorker::TOPIC_JOINT_PINKY_MCP = "joint/PINKY_MCP";
const std::string MQTTWorker::TOPIC_JOINT_PINKY_PIP = "joint/PINKY_PIP";
const std::string MQTTWorker::TOPIC_JOINT_PINKY_DIP = "joint/PINKY_DIP";
const std::string MQTTWorker::TOPIC_JOINT_THUMB_MCP = "joint/THUMB_MCP";
const std::string MQTTWorker::TOPIC_JOINT_THUMB_PIP = "joint/THUMB_PIP";
const std::string MQTTWorker::TOPIC_ABDUCTION_POINTER = "abduction/POINTER";
const std::string MQTTWorker::TOPIC_ABDUCTION_MIDDLE = "abduction/MIDDLE";
const std::string MQTTWorker::TOPIC_ABDUCTION_RING = "abduction/RING";
const std::string MQTTWorker::TOPIC_ABDUCTION_PINKY = "abduction/PINKY";
const std::string MQTTWorker::TOPIC_ABDUCTION_THUMB = "abduction/THUMB";
const std::string MQTTWorker::TOPIC_FORCE_POINTER = "force/POINTER";
const std::string MQTTWorker::TOPIC_FORCE_MIDDLE = "force/MIDDLE";
const std::string MQTTWorker::TOPIC_FORCE_THUMB = "force/THUMB";
const std::string MQTTWorker::TOPIC_FORCE_RING = "force/RING";
const std::string MQTTWorker::TOPIC_FORCE_PINKY = "force/PINKY";
const std::string MQTTWorker::TOPIC_ORIENTATION_WRIST_X = "orientation/WRIST_X";
const std::string MQTTWorker::TOPIC_ORIENTATION_WRIST_Y = "orientation/WRIST_Y";
const std::string MQTTWorker::TOPIC_SPO2_WRIST = "spo2/WRIST";
const std::string MQTTWorker::TOPIC_SYSTEM_COMMAND = "system/command";
const std::string MQTTWorker::TOPIC_SYSTEM_CALIBRATION_STATUS = "system/calibration_status";


MQTTWorker::MQTTWorker() :
    connectionAttemptCount(0),
    subFailed(false),
    pubFailed(false),
    mqttForwardCommandQueue(nullptr),
    flexSPO2ForwardMQTTQueue(nullptr),
    imuForceForwardMQTTQueue(nullptr),
    calibrationStatusQueue(nullptr),
    mqttForwardCommandMutex(nullptr),
    flexSPO2ForwardMQTTMutex(nullptr),
    imuForceForwardMQTTMutex(nullptr),
    calibrationStatusMutex(nullptr),
    callbackStateMutex(),
    clientAccessMutex(),
    failureStateMutex(),
    ownedMqttClient(nullptr),
    mqttClient(nullptr),
    workerFailed(false),
    failureReason(),
    calibrationEpochActive(false),
    currentCalibrationEpoch(0),
    calibrationMessagesReceived(0),
    calibrationMessagesRequired(0) {}


MQTTWorker::MQTTWorker(MQTTClient * mqttClient) :
    connectionAttemptCount(0),
    subFailed(false),
    pubFailed(false),
    mqttForwardCommandQueue(nullptr),
    flexSPO2ForwardMQTTQueue(nullptr),
    imuForceForwardMQTTQueue(nullptr),
    calibrationStatusQueue(nullptr),
    mqttForwardCommandMutex(nullptr),
    flexSPO2ForwardMQTTMutex(nullptr),
    imuForceForwardMQTTMutex(nullptr),
    calibrationStatusMutex(nullptr),
    callbackStateMutex(),
    clientAccessMutex(),
    failureStateMutex(),
    ownedMqttClient(nullptr),
    mqttClient(nullptr),
    workerFailed(false),
    failureReason(),
    calibrationEpochActive(false),
    currentCalibrationEpoch(0),
    calibrationMessagesReceived(0),
    calibrationMessagesRequired(0) {

}


bool MQTTWorker::initialize(std::queue<SessionCommand> * mqttForwardCommandQueue,
                            std::queue<DataOutputElement> * flexSPO2ForwardMQTTQueue,
                            std::queue<DataOutputElement> * imuForceForwardMQTTQueue,
                            std::queue<CalibrationStatusMessage> * calibrationStatusQueue,
                            std::mutex * mqttForwardCommandMutex,
                            std::mutex * flexSPO2ForwardMQTTMutex,
                            std::mutex * imuForceForwardMQTTMutex,
                            std::mutex * calibrationStatusMutex) {
    clearFailure();

    ownedMqttClient = std::make_unique<PahoMQTTClient>(SERVER_URI, CLIENT_ID);
    this->mqttClient = ownedMqttClient.get();

    this->mqttForwardCommandQueue = mqttForwardCommandQueue;
    this->flexSPO2ForwardMQTTQueue = flexSPO2ForwardMQTTQueue;
    this->imuForceForwardMQTTQueue = imuForceForwardMQTTQueue;
    this->calibrationStatusQueue = calibrationStatusQueue;
    this->mqttForwardCommandMutex = mqttForwardCommandMutex;
    this->flexSPO2ForwardMQTTMutex = flexSPO2ForwardMQTTMutex;
    this->imuForceForwardMQTTMutex = imuForceForwardMQTTMutex;
    this->calibrationStatusMutex = calibrationStatusMutex;

    Logger::instance().info("MQTTWorker",
                            "Initializing MQTT worker and attaching message handler.",
                            false);

    mqttClient->setMessageHandler([this](const std::string& topic, const std::string& payload){
        this->onMessageArrived(topic, payload);
    });

    if(!connectAndSubscribe()){
        if(!hasFailure()){
            setFailure("MQTT initialize failed during connect/subscribe.");
        }
        return false;
    }

    std::lock_guard guard(callbackStateMutex);
    return !subFailed && (connectionAttemptCount < MAX_RECONNECT_ATTEMPTS);
}


void MQTTWorker::run(std::stop_token stopToken){

    while(!stopToken.stop_requested()){
        {
            std::lock_guard guard(callbackStateMutex);
            if(subFailed || pubFailed || connectionAttemptCount >= MAX_RECONNECT_ATTEMPTS){
                return;
            }
        }

        bool connected = false;
        {
            std::lock_guard clientGuard(clientAccessMutex);
            connected = mqttClient->isConnected();
        }

        if(!connected){
            if(!connectAndSubscribe()){
                std::lock_guard guard(callbackStateMutex);
                if(subFailed || connectionAttemptCount >= MAX_RECONNECT_ATTEMPTS){
                    return;
                }
            }
            continue;
        }

        publishData(flexSPO2ForwardMQTTMutex, flexSPO2ForwardMQTTQueue);
        publishData(imuForceForwardMQTTMutex, imuForceForwardMQTTQueue);
        publishCalibrationStatus();
    }
}


void MQTTWorker::onMessageArrived(const std::string& topic, const std::string& payload){
    (void)topic;

    if(payload.empty()){
        return;
    }

    unsigned int value = 0;
    try {
        value = static_cast<unsigned int>(std::stoul(payload));
    } catch (...) {
        return;
    }

    if(value >= NUM_OF_COMMANDS){
        return;
    }

    std::lock_guard commandGuard(*mqttForwardCommandMutex);
    mqttForwardCommandQueue->push(static_cast<SessionCommand>(value));
}


bool MQTTWorker::connectAndSubscribe(){
    while(true){
        int attemptNumber = 0;
        {
            std::lock_guard stateGuard(callbackStateMutex);
            if(connectionAttemptCount >= MAX_RECONNECT_ATTEMPTS){
                setFailure("Failed to connect to MQTT broker after max reconnect attempts.");
                Logger::instance().error("MQTTWorker",
                                         "Failed to connect to MQTT broker after max reconnect attempts.",
                                         true);
                return false;
            }
            attemptNumber = connectionAttemptCount + 1;
        }

        Logger::instance().info("MQTTWorker",
                                "MQTT connect attempt " + std::to_string(attemptNumber) + "/" +
                                    std::to_string(MAX_RECONNECT_ATTEMPTS),
                                false);

        bool connected = false;
        {
            std::lock_guard clientGuard(clientAccessMutex);
            connected = mqttClient->connect();
        }

        if(connected){
            Logger::instance().info("MQTTWorker",
                                    "MQTT connect succeeded on attempt " +
                                        std::to_string(attemptNumber) + ".",
                                    false);
            break;
        }

        {
            std::lock_guard stateGuard(callbackStateMutex);
            connectionAttemptCount++;
        }

        Logger::instance().warn("MQTTWorker",
                                "MQTT connect attempt " + std::to_string(attemptNumber) +
                                    " failed. retry_count=" +
                                    std::to_string(connectionAttemptCount) + "/" +
                                    std::to_string(MAX_RECONNECT_ATTEMPTS),
                                false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    bool subscribed = false;
    {
        std::lock_guard clientGuard(clientAccessMutex);
        subscribed = mqttClient->subscribe(TOPIC_SYSTEM_COMMAND, QOS);
    }

    if(!subscribed){
        std::lock_guard stateGuard(callbackStateMutex);
        subFailed = true;
        setFailure("Connected to MQTT broker but subscribe failed for command topic.");
        Logger::instance().error("MQTTWorker",
                                 "Connected to MQTT broker but subscribe failed for command topic '" +
                                     TOPIC_SYSTEM_COMMAND + "'.",
                                 true);
        return false;
    }

    Logger::instance().info("MQTTWorker",
                            "Connected to MQTT broker and subscribed to command topic.",
                            true);
    return true;
}


bool MQTTWorker::publishWithRetries(const std::string& topic, const std::string& payload){
    for(int attempts = 0; attempts <= MAX_PUBLISH_ATTEMPTS; attempts++){
        const int attemptNumber = attempts + 1;
        bool success = false;
        {
            std::lock_guard clientGuard(clientAccessMutex);
            success = mqttClient->publish(topic, payload, QOS);
        }

        if(success){
            if(attemptNumber > 1){
                Logger::instance().info("MQTTWorker",
                                        "Publish succeeded for topic '" + topic + "' on retry " +
                                            std::to_string(attemptNumber) + ".",
                                        false);
            }
            return true;
        }

        Logger::instance().warn("MQTTWorker",
                                "Publish attempt " + std::to_string(attemptNumber) + "/" +
                                    std::to_string(MAX_PUBLISH_ATTEMPTS + 1) +
                                    " failed for topic '" + topic + "'.",
                                false);
    }

    std::lock_guard stateGuard(callbackStateMutex);
    pubFailed = true;
    setFailure("Publish failed for topic '" + topic + "' after max attempts.");
    return false;
}


void MQTTWorker::publishData(std::mutex * queueMut, std::queue<DataOutputElement> * elemQueue){
    std::lock_guard queueGuard(*queueMut);
    if(elemQueue->empty()){
        return;
    }

    DataOutputElement elem = elemQueue->front();
    std::string topic = getTopic(elem.id);
    if(topic.empty()){
        elemQueue->pop();
        return;
    }

    publishWithRetries(topic, elem.data);
    elemQueue->pop();
}


void MQTTWorker::publishCalibrationStatus(){
    CalibrationStatusMessage calibrationMessage;
    {
        std::lock_guard queueGuard(*calibrationStatusMutex);
        if(calibrationStatusQueue->empty()){
            return;
        }
        calibrationMessage = calibrationStatusQueue->front();
        calibrationStatusQueue->pop();
    }

    if(calibrationMessage.requiredCount == 0){
        return;
    }

    if(!calibrationEpochActive || calibrationMessage.epoch != currentCalibrationEpoch){
        calibrationEpochActive = true;
        currentCalibrationEpoch = calibrationMessage.epoch;
        calibrationMessagesReceived = 1;
        calibrationMessagesRequired = calibrationMessage.requiredCount;
    } else {
        calibrationMessagesReceived++;
    }

    if(calibrationMessagesReceived < calibrationMessagesRequired){
        return;
    }

    const std::string payload = std::to_string(static_cast<int>(SessionCommand::CALIBRATION_COMPLETED));
    const bool published = publishWithRetries(TOPIC_SYSTEM_CALIBRATION_STATUS, payload);
    if(published){
        Logger::instance().info("MQTTWorker",
                                "Published calibration completion status to MQTT.",
                                true);
    } else {
        Logger::instance().error("MQTTWorker",
                                 "Failed to publish calibration completion status to MQTT.",
                                 true);
    }

    calibrationEpochActive = false;
    calibrationMessagesReceived = 0;
    calibrationMessagesRequired = 0;
}

void MQTTWorker::setFailure(const std::string& reason){
    std::lock_guard guard(failureStateMutex);
    workerFailed = true;
    failureReason = reason;
    Logger::instance().error("MQTTWorker", reason, false);
}

void MQTTWorker::clearFailure(){
    std::lock_guard guard(failureStateMutex);
    workerFailed = false;
    failureReason.clear();
}

bool MQTTWorker::hasFailure(){
    std::lock_guard guard(failureStateMutex);
    return workerFailed;
}

std::string MQTTWorker::getFailureReason(){
    std::lock_guard guard(failureStateMutex);
    return failureReason;
}


std::string MQTTWorker::getTopic(SensorID id){
    switch(id){
        case SensorID::WRIST_SPO2:
            return TOPIC_SPO2_WRIST;

        case SensorID::HAND_IMU_X:
            return TOPIC_ORIENTATION_WRIST_X;
        case SensorID::HAND_IMU_Y:
            return TOPIC_ORIENTATION_WRIST_Y;
        case SensorID::POINTER_IMU:
            return TOPIC_ABDUCTION_POINTER;
        case SensorID::MIDDLE_IMU:
            return TOPIC_ABDUCTION_MIDDLE;
        case SensorID::THUMB_IMU:
            return TOPIC_ABDUCTION_THUMB;
        case SensorID::RING_IMU:
            return TOPIC_ABDUCTION_RING;
        case SensorID::PINKY_IMU:
            return TOPIC_ABDUCTION_PINKY;

        case SensorID::POINTER_MCP_FLEX:
            return TOPIC_JOINT_POINTER_MCP;
        case SensorID::POINTER_PIP_FLEX:
            return TOPIC_JOINT_POINTER_PIP;
        case SensorID::POINTER_DIP_FLEX:
            return TOPIC_JOINT_POINTER_DIP;
        case SensorID::MIDDLE_MCP_FLEX:
            return TOPIC_JOINT_MIDDLE_MCP;
        case SensorID::MIDDLE_PIP_FLEX:
            return TOPIC_JOINT_MIDDLE_PIP;
        case SensorID::MIDDLE_DIP_FLEX:
            return TOPIC_JOINT_MIDDLE_DIP;
        case SensorID::RING_MCP_FLEX:
            return TOPIC_JOINT_RING_MCP;
        case SensorID::RING_PIP_FLEX:
            return TOPIC_JOINT_RING_PIP;
        case SensorID::RING_DIP_FLEX:
            return TOPIC_JOINT_RING_DIP;
        case SensorID::PINKY_MCP_FLEX:
            return TOPIC_JOINT_PINKY_MCP;
        case SensorID::PINKY_PIP_FLEX:
            return TOPIC_JOINT_PINKY_PIP;
        case SensorID::PINKY_DIP_FLEX:
            return TOPIC_JOINT_PINKY_DIP;
        case SensorID::THUMB_MCP_FLEX:
            return TOPIC_JOINT_THUMB_MCP;
        case SensorID::THUMB_PIP_FLEX:
            return TOPIC_JOINT_THUMB_PIP;

        case SensorID::POINTER_FORCE:
            return TOPIC_FORCE_POINTER;
        case SensorID::MIDDLE_FORCE:
            return TOPIC_FORCE_MIDDLE;
        case SensorID::THUMB_FORCE:
            return TOPIC_FORCE_THUMB;
        case SensorID::RING_FORCE:
            return TOPIC_FORCE_RING;
        case SensorID::PINKY_FORCE:
            return TOPIC_FORCE_PINKY;
    }

    return "";
}
