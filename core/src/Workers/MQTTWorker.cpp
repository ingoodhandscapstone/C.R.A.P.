#include "MQTTWorker.h"

#include <algorithm>
#include <cctype>

const int MQTTWorker::MAX_RECONNECT_ATTEMPTS = 5;
const int MQTTWorker::MAX_PUBLISH_ATTEMPTS = 5;
const int MQTTWorker::QOS = 1;

const std::string MQTTWorker::SERVER_URI = "";
const std::string MQTTWorker::CLIENT_ID = "";

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
const std::string MQTTWorker::TOPIC_ORIENTATION_WRIST_X = "orientation/WRIST_X";
const std::string MQTTWorker::TOPIC_ORIENTATION_WRIST_Y = "orientation/WRIST_Y";
const std::string MQTTWorker::TOPIC_SPO2_WRIST = "spo2/WRIST";
const std::string MQTTWorker::TOPIC_SYSTEM_COMMAND = "system/command";
const std::string MQTTWorker::TOPIC_SYSTEM_CALIBRATION_STATUS = "system/calibration_status";



bool MQTTWorker::initialize(std::queue<SessionCommand> * mqttForwardCommandQueue,
                            std::queue<DataOutputElement> * flexSPO2ForwardMQTTQueue,
                            std::queue<DataOutputElement> * imuForceForwardMQTTQueue,
                            std::queue<CalibrationStatusMessage> * calibrationStatusQueue,
                            std::mutex * mqttForwardCommandMutex,
                            std::mutex * flexSPO2ForwardMQTTMutex,
                            std::mutex * imuForceForwardMQTTMutex,
                            std::mutex * calibrationStatusMutex) {
    this->mqttForwardCommandQueue = mqttForwardCommandQueue;
    this->flexSPO2ForwardMQTTQueue = flexSPO2ForwardMQTTQueue;
    this->imuForceForwardMQTTQueue = imuForceForwardMQTTQueue;
    this->calibrationStatusQueue = calibrationStatusQueue;
    this->mqttForwardCommandMutex = mqttForwardCommandMutex;
    this->flexSPO2ForwardMQTTMutex = flexSPO2ForwardMQTTMutex;
    this->imuForceForwardMQTTMutex = imuForceForwardMQTTMutex;
    this->calibrationStatusMutex = calibrationStatusMutex;
    
    // Set connect options
    connect_opts.set_mqtt_version(MQTTVERSION_3_1_1);
    connect_opts.set_clean_session(false);
    connect_opts.set_connect_timeout(3000);
    connect_opts.set_automatic_reconnect(false);

    // Set callback for client 
    cb = Callback(this);
    // Setup listeners
    subAL = SubscriberActionListener(this);
    pubAL = PublisherActionListener(this);

    // client.connect() passing cb, subscription should then be automatic
    client.connect(connect_opts, nullptr, cb);

    // Use mutex before checking
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Give some time for subscription to occur ??
    std::lock_guard guard(callbackStateMutex);
    
    // If subscription did not fail and we didn't 
    return !subFailed && (connectionAttemptCount < MAX_RECONNECT_ATTEMPTS); 
}


void MQTTWorker::run(){

    while(true){
    // Checking connection attempt count
    // Check for publish fail flag 
    // Check subscription fail flag (really should only be set if reconnet occurs here)
        {
            std::lock_guard guard(callbackStateMutex);
            if(subFailed || pubFailed || connectionAttemptCount >= MAX_RECONNECT_ATTEMPTS){
                return; // Leave this to main for further handling 
            }

        }

        publishData(flexSPO2ForwardMQTTMutex, flexSPO2ForwardMQTTQueue);
        publishData(imuForceForwardMQTTMutex, imuForceForwardMQTTQueue);
        publishCalibrationStatus();

    }
   
}

void MQTTWorker::Callback::connection_lost(const std::string& cause){
    reconnect(); // If this fails on_failure() gets called and that will carry out the additional attempts
}


void MQTTWorker::Callback::on_failure(const mqtt::token& asyncActionToken) {
    reconnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    

}

void MQTTWorker::Callback::reconnect(){
    // Assumes this reconnect ends (connect returns immediately) before, in the instance of a failure, the on_failure get called in thread (otherwise would deadlock)
    {
        std::lock_guard guard(worker->callbackStateMutex);
        if(worker->connectionAttemptCount >= MQTTWorker::MAX_RECONNECT_ATTEMPTS){
            return;
        }
        worker->connectionAttemptCount++;
    }

    {
        std::lock_guard guard(worker->clientAccessMutex);
        worker->client.connect(worker->connect_opts, nullptr, *this); // Uses itself as action listener so as to repeatedly call on_failure
    }

}

void MQTTWorker::Callback::connected(const std::string& cause){
    std::lock_guard guard(worker->clientAccessMutex);
    worker->client.subscribe(MQTTWorker::TOPIC_SYSTEM_COMMAND, QOS);
}


// We do not know whether the is a random ascii value but identical underlying number as command or string literal of the number which corresponds to command
// Chat did this and it will likely be changed
void MQTTWorker::Callback::message_arrived(mqtt::const_message_ptr message){
    std::lock_guard commandGuard(*(worker->mqttForwardCommandMutex));
    std::string payload = message.get()->get_payload();

    if(payload.empty()){
        return;
    }

    auto isValidCommandValue = [](unsigned int value) {
        return value <= static_cast<unsigned int>(SessionCommand::SESSION_CONFIG_GRIPPER);
    };

    // Accept either numeric text payloads (e.g. "5") or raw single-byte command payloads.
    const bool isNumericPayload = std::all_of(payload.begin(), payload.end(), [](unsigned char c){
        return std::isdigit(c) != 0;
    });

    if(isNumericPayload){
        try {
            const unsigned int value = static_cast<unsigned int>(std::stoul(payload));
            if(isValidCommandValue(value)){
                worker->mqttForwardCommandQueue->push(static_cast<SessionCommand>(value));
            }
        } catch (...) {
            // Ignore malformed command payloads.
        }
        return;
    }

    const unsigned int rawValue = static_cast<unsigned int>(static_cast<uint8_t>(payload[0]));
    if(isValidCommandValue(rawValue)){
        worker->mqttForwardCommandQueue->push(static_cast<SessionCommand>(rawValue));
    }
}

void MQTTWorker::SubscriberActionListener::on_failure(const mqtt::token& asyncActionToken){
    // Just setting the flag so MQTTWorker can handle it
    std::lock_guard guard(worker->callbackStateMutex);
    worker->subFailed = true;
}




void MQTTWorker::PublisherActionListener::on_success(const mqtt::token& asyncActionToken){
    std::lock_guard guard(worker->callbackStateMutex);

    int tokenKey = asyncActionToken.get_message_id();
    mqtt::delivery_token_ptr correspondingToken = worker->lastPublishedTokenMap.at(tokenKey);

    worker->lastPublishedTokenMap.erase(tokenKey);

}

void MQTTWorker::PublisherActionListener::on_failure(const mqtt::token& asyncActionToken){
    mqtt::const_message_ptr message;

    int tokenKey = asyncActionToken.get_message_id();
    {
        std::lock_guard guard(worker->callbackStateMutex);
        mqtt::delivery_token_ptr correspondingToken = worker->lastPublishedTokenMap.at(tokenKey);

        worker->lastPublishedTokenMap.erase(tokenKey);

        if(republishAttempts >= MQTTWorker::MAX_PUBLISH_ATTEMPTS){ 
           worker->pubFailed = true;
           return;
        }

        message = correspondingToken->get_message();

        republishAttempts++;
    }

    mqtt::delivery_token_ptr newToken;
    {
        std::lock_guard guard(worker->clientAccessMutex);
        // Already includes topic 
        newToken = worker->client.publish(message, nullptr, *this); // Maybe change publishing settings
    }

    {
        std::lock_guard guard(worker->callbackStateMutex);
        worker->lastPublishedTokenMap.insert({newToken->get_message_id(), newToken});
    }

    
}


void MQTTWorker::publishData(std::mutex * queueMut, std::queue<DataOutputElement> * elemQueue){
    // Access mutex for queue
    std::lock_guard queueGuard(*queueMut);
    // check empty
    if(elemQueue->empty()){
        return;
    }

    // Access element
    DataOutputElement elem = elemQueue->front();
    // Get topic 
    mqtt::string_ref topic = getTopic(elem.id);
    // Extract data from elem
    mqtt::binary_ref data = elem.data;
    // place into message to send
    const mqtt::message messageToPub(topic, data);
    mqtt::const_message_ptr messagePtr = std::make_shared<const mqtt::message>(messageToPub);

    std::lock_guard clientGuard(clientAccessMutex);
    client.publish(messagePtr);

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
    mqtt::const_message_ptr messagePtr = std::make_shared<const mqtt::message>(
        MQTTWorker::TOPIC_SYSTEM_CALIBRATION_STATUS, payload);

    {
        std::lock_guard clientGuard(clientAccessMutex);
        client.publish(messagePtr);
    }

    calibrationEpochActive = false;
    calibrationMessagesReceived = 0;
    calibrationMessagesRequired = 0;
}


std::string MQTTWorker::getTopic(SensorID id){
    
}
