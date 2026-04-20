#include "MQTTWorker.h"

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



bool MQTTWorker::initialize(std::queue<uint8_t> * mqttForwardCommandQueue,
                            std::queue<float> * flexSPO2ForwardMQTTQueue,
                            std::queue<imuForceElement> * imuForceForwardMQTTQueue,
                            std::mutex * mqttForwardCommandMutex,
                            std::mutex * flexSPO2ForwardMQTTMutex,
                            std::mutex * imuForceForwardMQTTMutex) {
    this->mqttForwardCommandQueue = mqttForwardCommandQueue;
    this->flexSPO2ForwardMQTTQueue = flexSPO2ForwardMQTTQueue;
    this->imuForceForwardMQTTQueue = imuForceForwardMQTTQueue;
    this->mqttForwardCommandMutex = mqttForwardCommandMutex;
    this->flexSPO2ForwardMQTTMutex = flexSPO2ForwardMQTTMutex;
    this->imuForceForwardMQTTMutex = imuForceForwardMQTTMutex;
    
    // Set connect options
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
    // Checking connection attempt count
        // Because reconnect is automatic so we need to check once limit has been hit 

    
    // Check for publish fail flag 

    // Check subscription fail flag (really should only be set if reconnet occurs here)

    // Check for messages from the relevant queues

    // Attempt to publish them 

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
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_POINTER_MCP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_POINTER_PIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_POINTER_DIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_MIDDLE_MCP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_MIDDLE_PIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_MIDDLE_DIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_RING_MCP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_RING_PIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_RING_DIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_PINKY_MCP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_PINKY_PIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_PINKY_DIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_THUMB_MCP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_JOINT_THUMB_PIP, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_ABDUCTION_POINTER, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_ABDUCTION_MIDDLE, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_ABDUCTION_RING, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_ABDUCTION_PINKY, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_ABDUCTION_THUMB, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_ORIENTATION_WRIST_X, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_ORIENTATION_WRIST_Y, QOS);
    worker->client.subscribe(MQTTWorker::TOPIC_SPO2_WRIST, QOS);
}

void MQTTWorker::Callback::message_arrived(mqtt::const_message_ptr message){
    std::lock_guard commandGuard(*(worker->mqttForwardCommandMutex));
    std::string payload = message.get()->get_payload();

    // It depends on if the publisher sent it as "1" or the underlying ascii value is the numbe 
    // If "1", then do worker->mqttForwardCommandQueue->push(stoi(string));
    // If underlying, then do worker->mqttForwardCommandQueue->push(static_cast<uint8_t>(payload[0]));
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
