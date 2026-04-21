#include "ComWorker.h"

const int ComWorker::DATA_ENDPOINTS_COUNT = 5;

namespace {
constexpr size_t TYPE_INDEX = 0;
constexpr size_t ID_INDEX = 1;
constexpr size_t TIMESTAMP_INDEX = 2;
constexpr size_t DATA_STARTING_INDEX = 6;

uint32_t readUint32(const std::vector<uint8_t>& bytes, size_t startIndex) {
    return static_cast<uint32_t>(bytes[startIndex]) |
           (static_cast<uint32_t>(bytes[startIndex + 1]) << 8) |
           (static_cast<uint32_t>(bytes[startIndex + 2]) << 16) |
           (static_cast<uint32_t>(bytes[startIndex + 3]) << 24);
}
}

void ComWorker::convertToDataToProcessorElem(std::vector<DataToProcessorElement>& dataElements,
                                             std::vector<std::vector<uint8_t>> receivedMessages) {
    dataElements.clear();

    for (const auto& message : receivedMessages) {
        if (message.size() < DATA_STARTING_INDEX) {
            continue;
        }

        DataToProcessorElement elem;
        elem.id = static_cast<SensorID>(message[ID_INDEX]);
        elem.timestamp = readUint32(message, TIMESTAMP_INDEX);

        const SensorType type = static_cast<SensorType>(message[TYPE_INDEX]);

        switch (type) {
            case SensorType::FLEX:
                if (message.size() >= DATA_STARTING_INDEX + 4) {
                    elem.data.push_back(readUint32(message, DATA_STARTING_INDEX));
                }
                break;

            case SensorType::SPO2:
                if (message.size() >= DATA_STARTING_INDEX + 8) {
                    elem.data.push_back(readUint32(message, DATA_STARTING_INDEX));
                    elem.data.push_back(readUint32(message, DATA_STARTING_INDEX + 4));
                }
                break;

            case SensorType::IMU_ACCEL:
            case SensorType::IMU_GYRO:
                if (message.size() >= DATA_STARTING_INDEX + 12) {
                    elem.data.push_back(readUint32(message, DATA_STARTING_INDEX));
                    elem.data.push_back(readUint32(message, DATA_STARTING_INDEX + 4));
                    elem.data.push_back(readUint32(message, DATA_STARTING_INDEX + 8));
                }
                break;

            default:
                break;
        }

        if (!elem.data.empty()) {
            dataElements.push_back(elem);
        }
    }
}


void ComWorker::run(){
    while(true){

        // Lock mutex for mqttForwardCommandMutex
        // Check the queue for new message
        // if message, forward to comCommandForwardProcessingQueue and write to both devices?? (Can this somehow go wrong given the flow?)
        bool noMessage = true;
        std::vector<uint8_t> message;
        {
            std::lock_guard guard(*mqttForwardCommandMutex);
            if(!(noMessage = mqttForwardCommandQueue->empty())){
                message.push_back(mqttForwardCommandQueue->front());
                mqttForwardCommandQueue->pop();
            }

        }

        if(noMessage){
            continue;
        }

        {
            std::lock_guard guard(comCommandForwardProcessingMutex);
            comCommandForwardProcessingQueue->push(message.at(0));
        }

        // I can write to both as long as the session config for gripper is different than for glove (which it is)
        // This requires that the received values are identical in meaning to COMMANDS, WHICH THEY SHOULD BE
        // Requires the session config command though is recognized by all devices otherwise could cause isses
        com->write(Endpoints::COMMAND_CHAR_GRIPPER, message);
        com->write(Endpoints::COMMAND_CHAR_GLOVE, message);


        // com->read() from both devices for all of the different types of data
        // Alocate for each relevant possible endpoint
        std::vector<std::vector<uint8_t>> receivedMessages(DATA_ENDPOINTS_COUNT);
        // First 5 enums (0 - 4 equivalent) are relevant to read from
        for(int i = 0; i < 5; i++){
            com->read(static_cast<Endpoints>(i), receivedMessages.at(i));
        }

        // Convert to DataToProcessorElement 
        std::vector<DataToProcessorElement> dataElements;
        convertToDataToProcessorElem(dataElements, receivedMessages);
        // Push to correct queues based on SensorID
        {
            std::lock_guard guard1(comForwardFlexSPO2Mutex);
            std::lock_guard guard2(comForwardIMUForceMutex);

            for(int i = 0; i < dataElements.size(); i++){
                placeElementInCorrectQueue(dataElements.at(i));
            }
        }
        
    }
}

void ComWorker::placeElementInCorrectQueue(DataToProcessorElement& elem) {
    switch (elem.id) {
        case SensorID::WRIST_SPO2:
        case SensorID::POINTER_MCP_FLEX:
        case SensorID::POINTER_PIP_FLEX:
        case SensorID::POINTER_DIP_FLEX:
        case SensorID::MIDDLE_MCP_FLEX:
        case SensorID::MIDDLE_PIP_FLEX:
        case SensorID::MIDDLE_DIP_FLEX:
        case SensorID::RING_MCP_FLEX:
        case SensorID::RING_PIP_FLEX:
        case SensorID::RING_DIP_FLEX:
        case SensorID::PINKY_MCP_FLEX:
        case SensorID::PINKY_PIP_FLEX:
        case SensorID::PINKY_DIP_FLEX:
        case SensorID::THUMB_MCP_FLEX:
        case SensorID::THUMB_PIP_FLEX:
            comForwardFlexSPO2Queue->push(elem);
            break;

        case SensorID::HAND_IMU:
        case SensorID::POINTER_IMU:
        case SensorID::MIDDLE_IMU:
        case SensorID::THUMB_IMU:
        case SensorID::RING_IMU:
        case SensorID::PINKY_IMU:
            comForwardIMUForceQueue->push(elem);
            break;

        default:
            break;
    }
}
