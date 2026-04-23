#include "ComWorker.h"

const int ComWorker::DATA_ENDPOINTS_COUNT = 5;

namespace {
constexpr size_t TYPE_INDEX = 0;
constexpr size_t ID_INDEX = 1;
constexpr size_t TIMESTAMP_INDEX = 2;
constexpr size_t DATA_STARTING_INDEX = 6;

uint32_t readUint32(const std::vector<uint8_t>& bytes, size_t startIndex) {
    return static_cast<uint32_t>(bytes[startIndex + 3]) |
           (static_cast<uint32_t>(bytes[startIndex + 2]) << 8) |
           (static_cast<uint32_t>(bytes[startIndex + 1]) << 16) |
           (static_cast<uint32_t>(bytes[startIndex]) << 24);
}
}

bool ComWorker::initialize(std::queue<SessionCommand> * mqttForwardCommandQueue,
                           std::queue<DataToProcessorElement> * sensorDataProcessingFlexSPO2Queue,
                           std::queue<DataToProcessorElement> * sensorDataProcessingImuForceQueue,
                           std::queue<SessionCommand>  * comCommandForwardProcessingFlexSPO2Queue,
                           std::queue<SessionCommand>  * comCommandForwardProcessingImuForceQueue,
                           std::mutex * mqttForwardCommandMutex,
                           std::mutex * sensorDataProcessingFlexSPO2Mutex,
                           std::mutex * sensorDataProcessingImuForceMutex,
                           std::mutex * comCommandForwardProcessingFlexSPO2Mutex,
                           std::mutex * comCommandForwardProcessingImuForceMutex,
                           Communication * com) {
    this->mqttForwardCommandQueue = mqttForwardCommandQueue;
    this->sensorDataProcessingFlexSPO2Queue = sensorDataProcessingFlexSPO2Queue;
    this->sensorDataProcessingImuForceQueue = sensorDataProcessingImuForceQueue;
    this->comCommandForwardProcessingFlexSPO2Queue = comCommandForwardProcessingFlexSPO2Queue;
    this->comCommandForwardProcessingImuForceQueue = comCommandForwardProcessingImuForceQueue;

    this->mqttForwardCommandMutex = mqttForwardCommandMutex;
    this->sensorDataProcessingFlexSPO2Mutex = sensorDataProcessingFlexSPO2Mutex;
    this->sensorDataProcessingImuForceMutex = sensorDataProcessingImuForceMutex;
    this->comCommandForwardProcessingFlexSPO2Mutex = comCommandForwardProcessingFlexSPO2Mutex;
    this->comCommandForwardProcessingImuForceMutex = comCommandForwardProcessingImuForceMutex;

    this->com = com;


    return true;
}

void ComWorker::convertToDataToProcessorElem(std::vector<DataToProcessorElement>& dataElements,
                                             std::vector<std::vector<uint8_t>> receivedMessages) {
    dataElements.clear();

    for (const auto& message : receivedMessages) {
        if (message.size() < DATA_STARTING_INDEX) {
            continue;
        }

        const SensorType type = static_cast<SensorType>(message[TYPE_INDEX]);
        DataToProcessorElement elem;
        elem.type = type;
        elem.id = static_cast<SensorID>(message[ID_INDEX]);
        elem.timestamp = readUint32(message, TIMESTAMP_INDEX);

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
            case SensorType::FORCE:
                if (message.size() >= DATA_STARTING_INDEX + 4) {
                    elem.data.push_back(readUint32(message, DATA_STARTING_INDEX));
                }

            default:
                break;
        }

        if (!elem.data.empty()) {
            dataElements.push_back(elem);
        }
    }
}


void ComWorker::run(std::stop_token stopToken){
    while(!stopToken.stop_requested()){

        // Lock mutex for mqttForwardCommandMutex
        // Check the queue for new message
        // if message, forward to both command-forward-processing queues and write to both devices?? (Can this somehow go wrong given the flow?)
        bool noMessage = true;
        SessionCommand command = SessionCommand::NONE;
        {
            std::lock_guard guard(*mqttForwardCommandMutex);
            if(!(noMessage = mqttForwardCommandQueue->empty())){
                command = mqttForwardCommandQueue->front();
                mqttForwardCommandQueue->pop();
            }

        }

        if(!noMessage){
            {
            std::lock_guard guardFlex(*comCommandForwardProcessingFlexSPO2Mutex);
            std::lock_guard guardSpo2(*comCommandForwardProcessingImuForceMutex);

            comCommandForwardProcessingFlexSPO2Queue->push(command);
            comCommandForwardProcessingImuForceQueue->push(command);

            }

            std::vector<uint8_t> message{static_cast<uint8_t>(command)};

            // I can write to both as long as the session config for gripper is different than for glove (which it is)
            // This requires that the received values are identical in meaning to COMMANDS, WHICH THEY SHOULD BE
            // Requires the session config command though is recognized by all devices otherwise could cause isses
            com->write(Endpoints::COMMAND_CHAR_GRIPPER, message);
            com->write(Endpoints::COMMAND_CHAR_GLOVE, message);
        }


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
            std::lock_guard guard1(*sensorDataProcessingFlexSPO2Mutex);
            std::lock_guard guard2(*sensorDataProcessingImuForceMutex);

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
            sensorDataProcessingFlexSPO2Queue->push(elem);
            break;

        case SensorID::HAND_IMU:
        case SensorID::POINTER_IMU:
        case SensorID::MIDDLE_IMU:
        case SensorID::THUMB_IMU:
        case SensorID::RING_IMU:
        case SensorID::PINKY_IMU:
        case SensorID::POINTER_FORCE:
        case SensorID::MIDDLE_FORCE:
        case SensorID::RING_FORCE:
        case SensorID::THUMB_FORCE:
        case SensorID::PINKY_FORCE:
            sensorDataProcessingImuForceQueue->push(elem);
            break;

        default:
            break;
    }
}
