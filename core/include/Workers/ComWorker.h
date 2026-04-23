#ifndef COM_WORKER_H
#define COM_WORKER_H

#include <mutex>
#include <queue>
#include <stop_token>
#include <vector>

#include "Communication.h"
#include "Endpoints.h"
#include "QueueMessageTypes.h"
#include "SessionCommand.h"


class ComWorker {

    static const int DATA_ENDPOINTS_COUNT;

    std::mutex * mqttForwardCommandMutex;
    std::mutex * sensorDataProcessingFlexSPO2Mutex;
    std::mutex * sensorDataProcessingImuForceMutex;
    std::mutex * comCommandForwardProcessingFlexSPO2Mutex;
    std::mutex * comCommandForwardProcessingImuForceMutex;


    std::queue<SessionCommand> * mqttForwardCommandQueue;
    std::queue<DataToProcessorElement> * sensorDataProcessingFlexSPO2Queue; 
    std::queue<DataToProcessorElement> * sensorDataProcessingImuForceQueue; 
    std::queue<SessionCommand>  * comCommandForwardProcessingFlexSPO2Queue;
    std::queue<SessionCommand>  * comCommandForwardProcessingImuForceQueue;


    Communication * com;

    void convertToDataToProcessorElem(std::vector<DataToProcessorElement>& dataElements, std::vector<std::vector<uint8_t>> receivedMessages);
    void placeElementInCorrectQueue(DataToProcessorElement& elem);

    public:
        ComWorker() :
            mqttForwardCommandMutex(nullptr),
            sensorDataProcessingFlexSPO2Mutex(nullptr),
            sensorDataProcessingImuForceMutex(nullptr),
            comCommandForwardProcessingFlexSPO2Mutex(nullptr),
            comCommandForwardProcessingImuForceMutex(nullptr),
            mqttForwardCommandQueue(nullptr),
            sensorDataProcessingFlexSPO2Queue(nullptr),
            sensorDataProcessingImuForceQueue(nullptr),
            comCommandForwardProcessingFlexSPO2Queue(nullptr),
            comCommandForwardProcessingImuForceQueue(nullptr),
            com(nullptr) {}

        bool initialize(std::queue<SessionCommand> * mqttForwardCommandQueue,
                        std::queue<DataToProcessorElement> * sensorDataProcessingFlexSPO2Queue,
                        std::queue<DataToProcessorElement> * sensorDataProcessingImuForceQueue,
                        std::queue<SessionCommand>  * comCommandForwardProcessingFlexSPO2Queue,
                        std::queue<SessionCommand>  * comCommandForwardProcessingImuForceQueue,
                        std::mutex * mqttForwardCommandMutex,
                        std::mutex * sensorDataProcessingFlexSPO2Mutex,
                        std::mutex * sensorDataProcessingImuForceMutex,
                        std::mutex * comCommandForwardProcessingFlexSPO2Mutex,
                        std::mutex * comCommandForwardProcessingImuForceMutex,
                        Communication * com);

        void run(std::stop_token stopToken);


};



#endif

// comForwardFlexSPO2Queue
// comForwardIMUForceQueue
