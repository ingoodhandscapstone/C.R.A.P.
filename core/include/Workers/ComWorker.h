#ifndef COM_WORKER_H
#define COM_WORKER_H

#include <mutex>
#include <queue>
#include <vector>

#include "Communication.h"
#include "Endpoints.h"
#include "QueueMessageTypes.h"


class ComWorker {

    static const int DATA_ENDPOINTS_COUNT;

    std::mutex * mqttForwardCommandMutex;
    std::mutex * comForwardFlexSPO2Mutex;
    std::mutex * comForwardIMUForceMutex;
    std::mutex * comCommandForwardProcessingMutex;

    std::queue<uint8_t> * mqttForwardCommandQueue;
    std::queue<DataToProcessorElement> * comForwardFlexSPO2Queue; 
    std::queue<DataToProcessorElement> * comForwardIMUForceQueue; 
    std::queue<uint8_t>  * comCommandForwardProcessingQueue;

    Communication * com;

    void convertToDataToProcessorElem(std::vector<DataToProcessorElement>& dataElements, std::vector<std::vector<uint8_t>> receivedMessages);
    void placeElementInCorrectQueue(DataToProcessorElement& elem);

    public:

        bool initialize();

        void run();


};



#endif

// comForwardFlexSPO2Queue
// comForwardIMUForceQueue