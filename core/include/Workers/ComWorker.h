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
    std::queue<dataToProcessorElement> * comForwardFlexSPO2Queue; 
    std::queue<dataToProcessorElement> * comForwardIMUForceQueue; 
    std::queue<uint8_t>  * comCommandForwardProcessingQueue;

    Communication * com;

    void convertToDataToProcessorElem(std::vector<dataToProcessorElement>& dataElements, std::vector<std::vector<uint8_t>> receivedMessages);
    void placeElementInCorrectQueue(dataToProcessorElement& elem);

    public:

        bool initialize();

        void run();


};



#endif

// comForwardFlexSPO2Queue
// comForwardIMUForceQueue