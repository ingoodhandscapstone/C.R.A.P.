#ifndef COM_WORKER_H
#define COM_WORKER_H

#include <mutex>
#include <queue>
#include <vector>

#include "Communication.h"
#include "QueueMessageTypes.h"


class ComWorker {

    std::mutex * mqttForwardCommandMutex;
    std::mutex * comForwardFlexSPO2Mutex;
    std::mutex * comForwardIMUForceMutex;

    std::queue<uint8_t> * mqttForwardCommandQueue;
    std::queue<dataToProcessorElement> * comForwardFlexSPO2Queue; 
    std::queue<dataToProcessorElement> * comForwardIMUForceQueue; 


    Communication * com;


    public:

        bool initialize();

        void run();


};



#endif

// comForwardFlexSPO2Queue
// comForwardIMUForceQueue