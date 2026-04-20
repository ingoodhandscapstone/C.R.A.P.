#ifndef COM_WORKER_H
#define COM_WORKER_H

#include <mutex>
#include <queue>
#include "Communication.h"


class ComWorker {

    std::mutex * mqttForwardCommandMutex;
    std::mutex * comForwardFlexSPO2Mutex;
    std::mutex * comForwardIMUForceMutex;

    std::queue<uint8_t> * mqttForwardCommandQueue;

    



    void deserialize();

    public:

        bool initialize();

        void run();


};



#endif

// comForwardFlexSPO2Queue
// comForwardIMUForceQueue