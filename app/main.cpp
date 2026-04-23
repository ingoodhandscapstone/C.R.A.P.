#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <atomic>

#include "SensorProcessingLaneWorker.h"
#include "ComWorker.h"
#include "MQTTWorker.h"

#include "Communication.h"
#include "Bluetooth.h"
#include "BloodOxygenProcessor.h"
#include "FingerAbductionProcessor.h"
#include "ForceProcessing.h"
#include "JointRomProcessor.h"
#include "ProcessorConfigs.h"
#include "WristOrientationProcessor.h"
#include "QueueMessageTypes.h"
#include "SessionCommand.h"



ImuProcessingConfig handImuConfig = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero()
};

ImuProcessingConfig pointerImuConfig = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero()
};

ImuProcessingConfig middleImuConfig = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero()
};

ImuProcessingConfig thumbImuConfig = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero()
};

ImuProcessingConfig ringImuConfig = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero()
};

ImuProcessingConfig pinkyImuConfig = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Matrix3d::Zero()
};

ResistiveSensorConfig pointerMcpFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig pointerPipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig pointerDipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig middleMcpFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig middlePipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig middleDipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig ringMcpFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig ringPipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig ringDipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig pinkyMcpFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig pinkyPipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig pinkyDipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig thumbMcpFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig thumbPipFlexConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig pointerForceConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig middleForceConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig thumbForceConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig ringForceConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};

ResistiveSensorConfig pinkyForceConfig = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f}
};






std::unordered_map<SensorID, ImuProcessingConfig> imuConfigs; // Configs used for imus
std::unordered_map<SensorID, ResistiveSensorConfig> resistiveSensorConfigs; // Configs used for resistive sensors


std::queue<SessionCommand> mqttForwardCommandQueue; // MQTTWorker -> ComWorker (relaying command it received from frontend)

std::queue<SessionCommand> comCommandForwardProcessingFlexSPO2Queue; // ComWorker -> SensorProcessingLaneWorker (relaying session command data to instance)
std::queue<SessionCommand> comCommandForwardProcessingImuForceQueue; // ComWorker -> SensorProcessingLaneWorker (relaying session command data to instance)

std::queue<DataToProcessorElement> sensorDataProcessingFlexSPO2Queue; // ComWorker -> SensorProcessingLaneWorker (relaying sensor data to flexSpo2 processor)
std::queue<DataToProcessorElement> sensorDataProcessingImuForceQueue; // ComWorker -> SensorProcessingLaneWorker (relaying sensor data to imuForce processor)

std::queue<CalibrationStatusMessage> calibrationStatusQueue; // SensorProcessingLaneWorker -> MQTTWorker (relaying calibration status to the mqtt worker)

std::queue <DataOutputElement> flexSPO2ForwardMQTTQueue; // SensorProcessingLaneWorker -> MQTTWorker (relaying processed spo2Flex sensor data to MQTTWorker)
std::queue<DataOutputElement> imuForceForwardMQTTQueue; // SensorProcessingLaneWorker -> MQTTWorker (relaying processed imuForce sensor data to MQTTWorker)

std::mutex mqttForwardCommandMutex;
std::mutex sensorDataProcessingFlexSPO2Mutex;
std::mutex sensorDataProcessingImuForceMutex;
std::mutex comCommandForwardProcessingFlexSPO2Mutex;
std::mutex comCommandForwardProcessingImuForceMutex;
std::mutex calibrationStatusMutex;
std::mutex flexSPO2ForwardMQTTMutex;
std::mutex imuForceForwardMQTTMutex;

Bluetooth bleCom;
ComWorker comWorker;
MQTTWorker mqttWorker;
SensorProcessingLaneWorker flexSpo2Worker;
SensorProcessingLaneWorker imuForceWorker;

JointRomProcessor pointerMcpFlex;
JointRomProcessor pointerPipFlex;
JointRomProcessor pointerDipFlex;
JointRomProcessor middleMcpFlex;
JointRomProcessor middlePipFlex;
JointRomProcessor middleDipFlex;
JointRomProcessor ringMcpFlex;
JointRomProcessor ringPipFlex;
JointRomProcessor ringDipFlex;
JointRomProcessor pinkyMcpFlex;
JointRomProcessor pinkyPipFlex;
JointRomProcessor pinkyDipFlex;
JointRomProcessor thumbMcpFlex;
JointRomProcessor thumbPipFlex;

ForceProcessing pointerForce;
ForceProcessing middleForce;
ForceProcessing thumbForce;
ForceProcessing ringForce;
ForceProcessing pinkyForce;

FingerAbductionProcessor pointerAbduction;
FingerAbductionProcessor middleAbduction;
FingerAbductionProcessor thumbAbduction;
FingerAbductionProcessor ringAbduction;
FingerAbductionProcessor pinkyAbduction;

BloodOxygenProcessor bloodOx;
WristOrientationProcessor wristOrientation;




// mqttForwardCommandQueue
// comForwardFlexSPO2Queue
// comForwardIMUForceQueue
// flexSPO2ForwardMQTTQueue
// imuForceForwardMQTTQueue
// comCommandForwardProcessingQueue

// Mutex for each queue

// Bluetooth -> Com object
// Communication Process Object
// Sensor Process Objects 
// MQTT Process Object
// 14 Joint Rom Objects -> 14 Respective Session Config Objects
// 1 Wrist Orientation Object -> Session Config Object
// 5 Finger Abduction Object -> 5 Respective Session Config Objects
// 5 Force Processing Objects -> 5 Respective Session Config Objects
// SPO2 Processing Object

// Wire everything together
// Call "process.run()" in thread wrapping in main
// If a thead finishes then an error occurred (failed to initialize, connect, etc)
// Possibly notify UI or just exit program 






bool initialize(){

    bool success = true; 

    imuConfigs[SensorID::HAND_IMU] = handImuConfig;
    imuConfigs[SensorID::POINTER_IMU] = pointerImuConfig;
    imuConfigs[SensorID::MIDDLE_IMU] = middleImuConfig;
    imuConfigs[SensorID::THUMB_IMU] = thumbImuConfig;
    imuConfigs[SensorID::RING_IMU] = ringImuConfig;
    imuConfigs[SensorID::PINKY_IMU] = pinkyImuConfig;

    resistiveSensorConfigs[SensorID::POINTER_MCP_FLEX] = pointerMcpFlexConfig;
    resistiveSensorConfigs[SensorID::POINTER_PIP_FLEX] = pointerPipFlexConfig;
    resistiveSensorConfigs[SensorID::POINTER_DIP_FLEX] = pointerDipFlexConfig;
    resistiveSensorConfigs[SensorID::MIDDLE_MCP_FLEX] = middleMcpFlexConfig;
    resistiveSensorConfigs[SensorID::MIDDLE_PIP_FLEX] = middlePipFlexConfig;
    resistiveSensorConfigs[SensorID::MIDDLE_DIP_FLEX] = middleDipFlexConfig;
    resistiveSensorConfigs[SensorID::RING_MCP_FLEX] = ringMcpFlexConfig;
    resistiveSensorConfigs[SensorID::RING_PIP_FLEX] = ringPipFlexConfig;
    resistiveSensorConfigs[SensorID::RING_DIP_FLEX] = ringDipFlexConfig;
    resistiveSensorConfigs[SensorID::PINKY_MCP_FLEX] = pinkyMcpFlexConfig;
    resistiveSensorConfigs[SensorID::PINKY_PIP_FLEX] = pinkyPipFlexConfig;
    resistiveSensorConfigs[SensorID::PINKY_DIP_FLEX] = pinkyDipFlexConfig;
    resistiveSensorConfigs[SensorID::THUMB_MCP_FLEX] = thumbMcpFlexConfig;
    resistiveSensorConfigs[SensorID::THUMB_PIP_FLEX] = thumbPipFlexConfig;
    resistiveSensorConfigs[SensorID::POINTER_FORCE] = pointerForceConfig;
    resistiveSensorConfigs[SensorID::MIDDLE_FORCE] = middleForceConfig;
    resistiveSensorConfigs[SensorID::THUMB_FORCE] = thumbForceConfig;
    resistiveSensorConfigs[SensorID::RING_FORCE] = ringForceConfig;
    resistiveSensorConfigs[SensorID::PINKY_FORCE] = pinkyForceConfig;

    success = success && bleCom.initialize();
    success = success && comWorker.initialize(
        &mqttForwardCommandQueue,
        &sensorDataProcessingFlexSPO2Queue,
        &sensorDataProcessingImuForceQueue,
        &comCommandForwardProcessingFlexSPO2Queue,
        &comCommandForwardProcessingImuForceQueue,
        &mqttForwardCommandMutex,
        &sensorDataProcessingFlexSPO2Mutex,
        &sensorDataProcessingImuForceMutex,
        &comCommandForwardProcessingFlexSPO2Mutex,
        &comCommandForwardProcessingImuForceMutex,
        &bleCom
    );

    success = success && mqttWorker.initialize(
        &mqttForwardCommandQueue,
        &flexSPO2ForwardMQTTQueue,
        &imuForceForwardMQTTQueue,
        &calibrationStatusQueue,
        &mqttForwardCommandMutex,
        &flexSPO2ForwardMQTTMutex,
        &imuForceForwardMQTTMutex,
        &calibrationStatusMutex
    );

    flexSpo2Worker.initialize(
        SensorProcessingLaneWorker::ProcessingGroup::FLEX_SPO2,
        &bloodOx,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &pointerMcpFlex,
        &pointerPipFlex,
        &pointerDipFlex,
        &middleMcpFlex,
        &middlePipFlex,
        &middleDipFlex,
        &ringMcpFlex,
        &ringPipFlex,
        &ringDipFlex,
        &pinkyMcpFlex,
        &pinkyPipFlex,
        &pinkyDipFlex,
        &thumbMcpFlex,
        &thumbPipFlex,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &imuConfigs,
        &resistiveSensorConfigs,
        &flexSPO2ForwardMQTTQueue,
        &comCommandForwardProcessingFlexSPO2Queue,
        &sensorDataProcessingFlexSPO2Queue,
        &calibrationStatusQueue,
        &flexSPO2ForwardMQTTMutex,
        &comCommandForwardProcessingFlexSPO2Mutex,
        &sensorDataProcessingFlexSPO2Mutex,
        &calibrationStatusMutex
    );

    imuForceWorker.initialize(
        SensorProcessingLaneWorker::ProcessingGroup::IMU_FORCE,
        nullptr,
        &wristOrientation,
        &pointerAbduction,
        &middleAbduction,
        &thumbAbduction,
        &ringAbduction,
        &pinkyAbduction,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &pointerForce,
        &middleForce,
        &thumbForce,
        &ringForce,
        &pinkyForce,
        &imuConfigs,
        &resistiveSensorConfigs,
        &imuForceForwardMQTTQueue,
        &comCommandForwardProcessingImuForceQueue,
        &sensorDataProcessingImuForceQueue,
        &calibrationStatusQueue,
        &imuForceForwardMQTTMutex,
        &comCommandForwardProcessingImuForceMutex,
        &sensorDataProcessingImuForceMutex,
        &calibrationStatusMutex
    );

    return success;
}

bool done = false;
std::mutex threadFailedMutex;
std::condition_variable threadFailedCV;


void comWorkerThreadFunc(std::stop_token stopToken){
    comWorker.run(stopToken); // Only return if failed or stop requested

    std::lock_guard guard(threadFailedMutex);
    done = true;
    threadFailedCV.notify_one();

}


void mqttWorkerThreadFunc(std::stop_token stopToken){
    mqttWorker.run(stopToken); // Only return if failed or stop requested

    std::lock_guard guard(threadFailedMutex);
    done = true;
    threadFailedCV.notify_one();
}

void flexSpo2WorkerThreadFunc(std::stop_token stopToken){
    flexSpo2Worker.run(stopToken); // Only return if failed or stop requested

    std::lock_guard guard(threadFailedMutex);
    done = true;
    threadFailedCV.notify_one();
}

void imuForceWorkerThreadFunc(std::stop_token stopToken){
    imuForceWorker.run(stopToken); // Only return if failed or stop requested

    std::lock_guard guard(threadFailedMutex);
    done = true;
    threadFailedCV.notify_one();
}

int main() {

    std::unique_lock<std::mutex> tfLock(threadFailedMutex);

    if(!initialize()){
        return 0;
    }

    std::jthread comThread(comWorkerThreadFunc);
    std::jthread mqttThread(mqttWorkerThreadFunc);
    std::jthread flexSpo2Thread(flexSpo2WorkerThreadFunc);
    std::jthread imuForceThread(imuForceWorkerThreadFunc);

    threadFailedCV.wait(tfLock, [&](){return done;});

    comThread.request_stop();
    mqttThread.request_stop();
    flexSpo2Thread.request_stop();
    imuForceThread.request_stop();

    return 0;
}


// 2. Look at BLE and MQTT depedency to get good enough understanding of needs
// 3. Look into std::threading and what the queue needs to look like
// 4. Outline each of the classes in header files (given now what I know about ekf class)
// 5. Outline each of the threads


// Process 1 - Communication
    // Communication object with read, write, isConnected -> list of bytes
    // Underlying BLE implementation
        // Receives information through notification from server
        // Writes to command char
    // If disconnected, always attempt to reconnect

    // Once data is read it should be deserialized into particular struct format for use
    // Placed into either one of two queues (depending on the sensor id and groupping of sensor processing associated with that queue)

    // Has queue from MQTT communication process. Currently, commands are in same (1 byte) format with same universal meaning per code across devices
    // We basically have session config, start, stop, calibration start, and calibration stop
    // Communication will forward all commands to the esp32 which enforces state machine logic to maintain expected program flow 
    // Calibration start will be forwaded to both sensor processing processes


// Process 2 & 3 - Sensor Processing 1 & 2
    // Sensor Processing 1 - Responsible for 6 IMUs (wrist orientation, finger abduction) and 5 force sensors (grip strength)
    // Sensor Processing 2 - Responsible for 14 Flex Sensors (rom at each finger joint) and 1 oximeter (SPO2)

    // Each of these processing units are geared towards a specific measurement (ex PINKY_MCP, Wrist Orientation, POINTER_ABDUCTION)
    // Notably this means each unit is not limited to access to a single sensor (in this case only finger abduction I believe needs 2 IMUs)

    // Each will have access to configuration object setting the necessary coefficients for its processing routine allowing customizability between units

    // Each of the sensor processes will contain a state machine enforcing particular action of the processors based on the state
        // This is how it will know during a calibration state to take data from queue from communication process and particularly gather calibration constants
        // On the start of a new session (WHICH IS REALLY SESSION CONFIG COMMAND) - Clear/reset members, coefficients, stored data, etc
        // On start calibration - It will interpret incoming data for the purpose of setting initial parameters (not output to UI)
        // On start session - It will interpret incoming data for the purpose of sending to MQTT process
    // The actual session config (what sensors will be used) are unnecessary for this to know because it is simply reacting to incoming data addressed to it
    // If no data sent is addressed to it, it does nothing

    // Each process will have its own output queue to the MQTT Process

// Process 4 - MQTT Process 

    // This will read from each of the Sensor Processing Queues and publish to the corresponding topic
    // Each specific measurement type has its own topic that the MQTT will publish to accordingly
    
    // The MQTT process will read from a command topic where the UI sends it topics. 
    // From here it should be able to directly send it to the communication process through a queue
        // Shouldnt be any conversion necessary (single byte, universal meaning of codes)
    
    // Literally just forwards commands to glove and gripper 
