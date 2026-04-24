#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <string>

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
#include "PahoMQTTClient.h"
#include "Logger.h"

namespace {
constexpr float FLEX_NOISE_FLOOR = 0.0f;
constexpr float FLEX_ADC_LSB_VOLTS = 0.001f; // 1 mV/LSB
constexpr float FLEX_DEADBAND = 0.0f; // drift
constexpr float FLEX_CALIBRATION_VOLTAGE = 0.0f;

ImuProcessingConfig makeImuConfig(double gyroProcessNoiseStd,
                                  double accelProcessNoiseStd,
                                  double accelBiasNoiseStd,
                                  double gyroBiasNoiseStd,
                                  double orientationVariance,
                                  double accelMeasurementStd) {
    ImuProcessingConfig config;
    config.gyroProcessNoise = Eigen::Vector3d::Constant(gyroProcessNoiseStd);
    config.accelProcessNoise = Eigen::Vector3d::Constant(accelProcessNoiseStd);
    config.accelsBiasNoise = Eigen::Vector3d::Constant(accelBiasNoiseStd);
    config.gyroBiasNoise = Eigen::Vector3d::Constant(gyroBiasNoiseStd);
    config.orthoCorrectionMat = Eigen::Matrix3d::Identity();
    config.orthoCorrectionBias = Eigen::Vector3d::Zero();
    config.orientationVariance = Eigen::Vector3d::Constant(orientationVariance);
    const double accelMeasurementVariance = accelMeasurementStd * accelMeasurementStd;
    config.accelMeasurementCovariance = Eigen::Matrix3d::Identity() * accelMeasurementVariance;
    return config;
}

ImuProcessingConfig makeHandImuConfig() {
    return makeImuConfig(
        0.02,  // gyro process noise std (rad/s)
        0.20,  // accel process noise std (m/s^2)
        0.01,  // accel bias noise std (m/s^2)
        0.001, // gyro bias noise std (rad/s)
        0.05,  // orientation variance
        0.35   // accel measurement std (m/s^2)
    );
}

ImuProcessingConfig makeFingerImuConfig() {
    return makeImuConfig(
        0.03,  // gyro process noise std (rad/s)
        0.25,  // accel process noise std (m/s^2)
        0.015, // accel bias noise std (m/s^2)
        0.0015,// gyro bias noise std (rad/s)
        0.07,  // orientation variance
        0.40   // accel measurement std (m/s^2)
    );
}

ResistiveSensorConfig makeFlexSensorConfig() {
    ResistiveSensorConfig config;
    config.noiseFloor = FLEX_NOISE_FLOOR;
    config.adcLSB = FLEX_ADC_LSB_VOLTS;
    config.deadband = FLEX_DEADBAND;
    config.calibratePositionVoltage = FLEX_CALIBRATION_VOLTAGE;

    // angle = -16.1*x^2 + 66*x + 18.4
    config.loadingPieceCoef = {-16.1f, 66.0f, 18.4f};
    config.unloadingPieceCoef = {-16.1f, 66.0f, 18.4f};
    return config;
}
}



ImuProcessingConfig handImuConfig = {
    makeHandImuConfig()
};

ImuProcessingConfig pointerImuConfig = {
    makeFingerImuConfig()
};

ImuProcessingConfig middleImuConfig = {
    makeFingerImuConfig()
};

ImuProcessingConfig thumbImuConfig = {
    makeFingerImuConfig()
};

ImuProcessingConfig ringImuConfig = {
    makeFingerImuConfig()
};

ImuProcessingConfig pinkyImuConfig = {
    makeFingerImuConfig()
};

ResistiveSensorConfig pointerMcpFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig pointerPipFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig pointerDipFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig middleMcpFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig middlePipFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig middleDipFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig ringMcpFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig ringPipFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig ringDipFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig pinkyMcpFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig pinkyPipFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig pinkyDipFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig thumbMcpFlexConfig = {
    makeFlexSensorConfig()
};

ResistiveSensorConfig thumbPipFlexConfig = {
    makeFlexSensorConfig()
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

    if(!bleCom.initialize()){
        Logger::instance().error("Main", "Bluetooth initialization failed.", true);
        return false;
    }

    if(!comWorker.initialize(
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
    )){
        Logger::instance().error("Main", "ComWorker initialization failed.", false);
        return false;
    }

    if(!mqttWorker.initialize(
        &mqttForwardCommandQueue,
        &flexSPO2ForwardMQTTQueue,
        &imuForceForwardMQTTQueue,
        &calibrationStatusQueue,
        &mqttForwardCommandMutex,
        &flexSPO2ForwardMQTTMutex,
        &imuForceForwardMQTTMutex,
        &calibrationStatusMutex
    )){
        std::string reason = mqttWorker.getFailureReason();
        if(reason.empty()){
            reason = "Unknown MQTT initialization failure.";
        }
        Logger::instance().error("Main", "MQTTWorker initialization failed: " + reason, true);
        return false;
    }

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

    Logger::instance().info("Main", "Initialization completed successfully.", false);
    return true;
}

bool done = false;
std::mutex threadFailedMutex;
std::condition_variable threadFailedCV;


void comWorkerThreadFunc(std::stop_token stopToken){
    comWorker.run(stopToken); // Only return if failed or stop requested

    if(!stopToken.stop_requested()){
        Logger::instance().error("Main",
                                 "ComWorker thread exited unexpectedly.",
                                 true);
    }

    std::lock_guard guard(threadFailedMutex);
    done = true;
    threadFailedCV.notify_one();

}


void mqttWorkerThreadFunc(std::stop_token stopToken){
    mqttWorker.run(stopToken); // Only return if failed or stop requested

    if(!stopToken.stop_requested()){
        std::string reason = mqttWorker.getFailureReason();
        if(reason.empty()){
            reason = "MQTTWorker exited unexpectedly without explicit failure reason.";
        }
        Logger::instance().error("Main",
                                 "MQTTWorker thread failed: " + reason,
                                 true);
    }

    std::lock_guard guard(threadFailedMutex);
    done = true;
    threadFailedCV.notify_one();
}

void flexSpo2WorkerThreadFunc(std::stop_token stopToken){
    flexSpo2Worker.run(stopToken); // Only return if failed or stop requested

    if(!stopToken.stop_requested()){
        std::string reason = flexSpo2Worker.getFailureReason();
        if(reason.empty()){
            reason = "Flex/SPO2 worker exited unexpectedly without explicit failure reason.";
        }
        Logger::instance().error("Main",
                                 "Flex/SPO2 worker thread failed: " + reason,
                                 true);
    }

    std::lock_guard guard(threadFailedMutex);
    done = true;
    threadFailedCV.notify_one();
}

void imuForceWorkerThreadFunc(std::stop_token stopToken){
    imuForceWorker.run(stopToken); // Only return if failed or stop requested

    if(!stopToken.stop_requested()){
        std::string reason = imuForceWorker.getFailureReason();
        if(reason.empty()){
            reason = "IMU/Force worker exited unexpectedly without explicit failure reason.";
        }
        Logger::instance().error("Main",
                                 "IMU/Force worker thread failed: " + reason,
                                 true);
    }

    std::lock_guard guard(threadFailedMutex);
    done = true;
    threadFailedCV.notify_one();
}

int main() {
    Logger::instance().initialize("logs");

    std::unique_lock<std::mutex> tfLock(threadFailedMutex);

    if(!initialize()){
        return 1;
    }

    std::jthread comThread(comWorkerThreadFunc);
    std::jthread mqttThread(mqttWorkerThreadFunc);
    std::jthread flexSpo2Thread(flexSpo2WorkerThreadFunc);
    std::jthread imuForceThread(imuForceWorkerThreadFunc);

    std::cout << "Press q then Enter to quit.\n";
    while(!threadFailedCV.wait_for(tfLock, std::chrono::milliseconds(100), [](){return done;})){
        if(std::cin.rdbuf()->in_avail() <= 0){
            continue;
        }

        std::string input;
        if(!std::getline(std::cin, input)){
            continue;
        }

        if(input == "q" || input == "Q"){
            done = true;
            threadFailedCV.notify_one();
        }
    }

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
