#ifndef SENSOR_PROCESSING_WORKER_H
#define SENSOR_PROCESSING_WORKER_H

#include <mutex>
#include <queue>
#include "QueueMessageTypes.h"
#include "SessionCommand.h"
#include "ProcessorConfigs.h"
#include "BloodOxygenProcessor.h"
#include "FingerAbductionProcessor.h"
#include "ForceProcessing.h"
#include "JointRomProcessor.h"
#include "WristOrientationProcessor.h"
#include <unordered_map>


// Im not even gonna do this as in multiple instances. Just two sensor processing functions
class SensorProcessingWorker {

    enum class SensorProcessingState{
        IDLE,
        CONFIGURED,
        CALIBRATING,
        WAITING_TO_RUN,
        RUNNING,
        STOP,
        RESETTING
    };


    // Sensor processors mapped from SensorID.
    BloodOxygenProcessor wristSPO2Processor;          // WRIST_SPO2

    WristOrientationProcessor handIMUProcessor;       // HAND_IMU
    FingerAbductionProcessor pointerIMUProcessor;     // POINTER_IMU
    FingerAbductionProcessor middleIMUProcessor;      // MIDDLE_IMU
    FingerAbductionProcessor thumbIMUProcessor;       // THUMB_IMU
    FingerAbductionProcessor ringIMUProcessor;        // RING_IMU
    FingerAbductionProcessor pinkyIMUProcessor;       // PINKY_IMU

    JointRomProcessor pointerMCPFlexProcessor;        // POINTER_MCP_FLEX
    JointRomProcessor pointerPIPFlexProcessor;        // POINTER_PIP_FLEX
    JointRomProcessor pointerDIPFlexProcessor;        // POINTER_DIP_FLEX
    JointRomProcessor middleMCPFlexProcessor;         // MIDDLE_MCP_FLEX
    JointRomProcessor middlePIPFlexProcessor;         // MIDDLE_PIP_FLEX
    JointRomProcessor middleDIPFlexProcessor;         // MIDDLE_DIP_FLEX
    JointRomProcessor ringMCPFlexProcessor;           // RING_MCP_FLEX
    JointRomProcessor ringPIPFlexProcessor;           // RING_PIP_FLEX
    JointRomProcessor ringDIPFlexProcessor;           // RING_DIP_FLEX
    JointRomProcessor pinkyMCPFlexProcessor;          // PINKY_MCP_FLEX
    JointRomProcessor pinkyPIPFlexProcessor;          // PINKY_PIP_FLEX
    JointRomProcessor pinkyDIPFlexProcessor;          // PINKY_DIP_FLEX
    JointRomProcessor thumbMCPFlexProcessor;          // THUMB_MCP_FLEX
    JointRomProcessor thumbPIPFlexProcessor;          // THUMB_PIP_FLEX

    ForceProcessing pointerForceProcessor;            // POINTER_FORCE
    ForceProcessing middleForceProcessor;             // MIDDLE_FORCE
    ForceProcessing thumbForceProcessor;              // THUMB_FORCE
    ForceProcessing ringForceProcessor;               // RING_FORCE
    ForceProcessing pinkyForceProcessor;              // PINKY_FORCE

    std::queue <DataOutputElement> * flexSPO2ForwardMQTTQueue;
    std::queue<DataOutputElement> * imuForceForwardMQTTQueue;
    std::queue<uint8_t>  * comCommandForwardProcessingQueue;

    std::mutex * flexSPO2ForwardMQTTMutex;
    std::mutex * imuForceForwardMQTTMutex;
    std::mutex * comCommandForwardProcessingMutex;
    std::mutex sharedMemberAccessMutex;

    std::unordered_map<SensorID, ImuProcessingConfig> *  imuConfigs;
    std::unordered_map<SensorID, ResistiveSensorConfig> * resistiveSensorConfigs;

    SessionCommand mostRecentConfigCommand;

    SensorProcessingState flexSPO2State;
    SensorProcessingState imuForceState;


    bool runSensorProcessFlexSPO2();
    bool runSensorProcessImuForce();
    

    public:

        void initialize(std::unordered_map<SensorID, ImuProcessingConfig> * ImuConfigs, std::unordered_map<SensorID, ResistiveSensorConfig> * resistiveSensorConfigs);

        void run();





};




#endif
