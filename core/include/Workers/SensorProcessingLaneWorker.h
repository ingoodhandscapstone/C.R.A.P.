#ifndef SENSOR_PROCESSING_LANE_WORKER_H
#define SENSOR_PROCESSING_LANE_WORKER_H

#include <mutex>
#include <queue>
#include <stop_token>
#include <unordered_map>

#include "ProcessorConfigs.h"
#ifndef PROCESSOR_CONFIGS_H
#define PROCESSOR_CONFIGS_H
#endif
#include "BloodOxygenProcessor.h"
#include "FingerAbductionProcessor.h"
#include "ForceProcessing.h"
#include "JointRomProcessor.h"
#include "QueueMessageTypes.h"
#include "SessionCommand.h"
#include "WristOrientationProcessor.h"
#include "Eigen/Core"


class SensorProcessingLaneWorker {
  public:
    enum class ProcessingGroup {
        FLEX_SPO2,
        IMU_FORCE
    };

  private:
    enum class SensorProcessingState {
        IDLE,
        CONFIGURED,
        CALIBRATING,
        WAITING_TO_RUN,
        RUNNING,
        RESETTING
    };

    ProcessingGroup processingGroup;

    // Sensor processors mapped from SensorID.
    BloodOxygenProcessor * wristSPO2Processor;          // WRIST_SPO2

    WristOrientationProcessor * handIMUProcessor;       // HAND_IMU
    FingerAbductionProcessor * pointerIMUProcessor;     // POINTER_IMU
    FingerAbductionProcessor * middleIMUProcessor;      // MIDDLE_IMU
    FingerAbductionProcessor * thumbIMUProcessor;       // THUMB_IMU
    FingerAbductionProcessor * ringIMUProcessor;        // RING_IMU
    FingerAbductionProcessor * pinkyIMUProcessor;       // PINKY_IMU

    JointRomProcessor * pointerMCPFlexProcessor;        // POINTER_MCP_FLEX
    JointRomProcessor * pointerPIPFlexProcessor;        // POINTER_PIP_FLEX
    JointRomProcessor * pointerDIPFlexProcessor;        // POINTER_DIP_FLEX
    JointRomProcessor * middleMCPFlexProcessor;         // MIDDLE_MCP_FLEX
    JointRomProcessor * middlePIPFlexProcessor;         // MIDDLE_PIP_FLEX
    JointRomProcessor * middleDIPFlexProcessor;         // MIDDLE_DIP_FLEX
    JointRomProcessor * ringMCPFlexProcessor;           // RING_MCP_FLEX
    JointRomProcessor * ringPIPFlexProcessor;           // RING_PIP_FLEX
    JointRomProcessor * ringDIPFlexProcessor;           // RING_DIP_FLEX
    JointRomProcessor * pinkyMCPFlexProcessor;          // PINKY_MCP_FLEX
    JointRomProcessor * pinkyPIPFlexProcessor;          // PINKY_PIP_FLEX
    JointRomProcessor * pinkyDIPFlexProcessor;          // PINKY_DIP_FLEX
    JointRomProcessor * thumbMCPFlexProcessor;          // THUMB_MCP_FLEX
    JointRomProcessor * thumbPIPFlexProcessor;          // THUMB_PIP_FLEX

    ForceProcessing * pointerForceProcessor;            // POINTER_FORCE
    ForceProcessing * middleForceProcessor;             // MIDDLE_FORCE
    ForceProcessing * thumbForceProcessor;              // THUMB_FORCE
    ForceProcessing * ringForceProcessor;               // RING_FORCE
    ForceProcessing * pinkyForceProcessor;              // PINKY_FORCE

    std::queue<DataOutputElement> * forwardMQTTQueue;
    std::queue<SessionCommand> * commandQueue;
    std::queue<DataToProcessorElement> * sensorDataQueue;
    std::queue<CalibrationStatusMessage> * calibrationStatusQueue;

    std::mutex * forwardMQTTMutex;
    std::mutex * commandMutex;
    std::mutex * sensorDataMutex;
    std::mutex * calibrationStatusMutex;

    std::unordered_map<SensorID, ImuProcessingConfig> * imuConfigs;
    std::unordered_map<SensorID, ResistiveSensorConfig> * resistiveSensorConfigs;

    SessionCommand mostRecentConfigCommand;
    SensorProcessingState state;
    uint32_t currentCalibrationEpoch;

    static std::mutex calibrationEpochMutex;
    static uint32_t calibrationEpochCounter;
    static uint32_t calibrationEpochJoinCount;
    static uint32_t calibrationEpochRequiredCount;
    static bool calibrationEpochOpen;

    bool (SensorProcessingLaneWorker::*calibrationFunc)();
    void (SensorProcessingLaneWorker::*sessionFunc)();

    bool isRelevantConfigCommand(SessionCommand command);
    uint32_t getRequiredCalibrationCount(SessionCommand command);
    uint32_t acquireCalibrationEpoch(uint32_t requiredCount);
    bool readFrontCommand(SessionCommand& command);
    void popFrontCommand();
    void clearSensorDataQueue();

    bool runCalibration();
    void runSession();
    void resetProcessingData();



    bool setConfigFunctionFlexSpo2();
    bool setConfigFunctionImuForce();

    // Session specific function to be used (flexSpo2 processor)
    bool pointerCalibrationFuncFlexSpo2();
    void pointerSessionFuncFlexSpo2();

    // Session specific function to be used (imuForce processor)
    bool pointerCalibrationFuncImuForce();
    void pointerSessionFuncImuForce();

    // Identify sensor for config
    JointRomProcessor * findPointerSensor(SensorID& id);
    bool isFlexSpo2PointerConfigCalibrated();

    // convert to usable data by processors
    void convertToImuInputData(Eigen::Vector3d& accels, Eigen::Vector3d& gyro,  DataToProcessorElement& elem);
    void convertToFlexInputData(int& digVoltage, DataToProcessorElement& elem);
    void convertToSpo2InputData(int& ir, int& red,  DataToProcessorElement& elem);


    void resetFlexSPO2Data();
    void resetImuForceData();

  public:
    SensorProcessingLaneWorker() :
        processingGroup(ProcessingGroup::FLEX_SPO2),
        wristSPO2Processor(nullptr),
        handIMUProcessor(nullptr),
        pointerIMUProcessor(nullptr),
        middleIMUProcessor(nullptr),
        thumbIMUProcessor(nullptr),
        ringIMUProcessor(nullptr),
        pinkyIMUProcessor(nullptr),
        pointerMCPFlexProcessor(nullptr),
        pointerPIPFlexProcessor(nullptr),
        pointerDIPFlexProcessor(nullptr),
        middleMCPFlexProcessor(nullptr),
        middlePIPFlexProcessor(nullptr),
        middleDIPFlexProcessor(nullptr),
        ringMCPFlexProcessor(nullptr),
        ringPIPFlexProcessor(nullptr),
        ringDIPFlexProcessor(nullptr),
        pinkyMCPFlexProcessor(nullptr),
        pinkyPIPFlexProcessor(nullptr),
        pinkyDIPFlexProcessor(nullptr),
        thumbMCPFlexProcessor(nullptr),
        thumbPIPFlexProcessor(nullptr),
        pointerForceProcessor(nullptr),
        middleForceProcessor(nullptr),
        thumbForceProcessor(nullptr),
        ringForceProcessor(nullptr),
        pinkyForceProcessor(nullptr),
        forwardMQTTQueue(nullptr),
        commandQueue(nullptr),
        sensorDataQueue(nullptr),
        calibrationStatusQueue(nullptr),
        forwardMQTTMutex(nullptr),
        commandMutex(nullptr),
        sensorDataMutex(nullptr),
        calibrationStatusMutex(nullptr),
        imuConfigs(nullptr),
        resistiveSensorConfigs(nullptr),
        mostRecentConfigCommand(SessionCommand::NONE),
        state(SensorProcessingState::IDLE),
        currentCalibrationEpoch(0),
        calibrationFunc(nullptr),
        sessionFunc(nullptr) {}

    void initialize(ProcessingGroup processingGroup,
                    BloodOxygenProcessor * wristSPO2Processor,
                    WristOrientationProcessor * handIMUProcessor,
                    FingerAbductionProcessor * pointerIMUProcessor,
                    FingerAbductionProcessor * middleIMUProcessor,
                    FingerAbductionProcessor * thumbIMUProcessor,
                    FingerAbductionProcessor * ringIMUProcessor,
                    FingerAbductionProcessor * pinkyIMUProcessor,
                    JointRomProcessor * pointerMCPFlexProcessor,
                    JointRomProcessor * pointerPIPFlexProcessor,
                    JointRomProcessor * pointerDIPFlexProcessor,
                    JointRomProcessor * middleMCPFlexProcessor,
                    JointRomProcessor * middlePIPFlexProcessor,
                    JointRomProcessor * middleDIPFlexProcessor,
                    JointRomProcessor * ringMCPFlexProcessor,
                    JointRomProcessor * ringPIPFlexProcessor,
                    JointRomProcessor * ringDIPFlexProcessor,
                    JointRomProcessor * pinkyMCPFlexProcessor,
                    JointRomProcessor * pinkyPIPFlexProcessor,
                    JointRomProcessor * pinkyDIPFlexProcessor,
                    JointRomProcessor * thumbMCPFlexProcessor,
                    JointRomProcessor * thumbPIPFlexProcessor,
                    ForceProcessing * pointerForceProcessor,
                    ForceProcessing * middleForceProcessor,
                    ForceProcessing * thumbForceProcessor,
                    ForceProcessing * ringForceProcessor,
                    ForceProcessing * pinkyForceProcessor,
                    std::unordered_map<SensorID, ImuProcessingConfig> * imuConfigs,
                    std::unordered_map<SensorID, ResistiveSensorConfig> * resistiveSensorConfigs,
                    std::queue<DataOutputElement> * forwardMQTTQueue,
                    std::queue<SessionCommand> * commandQueue,
                    std::queue<DataToProcessorElement> * sensorDataQueue,
                    std::queue<CalibrationStatusMessage> * calibrationStatusQueue,
                    std::mutex * forwardMQTTMutex,
                    std::mutex * commandMutex,
                    std::mutex * sensorDataMutex,
                    std::mutex * calibrationStatusMutex);

    void run(std::stop_token stopToken);
};


#endif
