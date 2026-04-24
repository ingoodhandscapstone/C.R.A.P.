#ifndef SENSOR_PROCESSING_LANE_WORKER_H
#define SENSOR_PROCESSING_LANE_WORKER_H

#include <mutex>
#include <queue>
#include <stop_token>
#include <string>
#include <unordered_map>
#include <chrono>
#include <vector>

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

    struct ImuCalibrationState {
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        bool hasAccel;
        bool hasGyro;
        bool done;
    };

    std::vector<SensorID> activeFlexSensors;
    std::vector<SensorID> activeFingerImuSensors;
    std::vector<SensorID> activeForceSensors;
    bool activeSpo2Session;
    bool activeWristOrientationOutput;

    ImuCalibrationState handImuCalibrationState;
    ImuCalibrationState pointerImuCalibrationState;
    ImuCalibrationState middleImuCalibrationState;
    ImuCalibrationState thumbImuCalibrationState;
    ImuCalibrationState ringImuCalibrationState;
    ImuCalibrationState pinkyImuCalibrationState;

    bool pointerForceCalibrationDone;
    bool middleForceCalibrationDone;
    bool thumbForceCalibrationDone;
    bool ringForceCalibrationDone;
    bool pinkyForceCalibrationDone;

    bool handHasUpdatedInSession;

    static std::mutex calibrationEpochMutex;
    static uint32_t calibrationEpochCounter;
    static uint32_t calibrationEpochJoinCount;
    static uint32_t calibrationEpochRequiredCount;
    static bool calibrationEpochOpen;
    static const std::chrono::milliseconds CALIBRATION_TIMEOUT_MS;

    std::chrono::steady_clock::time_point calibrationStartTime;

    bool (SensorProcessingLaneWorker::*calibrationFunc)();
    void (SensorProcessingLaneWorker::*sessionFunc)();
    void (SensorProcessingLaneWorker::*resetFunc)();

    mutable std::mutex failureStateMutex;
    bool workerFailed;
    std::string failureReason;

    bool isSessionConfigCommand(SessionCommand command);
    bool isRelevantConfigCommand(SessionCommand command);
    bool usesFlexSpo2ForCommand(SessionCommand command);
    bool usesImuForceForCommand(SessionCommand command);
    uint32_t getRequiredCalibrationCount(SessionCommand command);
    uint32_t acquireCalibrationEpoch(uint32_t requiredCount);
    bool readFrontCommand(SessionCommand& command);
    void popFrontCommand();
    void clearSensorDataQueue();

    bool runCalibration();
    void runSession();
    void runReset();
    void resetProcessingData();



    bool setConfigFunctionFlexSpo2();
    bool setConfigFunctionImuForce();

    bool calibrateFlexSession();
    bool calibrateSpo2Session();
    bool calibrateImuSession();
    bool calibrateForceSession();

    void runFlexSession();
    void runSpo2Session();
    void runImuSession();
    void runForceSession();

    void configureFlexSession(SessionCommand command);
    void configureImuSession(SessionCommand command);

    JointRomProcessor * findFlexSensorProcessor(const SensorID& id);
    FingerAbductionProcessor * findFingerImuProcessor(const SensorID& id);
    ForceProcessing * findForceProcessor(const SensorID& id);
    ImuCalibrationState * findImuCalibrationState(const SensorID& id);

    bool isActiveFlexSensor(const SensorID& id);
    bool isActiveFingerImuSensor(const SensorID& id);
    bool isActiveForceSensor(const SensorID& id);
    bool isHandImuActive();
    bool areActiveFlexSensorsCalibrated();
    bool areActiveImuSensorsCalibrated();
    bool areActiveForceSensorsCalibrated();

    // convert to usable data by processors
    void convertToImuInputData(Eigen::Vector3d& accels, Eigen::Vector3d& gyro, const DataToProcessorElement& elem);
    void convertToFlexInputData(int& digVoltage, const DataToProcessorElement& elem);
    void convertToSpo2InputData(int& ir, int& red, const DataToProcessorElement& elem);


    void resetFlexSPO2Data();
    void resetImuForceData();
    void resetSessionSelections();
    void resetImuCalibrationState();
    void resetForceCalibrationState();
    void resetFlexSPO2ConfigData();
    void resetImuForceConfigData();
    void setFailure(const std::string& reason);
    void clearFailure();
    void logSensorElementRead(const char * stage, const DataToProcessorElement& elem);
    void logOutputElement(const DataOutputElement& elem);
    void logCalibrationInitialStates();

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
        activeFlexSensors(),
        activeFingerImuSensors(),
        activeForceSensors(),
        activeSpo2Session(false),
        activeWristOrientationOutput(false),
        handImuCalibrationState{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false},
        pointerImuCalibrationState{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false},
        middleImuCalibrationState{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false},
        thumbImuCalibrationState{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false},
        ringImuCalibrationState{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false},
        pinkyImuCalibrationState{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false},
        pointerForceCalibrationDone(false),
        middleForceCalibrationDone(false),
        thumbForceCalibrationDone(false),
        ringForceCalibrationDone(false),
        pinkyForceCalibrationDone(false),
        calibrationStartTime(std::chrono::steady_clock::time_point::min()),
        handHasUpdatedInSession(false),
        calibrationFunc(nullptr),
        sessionFunc(nullptr),
        resetFunc(nullptr),
        failureStateMutex(),
        workerFailed(false),
        failureReason() {}

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
    bool hasFailure();
    std::string getFailureReason();
};


#endif
