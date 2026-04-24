#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <bit>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "ComWorker.h"
#include "Communication.h"
#include "Endpoints.h"
#include "MQTTClient.h"
#include "QueueMessageTypes.h"
#include "SensorProcessingLaneWorker.h"
#include "SessionCommand.h"

namespace test_support {

inline ImuProcessingConfig makeImuConfig() {
    ImuProcessingConfig config;
    config.gyroProcessNoise = Eigen::Vector3d(1e-3, 1e-3, 1e-3);
    config.accelProcessNoise = Eigen::Vector3d(1e-2, 1e-2, 1e-2);
    config.accelsBiasNoise = Eigen::Vector3d(1e-5, 1e-5, 1e-5);
    config.gyroBiasNoise = Eigen::Vector3d(1e-5, 1e-5, 1e-5);
    config.orthoCorrectionMat = Eigen::Matrix3d::Identity();
    config.orthoCorrectionBias = Eigen::Vector3d::Zero();
    config.orientationVariance = Eigen::Vector3d(1e-2, 1e-2, 1e-2);
    config.accelMeasurementCovariance = Eigen::Matrix3d::Identity() * 1e-2;
    return config;
}

inline ResistiveSensorConfig makeResistiveConfig() {
    ResistiveSensorConfig config;
    config.noiseFloor = -1.0f;
    config.adcLSB = 1.0f;
    config.deadband = -1.0f;
    config.calibratePositionVoltage = 0.0f;
    config.loadingPieceCoef = {1.0f, 0.0f, 0.0f};
    config.unloadingPieceCoef = {1.0f, 0.0f, 0.0f};
    return config;
}

inline uint32_t floatToU32(float value) {
    return std::bit_cast<uint32_t>(value);
}

inline void appendUint32BE(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(value & 0xFF));
}

inline std::vector<uint8_t> serializeSensorMessage(SensorType type,
                                                   SensorID id,
                                                   uint32_t timestamp,
                                                   const std::vector<uint32_t>& payload) {
    std::vector<uint8_t> bytes;
    bytes.reserve(static_cast<size_t>(6 + payload.size() * 4));
    bytes.push_back(static_cast<uint8_t>(type));
    bytes.push_back(static_cast<uint8_t>(id));
    appendUint32BE(bytes, timestamp);
    for (const uint32_t value : payload) {
        appendUint32BE(bytes, value);
    }
    return bytes;
}

inline DataToProcessorElement makeFlexElem(SensorID id, uint32_t timestamp, uint32_t value) {
    DataToProcessorElement elem;
    elem.type = SensorType::FLEX;
    elem.id = id;
    elem.timestamp = timestamp;
    elem.data = {value};
    return elem;
}

inline DataToProcessorElement makeForceElem(SensorID id, uint32_t timestamp, uint32_t value) {
    DataToProcessorElement elem;
    elem.type = SensorType::FORCE;
    elem.id = id;
    elem.timestamp = timestamp;
    elem.data = {value};
    return elem;
}

inline DataToProcessorElement makeSpo2Elem(uint32_t timestamp, uint32_t ir, uint32_t red) {
    DataToProcessorElement elem;
    elem.type = SensorType::SPO2;
    elem.id = SensorID::WRIST_SPO2;
    elem.timestamp = timestamp;
    elem.data = {ir, red};
    return elem;
}

inline DataToProcessorElement makeImuElem(SensorType type,
                                          SensorID id,
                                          uint32_t timestamp,
                                          float x,
                                          float y,
                                          float z) {
    DataToProcessorElement elem;
    elem.type = type;
    elem.id = id;
    elem.timestamp = timestamp;
    elem.data = {floatToU32(x), floatToU32(y), floatToU32(z)};
    return elem;
}

template <typename Predicate>
inline bool waitUntil(Predicate&& predicate, int timeoutMs = 500) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return predicate();
}

template <typename T>
inline size_t lockedQueueSize(std::mutex& mutex, std::queue<T>& queue) {
    std::lock_guard<std::mutex> lock(mutex);
    return queue.size();
}

template <typename T>
inline void pushLocked(std::mutex& mutex, std::queue<T>& queue, const T& value) {
    std::lock_guard<std::mutex> lock(mutex);
    queue.push(value);
}

template <typename T>
inline bool popLocked(std::mutex& mutex, std::queue<T>& queue, T& out) {
    std::lock_guard<std::mutex> lock(mutex);
    if (queue.empty()) {
        return false;
    }
    out = queue.front();
    queue.pop();
    return true;
}

template <typename T>
inline void clearLocked(std::mutex& mutex, std::queue<T>& queue) {
    std::lock_guard<std::mutex> lock(mutex);
    while (!queue.empty()) {
        queue.pop();
    }
}

class MockCommunication : public Communication {
  public:
    MOCK_METHOD(bool, read, (const Endpoints& endpoint, std::vector<uint8_t>& message), (override));
    MOCK_METHOD(bool, write, (const Endpoints& endpoint, std::vector<uint8_t>& message), (override));
    MOCK_METHOD(bool, isConnected, (), (override));
};

class MockMQTTClient : public MQTTClient {
  public:
    MOCK_METHOD(bool, connect, (), (override));
    MOCK_METHOD(bool, subscribe, (const std::string& topic, int qos), (override));
    MOCK_METHOD(bool, publish, (const std::string& topic, const std::string& payload, int qos), (override));
    MOCK_METHOD(bool, isConnected, (), (override));
    MOCK_METHOD(void, setMessageHandler, (MessageHandler handler), (override));
};

struct WorkerRig {
    std::queue<DataOutputElement> forwardQueue;
    std::queue<SessionCommand> commandQueue;
    std::queue<DataToProcessorElement> sensorDataQueue;
    std::queue<CalibrationStatusMessage> calibrationStatusQueue;

    std::mutex forwardMutex;
    std::mutex commandMutex;
    std::mutex sensorDataMutex;
    std::mutex calibrationStatusMutex;

    std::unordered_map<SensorID, ImuProcessingConfig> imuConfigs;
    std::unordered_map<SensorID, ResistiveSensorConfig> resistiveConfigs;

    BloodOxygenProcessor spo2Processor;
    WristOrientationProcessor wristProcessor;
    FingerAbductionProcessor pointerImu;
    FingerAbductionProcessor middleImu;
    FingerAbductionProcessor thumbImu;
    FingerAbductionProcessor ringImu;
    FingerAbductionProcessor pinkyImu;

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

    SensorProcessingLaneWorker worker;

    WorkerRig() {
        const ImuProcessingConfig imuConfig = makeImuConfig();
        imuConfigs[SensorID::HAND_IMU] = imuConfig;
        imuConfigs[SensorID::POINTER_IMU] = imuConfig;
        imuConfigs[SensorID::MIDDLE_IMU] = imuConfig;
        imuConfigs[SensorID::THUMB_IMU] = imuConfig;
        imuConfigs[SensorID::RING_IMU] = imuConfig;
        imuConfigs[SensorID::PINKY_IMU] = imuConfig;

        const ResistiveSensorConfig resistiveConfig = makeResistiveConfig();
        resistiveConfigs[SensorID::POINTER_MCP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::POINTER_PIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::POINTER_DIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::MIDDLE_MCP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::MIDDLE_PIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::MIDDLE_DIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::RING_MCP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::RING_PIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::RING_DIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::PINKY_MCP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::PINKY_PIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::PINKY_DIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::THUMB_MCP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::THUMB_PIP_FLEX] = resistiveConfig;
        resistiveConfigs[SensorID::POINTER_FORCE] = resistiveConfig;
        resistiveConfigs[SensorID::MIDDLE_FORCE] = resistiveConfig;
        resistiveConfigs[SensorID::THUMB_FORCE] = resistiveConfig;
        resistiveConfigs[SensorID::RING_FORCE] = resistiveConfig;
        resistiveConfigs[SensorID::PINKY_FORCE] = resistiveConfig;
    }

    void initializeFlexLane(std::queue<CalibrationStatusMessage>* sharedCalibrationQueue = nullptr,
                            std::mutex* sharedCalibrationMutex = nullptr) {
        std::queue<CalibrationStatusMessage>* calibrationQueuePtr =
            (sharedCalibrationQueue != nullptr) ? sharedCalibrationQueue : &calibrationStatusQueue;
        std::mutex* calibrationMutexPtr =
            (sharedCalibrationMutex != nullptr) ? sharedCalibrationMutex : &calibrationStatusMutex;

        worker.initialize(
            SensorProcessingLaneWorker::ProcessingGroup::FLEX_SPO2,
            &spo2Processor,
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
            &resistiveConfigs,
            &forwardQueue,
            &commandQueue,
            &sensorDataQueue,
            calibrationQueuePtr,
            &forwardMutex,
            &commandMutex,
            &sensorDataMutex,
            calibrationMutexPtr);
    }

    void initializeImuForceLane(std::queue<CalibrationStatusMessage>* sharedCalibrationQueue = nullptr,
                                std::mutex* sharedCalibrationMutex = nullptr) {
        std::queue<CalibrationStatusMessage>* calibrationQueuePtr =
            (sharedCalibrationQueue != nullptr) ? sharedCalibrationQueue : &calibrationStatusQueue;
        std::mutex* calibrationMutexPtr =
            (sharedCalibrationMutex != nullptr) ? sharedCalibrationMutex : &calibrationStatusMutex;

        worker.initialize(
            SensorProcessingLaneWorker::ProcessingGroup::IMU_FORCE,
            nullptr,
            &wristProcessor,
            &pointerImu,
            &middleImu,
            &thumbImu,
            &ringImu,
            &pinkyImu,
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
            &resistiveConfigs,
            &forwardQueue,
            &commandQueue,
            &sensorDataQueue,
            calibrationQueuePtr,
            &forwardMutex,
            &commandMutex,
            &sensorDataMutex,
            calibrationMutexPtr);
    }

    void pushCommand(SessionCommand command) {
        pushLocked(commandMutex, commandQueue, command);
    }

    void pushSensor(const DataToProcessorElement& elem) {
        pushLocked(sensorDataMutex, sensorDataQueue, elem);
    }

    size_t forwardSize() {
        return lockedQueueSize(forwardMutex, forwardQueue);
    }

    size_t calibrationStatusSize() {
        return lockedQueueSize(calibrationStatusMutex, calibrationStatusQueue);
    }

    bool popForward(DataOutputElement& out) {
        return popLocked(forwardMutex, forwardQueue, out);
    }

    bool popCalibrationStatus(CalibrationStatusMessage& out) {
        return popLocked(calibrationStatusMutex, calibrationStatusQueue, out);
    }
};

inline void enqueueImuCalibrationSamples(WorkerRig& rig,
                                         SensorID id,
                                         int sampleCount,
                                         uint32_t startTimestamp) {
    uint32_t ts = startTimestamp;
    for (int i = 0; i < sampleCount; i++) {
        rig.pushSensor(makeImuElem(SensorType::IMU_ACCEL, id, ts++, 0.0f, 0.0f, -9.81f));
        rig.pushSensor(makeImuElem(SensorType::IMU_GYRO, id, ts++, 0.01f, 0.02f, 0.03f));
    }
}

} // namespace test_support

#endif
