#include "test_support.h"

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"
#include "MQTTWorker.h"

using namespace test_support;

namespace {

using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

class ComWorkerFixture : public ::testing::Test {
  protected:
    std::queue<SessionCommand> mqttForwardCommandQueue;
    std::queue<DataToProcessorElement> sensorDataProcessingFlexSPO2Queue;
    std::queue<DataToProcessorElement> sensorDataProcessingImuForceQueue;
    std::queue<SessionCommand> comCommandForwardProcessingFlexSPO2Queue;
    std::queue<SessionCommand> comCommandForwardProcessingImuForceQueue;

    std::mutex mqttForwardCommandMutex;
    std::mutex sensorDataProcessingFlexSPO2Mutex;
    std::mutex sensorDataProcessingImuForceMutex;
    std::mutex comCommandForwardProcessingFlexSPO2Mutex;
    std::mutex comCommandForwardProcessingImuForceMutex;

    MockCommunication mockCom;
    ComWorker worker;

    void SetUp() override {
        worker.initialize(
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
            &mockCom);
    }
};

TEST_F(ComWorkerFixture, ForwardsCommandToBothQueuesAndWritesToBothEndpoints) {
    pushLocked(mqttForwardCommandMutex, mqttForwardCommandQueue, SessionCommand::SESSION_START);

    EXPECT_CALL(mockCom, read(_, _)).WillRepeatedly(Return(false));
    EXPECT_CALL(mockCom, write(Endpoints::COMMAND_CHAR_GRIPPER, _))
        .Times(1)
        .WillOnce(Invoke([](const Endpoints&, std::vector<uint8_t>& message) {
            EXPECT_EQ(message.size(), 1u);
            EXPECT_EQ(message[0], static_cast<uint8_t>(SessionCommand::SESSION_START));
            return true;
        }));
    EXPECT_CALL(mockCom, write(Endpoints::COMMAND_CHAR_GLOVE, _))
        .Times(1)
        .WillOnce(Invoke([](const Endpoints&, std::vector<uint8_t>& message) {
            EXPECT_EQ(message.size(), 1u);
            EXPECT_EQ(message[0], static_cast<uint8_t>(SessionCommand::SESSION_START));
            return true;
        }));

    std::jthread thread([&](std::stop_token stopToken) { worker.run(stopToken); });

    ASSERT_TRUE(waitUntil([&]() {
        return lockedQueueSize(comCommandForwardProcessingFlexSPO2Mutex,
                               comCommandForwardProcessingFlexSPO2Queue) == 1 &&
               lockedQueueSize(comCommandForwardProcessingImuForceMutex,
                               comCommandForwardProcessingImuForceQueue) == 1;
    }));

    thread.request_stop();

    SessionCommand flexCommand = SessionCommand::NONE;
    SessionCommand imuCommand = SessionCommand::NONE;
    ASSERT_TRUE(popLocked(comCommandForwardProcessingFlexSPO2Mutex,
                          comCommandForwardProcessingFlexSPO2Queue,
                          flexCommand));
    ASSERT_TRUE(popLocked(comCommandForwardProcessingImuForceMutex,
                          comCommandForwardProcessingImuForceQueue,
                          imuCommand));
    EXPECT_EQ(flexCommand, SessionCommand::SESSION_START);
    EXPECT_EQ(imuCommand, SessionCommand::SESSION_START);
}

TEST_F(ComWorkerFixture, DeserializesAndRoutesAllSensorMessageTypesWithExactData) {
    std::unordered_map<int, std::vector<uint8_t>> messages;
    std::unordered_map<int, bool> served;

    messages[static_cast<int>(Endpoints::IMU_GYRO_CHAR)] =
        serializeSensorMessage(SensorType::IMU_GYRO,
                               SensorID::HAND_IMU,
                               101u,
                               {floatToU32(1.5f), floatToU32(-2.0f), floatToU32(3.25f)});
    messages[static_cast<int>(Endpoints::IMU_ACCEL_CHAR)] =
        serializeSensorMessage(SensorType::IMU_ACCEL,
                               SensorID::POINTER_IMU,
                               102u,
                               {floatToU32(4.0f), floatToU32(5.0f), floatToU32(6.0f)});
    messages[static_cast<int>(Endpoints::FLEX_CHAR)] =
        serializeSensorMessage(SensorType::FLEX, SensorID::POINTER_MCP_FLEX, 103u, {1234u});
    messages[static_cast<int>(Endpoints::SPO2_CHAR)] =
        serializeSensorMessage(SensorType::SPO2, SensorID::WRIST_SPO2, 104u, {111u, 222u});
    messages[static_cast<int>(Endpoints::FORCE_CHAR)] =
        serializeSensorMessage(SensorType::FORCE, SensorID::POINTER_FORCE, 105u, {333u});

    EXPECT_CALL(mockCom, write(_, _)).WillRepeatedly(Return(true));
    EXPECT_CALL(mockCom, read(_, _)).WillRepeatedly(Invoke(
        [&](const Endpoints& endpoint, std::vector<uint8_t>& message) {
            const int key = static_cast<int>(endpoint);
            auto iter = messages.find(key);
            if (iter != messages.end() && !served[key]) {
                message = iter->second;
                served[key] = true;
                return true;
            }
            message.clear();
            return false;
        }));

    std::jthread thread([&](std::stop_token stopToken) { worker.run(stopToken); });

    ASSERT_TRUE(waitUntil([&]() {
        return lockedQueueSize(sensorDataProcessingFlexSPO2Mutex, sensorDataProcessingFlexSPO2Queue) == 2 &&
               lockedQueueSize(sensorDataProcessingImuForceMutex, sensorDataProcessingImuForceQueue) == 3;
    }));

    thread.request_stop();

    DataToProcessorElement elem;
    ASSERT_TRUE(popLocked(sensorDataProcessingImuForceMutex, sensorDataProcessingImuForceQueue, elem));
    EXPECT_EQ(elem.type, SensorType::IMU_GYRO);
    EXPECT_EQ(elem.id, SensorID::HAND_IMU);
    EXPECT_EQ(elem.timestamp, 101u);
    ASSERT_EQ(elem.data.size(), 3u);
    EXPECT_EQ(elem.data[0], floatToU32(1.5f));
    EXPECT_EQ(elem.data[1], floatToU32(-2.0f));
    EXPECT_EQ(elem.data[2], floatToU32(3.25f));

    ASSERT_TRUE(popLocked(sensorDataProcessingImuForceMutex, sensorDataProcessingImuForceQueue, elem));
    EXPECT_EQ(elem.type, SensorType::IMU_ACCEL);
    EXPECT_EQ(elem.id, SensorID::POINTER_IMU);
    EXPECT_EQ(elem.timestamp, 102u);
    ASSERT_EQ(elem.data.size(), 3u);
    EXPECT_EQ(elem.data[0], floatToU32(4.0f));
    EXPECT_EQ(elem.data[1], floatToU32(5.0f));
    EXPECT_EQ(elem.data[2], floatToU32(6.0f));

    ASSERT_TRUE(popLocked(sensorDataProcessingImuForceMutex, sensorDataProcessingImuForceQueue, elem));
    EXPECT_EQ(elem.type, SensorType::FORCE);
    EXPECT_EQ(elem.id, SensorID::POINTER_FORCE);
    EXPECT_EQ(elem.timestamp, 105u);
    ASSERT_EQ(elem.data.size(), 1u);
    EXPECT_EQ(elem.data[0], 333u);

    ASSERT_TRUE(popLocked(sensorDataProcessingFlexSPO2Mutex, sensorDataProcessingFlexSPO2Queue, elem));
    EXPECT_EQ(elem.type, SensorType::FLEX);
    EXPECT_EQ(elem.id, SensorID::POINTER_MCP_FLEX);
    EXPECT_EQ(elem.timestamp, 103u);
    ASSERT_EQ(elem.data.size(), 1u);
    EXPECT_EQ(elem.data[0], 1234u);

    ASSERT_TRUE(popLocked(sensorDataProcessingFlexSPO2Mutex, sensorDataProcessingFlexSPO2Queue, elem));
    EXPECT_EQ(elem.type, SensorType::SPO2);
    EXPECT_EQ(elem.id, SensorID::WRIST_SPO2);
    EXPECT_EQ(elem.timestamp, 104u);
    ASSERT_EQ(elem.data.size(), 2u);
    EXPECT_EQ(elem.data[0], 111u);
    EXPECT_EQ(elem.data[1], 222u);
}

TEST_F(ComWorkerFixture, IgnoresMalformedAndUnroutableMessages) {
    std::unordered_map<int, std::vector<uint8_t>> messages;
    std::unordered_map<int, bool> served;

    messages[static_cast<int>(Endpoints::FLEX_CHAR)] = {static_cast<uint8_t>(SensorType::FLEX),
                                                        static_cast<uint8_t>(SensorID::POINTER_MCP_FLEX),
                                                        0x00,
                                                        0x00,
                                                        0x00};
    messages[static_cast<int>(Endpoints::IMU_ACCEL_CHAR)] =
        serializeSensorMessage(SensorType::IMU_ACCEL,
                               SensorID::HAND_IMU_Y,
                               201u,
                               {floatToU32(1.0f), floatToU32(2.0f), floatToU32(3.0f)});
    messages[static_cast<int>(Endpoints::IMU_GYRO_CHAR)] = {99u, 1u, 0u, 0u, 0u, 1u, 0u, 0u, 0u, 1u};

    EXPECT_CALL(mockCom, write(_, _)).WillRepeatedly(Return(true));
    EXPECT_CALL(mockCom, read(_, _)).WillRepeatedly(Invoke(
        [&](const Endpoints& endpoint, std::vector<uint8_t>& message) {
            const int key = static_cast<int>(endpoint);
            auto iter = messages.find(key);
            if (iter != messages.end() && !served[key]) {
                message = iter->second;
                served[key] = true;
                return true;
            }
            message.clear();
            return false;
        }));

    std::jthread thread([&](std::stop_token stopToken) { worker.run(stopToken); });
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    thread.request_stop();

    EXPECT_EQ(lockedQueueSize(sensorDataProcessingFlexSPO2Mutex, sensorDataProcessingFlexSPO2Queue), 0u);
    EXPECT_EQ(lockedQueueSize(sensorDataProcessingImuForceMutex, sensorDataProcessingImuForceQueue), 0u);
}

class FlexLaneFixture : public ::testing::Test {
  protected:
    WorkerRig rig;
    void SetUp() override {
        rig.initializeFlexLane();
    }
};

class ImuForceLaneFixture : public ::testing::Test {
  protected:
    WorkerRig rig;
    void SetUp() override {
        rig.initializeImuForceLane();
    }
};

TEST_F(FlexLaneFixture, PointerCalibrationWaitsForAllActiveSensors) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    rig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 1u, 100u));

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    thread.request_stop();

    EXPECT_EQ(rig.calibrationStatusSize(), 0u);
    EXPECT_EQ(rig.forwardSize(), 0u);
}

TEST_F(FlexLaneFixture, PointerMiddleCalibrationRequiresBothFingerSets) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER_MIDDLE);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    rig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 1u, 100u));
    rig.pushSensor(makeFlexElem(SensorID::POINTER_PIP_FLEX, 2u, 101u));
    rig.pushSensor(makeFlexElem(SensorID::POINTER_DIP_FLEX, 3u, 102u));
    rig.pushSensor(makeFlexElem(SensorID::MIDDLE_MCP_FLEX, 4u, 103u));
    rig.pushSensor(makeFlexElem(SensorID::MIDDLE_PIP_FLEX, 5u, 104u));

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    thread.request_stop();

    EXPECT_EQ(rig.calibrationStatusSize(), 0u);
}

TEST_F(FlexLaneFixture, RunningOutputsDataAndStopsOnSessionStop) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    rig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 1u, 100u));
    rig.pushSensor(makeFlexElem(SensorID::POINTER_PIP_FLEX, 2u, 100u));
    rig.pushSensor(makeFlexElem(SensorID::POINTER_DIP_FLEX, 3u, 100u));

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });

    ASSERT_TRUE(waitUntil([&]() { return rig.calibrationStatusSize() == 1u; }));
    rig.pushCommand(SessionCommand::SESSION_START);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));

    rig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 10u, 120u));
    ASSERT_TRUE(waitUntil([&]() { return rig.forwardSize() >= 1u; }));

    const size_t beforeStopOutputCount = rig.forwardSize();
    rig.pushCommand(SessionCommand::SESSION_STOP);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));

    rig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 20u, 130u));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    thread.request_stop();

    EXPECT_EQ(rig.forwardSize(), beforeStopOutputCount);
}

TEST_F(FlexLaneFixture, ResetSeparatesSessionConfigurations) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    rig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 1u, 100u));
    rig.pushSensor(makeFlexElem(SensorID::POINTER_PIP_FLEX, 2u, 100u));
    rig.pushSensor(makeFlexElem(SensorID::POINTER_DIP_FLEX, 3u, 100u));

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });

    ASSERT_TRUE(waitUntil([&]() { return rig.calibrationStatusSize() == 1u; }));
    rig.pushCommand(SessionCommand::SESSION_START);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));
    rig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 10u, 120u));
    ASSERT_TRUE(waitUntil([&]() { return rig.forwardSize() >= 1u; }));

    rig.pushCommand(SessionCommand::SESSION_STOP);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));
    clearLocked(rig.forwardMutex, rig.forwardQueue);

    rig.pushCommand(SessionCommand::SESSION_CONFIG_THUMB);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    rig.pushSensor(makeFlexElem(SensorID::THUMB_MCP_FLEX, 30u, 100u));
    rig.pushSensor(makeFlexElem(SensorID::THUMB_PIP_FLEX, 31u, 100u));
    ASSERT_TRUE(waitUntil([&]() { return rig.calibrationStatusSize() >= 2u; }));

    rig.pushCommand(SessionCommand::SESSION_START);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));

    rig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 40u, 150u));
    rig.pushSensor(makeFlexElem(SensorID::THUMB_MCP_FLEX, 41u, 150u));
    ASSERT_TRUE(waitUntil([&]() { return rig.forwardSize() >= 1u; }));

    DataOutputElement output;
    ASSERT_TRUE(rig.popForward(output));
    thread.request_stop();

    EXPECT_EQ(output.id, SensorID::THUMB_MCP_FLEX);
}

TEST_F(ImuForceLaneFixture, PointerMiddleImuCalibrationGatesUntilAllActiveImusCalibrated) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER_MIDDLE);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);

    enqueueImuCalibrationSamples(rig, SensorID::HAND_IMU, 100, 1000u);
    enqueueImuCalibrationSamples(rig, SensorID::POINTER_IMU, 100, 2000u);

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_EQ(rig.calibrationStatusSize(), 0u);

    enqueueImuCalibrationSamples(rig, SensorID::MIDDLE_IMU, 100, 3000u);
    ASSERT_TRUE(waitUntil([&]() { return rig.calibrationStatusSize() == 1u; }, 3000));

    thread.request_stop();
}

TEST_F(ImuForceLaneFixture, ImuRunningDoesNotOutputUntilGyroAndAccelAreReady) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    enqueueImuCalibrationSamples(rig, SensorID::HAND_IMU, 100, 1000u);
    enqueueImuCalibrationSamples(rig, SensorID::POINTER_IMU, 100, 2000u);

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });
    ASSERT_TRUE(waitUntil([&]() { return rig.calibrationStatusSize() == 1u; }, 3000));

    rig.pushCommand(SessionCommand::SESSION_START);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));

    rig.pushSensor(makeImuElem(SensorType::IMU_ACCEL, SensorID::HAND_IMU, 5000u, 0.0f, 0.0f, -9.81f));
    rig.pushSensor(makeImuElem(SensorType::IMU_ACCEL, SensorID::POINTER_IMU, 5001u, 0.1f, 0.0f, -9.7f));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

    thread.request_stop();
    EXPECT_EQ(rig.forwardSize(), 0u);
}

TEST_F(ImuForceLaneFixture, PointerImuSessionOutputsAfterReadySamples) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    enqueueImuCalibrationSamples(rig, SensorID::HAND_IMU, 100, 1000u);
    enqueueImuCalibrationSamples(rig, SensorID::POINTER_IMU, 100, 2000u);

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });
    ASSERT_TRUE(waitUntil([&]() { return rig.calibrationStatusSize() == 1u; }, 3000));

    rig.pushCommand(SessionCommand::SESSION_START);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));

    rig.pushSensor(makeImuElem(SensorType::IMU_ACCEL, SensorID::HAND_IMU, 6000u, 0.0f, 0.0f, -9.81f));
    rig.pushSensor(makeImuElem(SensorType::IMU_GYRO, SensorID::HAND_IMU, 6001u, 0.01f, 0.01f, 0.01f));
    rig.pushSensor(makeImuElem(SensorType::IMU_ACCEL, SensorID::POINTER_IMU, 6002u, 0.1f, 0.0f, -9.7f));
    rig.pushSensor(makeImuElem(SensorType::IMU_GYRO, SensorID::POINTER_IMU, 6003u, 0.02f, 0.01f, 0.01f));

    ASSERT_TRUE(waitUntil([&]() { return rig.forwardSize() >= 1u; }, 1000));

    DataOutputElement output;
    ASSERT_TRUE(rig.popForward(output));
    thread.request_stop();

    EXPECT_EQ(output.id, SensorID::POINTER_IMU);
}

TEST_F(ImuForceLaneFixture, WristSessionOutputsXAxisAndYAxisData) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_WRIST);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    enqueueImuCalibrationSamples(rig, SensorID::HAND_IMU, 100, 1000u);

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });
    ASSERT_TRUE(waitUntil([&]() { return rig.calibrationStatusSize() == 1u; }, 3000));

    rig.pushCommand(SessionCommand::SESSION_START);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));

    rig.pushSensor(makeImuElem(SensorType::IMU_ACCEL, SensorID::HAND_IMU, 7000u, 0.0f, 0.0f, -9.81f));
    rig.pushSensor(makeImuElem(SensorType::IMU_GYRO, SensorID::HAND_IMU, 7001u, 0.01f, 0.02f, 0.03f));

    ASSERT_TRUE(waitUntil([&]() { return rig.forwardSize() >= 2u; }, 1000));

    DataOutputElement first;
    DataOutputElement second;
    ASSERT_TRUE(rig.popForward(first));
    ASSERT_TRUE(rig.popForward(second));
    thread.request_stop();

    EXPECT_EQ(first.id, SensorID::HAND_IMU);
    EXPECT_EQ(second.id, SensorID::HAND_IMU_Y);
}

TEST_F(ImuForceLaneFixture, ForceSingleAndMultiSensorCalibrationGatingWorks) {
    rig.pushCommand(SessionCommand::SESSION_CONFIG_GRIPPER_POINTER_MIDDLE);
    rig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    rig.pushSensor(makeForceElem(SensorID::POINTER_FORCE, 10u, 100u));

    std::jthread thread([&](std::stop_token stopToken) { rig.worker.run(stopToken); });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_EQ(rig.calibrationStatusSize(), 0u);

    rig.pushSensor(makeForceElem(SensorID::MIDDLE_FORCE, 11u, 101u));
    ASSERT_TRUE(waitUntil([&]() { return rig.calibrationStatusSize() == 1u; }));

    rig.pushCommand(SessionCommand::SESSION_START);
    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(rig.commandMutex, rig.commandQueue) == 0u; }));
    rig.pushSensor(makeForceElem(SensorID::POINTER_FORCE, 20u, 150u));
    ASSERT_TRUE(waitUntil([&]() { return rig.forwardSize() >= 1u; }));

    DataOutputElement output;
    ASSERT_TRUE(rig.popForward(output));
    thread.request_stop();

    EXPECT_EQ(output.id, SensorID::POINTER_FORCE);
}

class MQTTCalibrationFixture : public ::testing::Test {
  protected:
    std::queue<SessionCommand> mqttForwardCommandQueue;
    std::queue<DataOutputElement> flexQueue;
    std::queue<DataOutputElement> imuQueue;
    std::queue<CalibrationStatusMessage> calibrationQueue;

    std::mutex mqttForwardCommandMutex;
    std::mutex flexMutex;
    std::mutex imuMutex;
    std::mutex calibrationMutex;

    MockMQTTClient mockClient;
    MQTTWorker worker{&mockClient};

    void SetUp() override {
        EXPECT_CALL(mockClient, setMessageHandler(_)).Times(1);
        EXPECT_CALL(mockClient, connect()).Times(1).WillOnce(Return(true));
        EXPECT_CALL(mockClient, subscribe(_, _)).Times(1).WillOnce(Return(true));
        ON_CALL(mockClient, isConnected()).WillByDefault(Return(true));
        ASSERT_TRUE(worker.initialize(&mqttForwardCommandQueue,
                                      &flexQueue,
                                      &imuQueue,
                                      &calibrationQueue,
                                      &mqttForwardCommandMutex,
                                      &flexMutex,
                                      &imuMutex,
                                      &calibrationMutex));
    }

    void runWorkerForDrain() {
        std::jthread thread([&](std::stop_token stopToken) { worker.run(stopToken); });
        waitUntil([&]() { return lockedQueueSize(calibrationMutex, calibrationQueue) == 0u; }, 500);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        thread.request_stop();
    }
};

TEST_F(MQTTCalibrationFixture, RequiredCountOnePublishesImmediately) {
    EXPECT_CALL(mockClient, publish(_, std::to_string(static_cast<int>(SessionCommand::CALIBRATION_COMPLETED)), _))
        .Times(1)
        .WillOnce(Return(true));

    pushLocked(calibrationMutex, calibrationQueue, CalibrationStatusMessage{1u, 1u});
    runWorkerForDrain();
}

TEST_F(MQTTCalibrationFixture, RequiredCountTwoPublishesOnlyAfterSecondMessageSameEpoch) {
    EXPECT_CALL(mockClient, publish(_, std::to_string(static_cast<int>(SessionCommand::CALIBRATION_COMPLETED)), _))
        .Times(1)
        .WillOnce(Return(true));

    pushLocked(calibrationMutex, calibrationQueue, CalibrationStatusMessage{5u, 2u});
    pushLocked(calibrationMutex, calibrationQueue, CalibrationStatusMessage{5u, 2u});
    runWorkerForDrain();
}

TEST_F(MQTTCalibrationFixture, RequiredCountZeroIsIgnored) {
    EXPECT_CALL(mockClient, publish(_, _, _)).Times(0);

    pushLocked(calibrationMutex, calibrationQueue, CalibrationStatusMessage{2u, 0u});
    runWorkerForDrain();
}

TEST_F(MQTTCalibrationFixture, OutOfOrderEpochResetsTrackingAndStillRequiresCount) {
    EXPECT_CALL(mockClient, publish(_, std::to_string(static_cast<int>(SessionCommand::CALIBRATION_COMPLETED)), _))
        .Times(1)
        .WillOnce(Return(true));

    pushLocked(calibrationMutex, calibrationQueue, CalibrationStatusMessage{10u, 2u});
    pushLocked(calibrationMutex, calibrationQueue, CalibrationStatusMessage{11u, 2u});
    pushLocked(calibrationMutex, calibrationQueue, CalibrationStatusMessage{11u, 2u});
    runWorkerForDrain();
}

} // namespace
