#include "test_support.h"

#include "gtest/gtest.h"

using namespace test_support;

namespace {

TEST(WorkerThreadedTest, FlexAndImuLanesCanCalibrateConcurrentlyWithSharedCalibrationQueue) {
    WorkerRig flexRig;
    WorkerRig imuRig;

    std::queue<CalibrationStatusMessage> sharedCalibrationQueue;
    std::mutex sharedCalibrationMutex;

    flexRig.initializeFlexLane(&sharedCalibrationQueue, &sharedCalibrationMutex);
    imuRig.initializeImuForceLane(&sharedCalibrationQueue, &sharedCalibrationMutex);

    flexRig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER);
    flexRig.pushCommand(SessionCommand::CALIBRATE_SESSION);
    imuRig.pushCommand(SessionCommand::SESSION_CONFIG_POINTER);
    imuRig.pushCommand(SessionCommand::CALIBRATE_SESSION);

    flexRig.pushSensor(makeFlexElem(SensorID::POINTER_MCP_FLEX, 1u, 100u));
    flexRig.pushSensor(makeFlexElem(SensorID::POINTER_PIP_FLEX, 2u, 100u));
    flexRig.pushSensor(makeFlexElem(SensorID::POINTER_DIP_FLEX, 3u, 100u));

    enqueueImuCalibrationSamples(imuRig, SensorID::HAND_IMU, 100, 1000u);
    enqueueImuCalibrationSamples(imuRig, SensorID::POINTER_IMU, 100, 2000u);

    std::jthread flexThread([&](std::stop_token stopToken) { flexRig.worker.run(stopToken); });
    std::jthread imuThread([&](std::stop_token stopToken) { imuRig.worker.run(stopToken); });

    ASSERT_TRUE(waitUntil([&]() { return lockedQueueSize(sharedCalibrationMutex, sharedCalibrationQueue) >= 2u; },
                          4000));

    flexThread.request_stop();
    imuThread.request_stop();

    CalibrationStatusMessage first{};
    CalibrationStatusMessage second{};
    ASSERT_TRUE(popLocked(sharedCalibrationMutex, sharedCalibrationQueue, first));
    ASSERT_TRUE(popLocked(sharedCalibrationMutex, sharedCalibrationQueue, second));

    EXPECT_EQ(first.requiredCount, 2u);
    EXPECT_EQ(second.requiredCount, 2u);
    EXPECT_EQ(first.epoch, second.epoch);
}

} // namespace
