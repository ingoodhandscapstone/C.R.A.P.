#ifndef WRIST_ORIENTATION_PROCESSOR_H
#define WRIST_ORIENTATION_PROCESSOR_H

#include "ProcessorConfigs.h"
#include "InEKF/Core"
#include "InEKF/Inertial"
#include "AccelGravityMeasureModel.h"

#include <cstdint>
#include <optional>

class WristOrientationProcessor {
    static const int CALIBRATION_SAMPLE_COUNT;
    static const Eigen::Vector3d GRAVITY;

    static const float VELOCITY_VARIANCE;
    static const float POSITION_VARIANCE;

    ImuProcessingConfig * config;

    std::optional<InEKF::InEKF<InEKF::InertialProcess>> ekf;
    InEKF::InertialProcess pModel;
    AccelGravityMeasureModel mModel;

    int sampleCalibrationCount;

    Eigen::Matrix3d initialOrientation;
    Eigen::Vector3d initialGyroBias;

    InEKF::SE3<2, 6> currentState;
    uint32_t currentTimestamp;

    void correctOrthoOfReading(Eigen::Vector3d& accel);
    double wrapTo360(double angleDegrees) const;

  public:
    WristOrientationProcessor()
        : config(nullptr),
          ekf(std::nullopt),
          pModel(),
          mModel(),
          sampleCalibrationCount(0),
          initialOrientation(Eigen::Matrix3d::Identity()),
          initialGyroBias(Eigen::Vector3d::Zero()),
          currentState(),
          currentTimestamp(0) {}

    bool initialize(ImuProcessingConfig * config);
    bool calibrate(Eigen::Vector3d& accels, Eigen::Vector3d& gyro);

    bool reset();

    void setInitialTimestamp(uint32_t timestamp);
    void predict(Eigen::Vector6d& u, uint32_t timestamp);
    void update(Eigen::Vector3d& accel);

    Eigen::Matrix3d getHandOrientationMatrix();
    void getXAxisAngle(float& angle);
    void getYAxisAngle(float& angle);
};

#endif
