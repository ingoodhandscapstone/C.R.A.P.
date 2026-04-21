#include "WristOrientationProcessor.h"

#include <Eigen/Geometry>
#include <cmath>

const int WristOrientationProcessor::CALIBRATION_SAMPLE_COUNT = 100;
const Eigen::Vector3d WristOrientationProcessor::GRAVITY(0.0, 0.0, -9.81);
const float WristOrientationProcessor::VELOCITY_VARIANCE = 0.0f;
const float WristOrientationProcessor::POSITION_VARIANCE = 0.0f;

bool WristOrientationProcessor::calibrate(Eigen::Vector3d& accels, Eigen::Vector3d& gyro) {
    if (sampleCalibrationCount >= CALIBRATION_SAMPLE_COUNT) {
        return true;
    }

    Eigen::Vector3d correctedAccel = accels;
    correctOrthoOfReading(correctedAccel);

    const double accelNorm = correctedAccel.norm();
    if (accelNorm <= 1e-9) {
        return false;
    }

    Eigen::Matrix3d rInertialRespectBody =
        Eigen::Quaterniond::FromTwoVectors(GRAVITY, correctedAccel).toRotationMatrix();
    Eigen::Matrix3d orientationReading = rInertialRespectBody.transpose();

    if (sampleCalibrationCount == 0) {
        initialOrientation = orientationReading;
        initialGyroBias = gyro;
    } else {
        const double n = static_cast<double>(sampleCalibrationCount);
        initialOrientation = ((initialOrientation * n) + orientationReading) / (n + 1.0);
        initialGyroBias = ((initialGyroBias * n) + gyro) / (n + 1.0);
    }

    sampleCalibrationCount++;
    return sampleCalibrationCount >= CALIBRATION_SAMPLE_COUNT;
}

void WristOrientationProcessor::correctOrthoOfReading(Eigen::Vector3d& accel) {
    accel = (config.orthoCorrectionMat * accel) + config.orthoCorrectionBias;
}

bool WristOrientationProcessor::initialize(ImuProcessingConfig config) {
    this->config = config;

    Eigen::Matrix<double, 12, 1> xi = Eigen::Matrix<double, 12, 1>::Zero();
    xi.segment<3>(6) = initialGyroBias;
    xi.tail<3>() = Eigen::Vector3d::Zero();

    InEKF::SE3<2, 6>::MatrixCov initialCov = InEKF::SE3<2, 6>::MatrixCov::Zero();
    initialCov.block<3, 3>(0, 0) = config.orientationVariance.asDiagonal();
    initialCov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * VELOCITY_VARIANCE;
    initialCov.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * POSITION_VARIANCE;
    initialCov.block<3, 3>(9, 9) = config.gyroBiasNoise.cwiseProduct(config.gyroBiasNoise).asDiagonal();
    initialCov.block<3, 3>(12, 12) = config.accelsBiasNoise.cwiseProduct(config.accelsBiasNoise).asDiagonal();

    currentState = InEKF::SE3<2, 6>(InEKF::SO3<>(initialOrientation), xi, initialCov);

    InEKF::InertialProcess::MatrixCov q = InEKF::InertialProcess::MatrixCov::Zero();
    q.block<3, 3>(0, 0) = config.gyroProcessNoise.cwiseProduct(config.gyroProcessNoise).asDiagonal();
    q.block<3, 3>(3, 3) = config.accelProcessNoise.cwiseProduct(config.accelProcessNoise).asDiagonal();
    q.block<3, 3>(9, 9) = config.gyroBiasNoise.cwiseProduct(config.gyroBiasNoise).asDiagonal();
    q.block<3, 3>(12, 12) = config.accelsBiasNoise.cwiseProduct(config.accelsBiasNoise).asDiagonal();
    pModel.setQ(q);

    mModel.setGravity(GRAVITY);
    mModel.setCovariance(config.accelMeasurementCovariance);

    ekf.emplace(&pModel, currentState, InEKF::ERROR::RIGHT);
    ekf->addMeasureModel("accel", &mModel);

    return true;
}

bool WristOrientationProcessor::reset() {
    ekf.reset();
    sampleCalibrationCount = 0;
    currentTimestamp = 0;
    currentState = InEKF::SE3<2, 6>();
    initialOrientation = Eigen::Matrix3d::Identity();
    initialGyroBias = Eigen::Vector3d::Zero();
    return true;
}

void WristOrientationProcessor::setInitialTimestamp(uint32_t timestamp) { currentTimestamp = timestamp; }

void WristOrientationProcessor::predict(Eigen::Vector6d& u, uint32_t timestamp) {
    if (!ekf.has_value()) {
        return;
    }

    Eigen::Vector6d correctedU = u;
    Eigen::Vector3d correctedAccel = correctedU.tail<3>();
    correctOrthoOfReading(correctedAccel);
    correctedU.tail<3>() = correctedAccel;

    const double dt = static_cast<double>(timestamp - currentTimestamp) * 1e-6;
    currentTimestamp = timestamp;
    currentState = ekf->predict(correctedU, dt);
}

void WristOrientationProcessor::update(Eigen::Vector3d& accel) {
    if (!ekf.has_value()) {
        return;
    }

    Eigen::Vector3d correctedAccel = accel;
    correctOrthoOfReading(correctedAccel);

    Eigen::VectorXd z(3);
    z = correctedAccel;
    currentState = ekf->update("accel", z);
}

Eigen::Matrix3d WristOrientationProcessor::getHandOrientationMatrix() { return currentState.R()(); }

double WristOrientationProcessor::wrapTo360(double angleDegrees) const {
    if (angleDegrees < 0.0) {
        angleDegrees += 360.0;
    }
    return angleDegrees;
}




void WristOrientationProcessor::getXAxisAngle(float& angle) {
    const Eigen::Matrix3d relativeOrientation = initialOrientation.transpose() * currentState.R()();
    const double xDegrees = std::atan2(relativeOrientation(2, 1), relativeOrientation(2, 2)) *
                            (180.0 / 3.14159265358979323846);
    angle = static_cast<float>(xDegrees);
}

void WristOrientationProcessor::getYAxisAngle(float& angle) {
    const Eigen::Matrix3d relativeOrientation = initialOrientation.transpose() * currentState.R()();
    const double yDegrees =
        std::atan2(-relativeOrientation(2, 0),
                   std::sqrt(relativeOrientation(2, 1) * relativeOrientation(2, 1) +
                             relativeOrientation(2, 2) * relativeOrientation(2, 2))) *
        (180.0 / 3.14159265358979323846);
    angle = static_cast<float>(yDegrees);
}
