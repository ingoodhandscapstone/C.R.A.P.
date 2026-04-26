#include "WristOrientationProcessor.h"

#include <Eigen/Geometry>
#include <cmath>

const int WristOrientationProcessor::CALIBRATION_SAMPLE_COUNT = 10;
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
    if(config == nullptr){
        return;
    }
    accel = (config->orthoCorrectionMat * accel) + config->orthoCorrectionBias;
}

bool WristOrientationProcessor::initialize(ImuProcessingConfig * config) {
    if(config == nullptr){
        return false;
    }
    this->config = config;

    Eigen::Matrix<double, 12, 1> xi = Eigen::Matrix<double, 12, 1>::Zero();
    xi.segment<3>(6) = initialGyroBias;
    xi.tail<3>() = Eigen::Vector3d::Zero();

    InEKF::SE3<2, 6>::MatrixCov initialCov = InEKF::SE3<2, 6>::MatrixCov::Zero();
    initialCov.block<3, 3>(0, 0) = config->orientationVariance.asDiagonal();
    initialCov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * VELOCITY_VARIANCE;
    initialCov.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * POSITION_VARIANCE;
    initialCov.block<3, 3>(9, 9) = config->gyroBiasNoise.cwiseProduct(config->gyroBiasNoise).asDiagonal();
    initialCov.block<3, 3>(12, 12) = config->accelsBiasNoise.cwiseProduct(config->accelsBiasNoise).asDiagonal();

    currentState = InEKF::SE3<2, 6>(InEKF::SO3<>(initialOrientation), xi, initialCov);

    InEKF::InertialProcess::MatrixCov q = InEKF::InertialProcess::MatrixCov::Zero();
    q.block<3, 3>(0, 0) = config->gyroProcessNoise.cwiseProduct(config->gyroProcessNoise).asDiagonal();
    q.block<3, 3>(3, 3) = config->accelProcessNoise.cwiseProduct(config->accelProcessNoise).asDiagonal();
    q.block<3, 3>(9, 9) = config->gyroBiasNoise.cwiseProduct(config->gyroBiasNoise).asDiagonal();
    q.block<3, 3>(12, 12) = config->accelsBiasNoise.cwiseProduct(config->accelsBiasNoise).asDiagonal();
    pModel.setQ(q);

    mModel.setGravity(GRAVITY);
    mModel.setCovariance(config->accelMeasurementCovariance);

    ekf.emplace(&pModel, currentState, InEKF::ERROR::RIGHT);
    ekf->addMeasureModel("accel", &mModel);

    return true;
}

bool WristOrientationProcessor::reset() {
    sampleCalibrationCount = 0;
    currentTimestamp = 0;
    currentGyro.reset();
    currentAccel.reset();
    currentGyroTimestamp.reset();
    currentAccelTimestamp.reset();
    hasPredictedThisCycle = false;
    initialOrientation = Eigen::Matrix3d::Identity();
    initialGyroBias = Eigen::Vector3d::Zero();
    return true;
}

void WristOrientationProcessor::setInitialTimestamp(uint32_t timestamp) { currentTimestamp = timestamp; }

void WristOrientationProcessor::setGyro(const Eigen::Vector3d& gyro, uint32_t timestamp) {
    currentGyro = gyro;
    currentGyroTimestamp = timestamp;
}

void WristOrientationProcessor::setAccel(const Eigen::Vector3d& accel, uint32_t timestamp) {
    currentAccel = accel;
    currentAccelTimestamp = timestamp;
}

bool WristOrientationProcessor::hasGyroAndAccel() const {
    return currentGyro.has_value() && currentAccel.has_value() &&
           currentGyroTimestamp.has_value() && currentAccelTimestamp.has_value();
}

void WristOrientationProcessor::predict() {
    if (!ekf.has_value()) {
        return;
    }

    if (!hasGyroAndAccel()) {
        return;
    }

    const uint64_t timestampAverage = (static_cast<uint64_t>(currentGyroTimestamp.value()) +
                                       static_cast<uint64_t>(currentAccelTimestamp.value())) / 2ULL;
    const uint32_t timestamp = static_cast<uint32_t>(timestampAverage);

    Eigen::Vector6d u;
    u.head<3>() = currentGyro.value();
    u.tail<3>() = currentAccel.value();

    Eigen::Vector6d correctedU = u;
    Eigen::Vector3d correctedAccel = correctedU.tail<3>();
    correctOrthoOfReading(correctedAccel);
    correctedU.tail<3>() = correctedAccel;

    if (currentTimestamp == 0) {
        currentTimestamp = timestamp;
    }
    const double dt = static_cast<double>(timestamp - currentTimestamp) * 1e-6;
    currentTimestamp = timestamp;
    currentState = ekf->predict(correctedU, dt);
    hasPredictedThisCycle = true;
}

void WristOrientationProcessor::update() {
    if (!ekf.has_value()) {
        return;
    }

    if (!hasPredictedThisCycle || !currentAccel.has_value()) {
        return;
    }

    Eigen::Vector3d correctedAccel = currentAccel.value();
    correctOrthoOfReading(correctedAccel);

    Eigen::VectorXd z(3);
    z = correctedAccel;
    currentState = ekf->update("accel", z);

    currentGyro.reset();
    currentAccel.reset();
    currentGyroTimestamp.reset();
    currentAccelTimestamp.reset();
    hasPredictedThisCycle = false;
}

Eigen::Matrix3d WristOrientationProcessor::getHandOrientationMatrix() { return currentState.R()(); }

Eigen::Matrix3d WristOrientationProcessor::getInitialOrientationMatrix() const { return initialOrientation; }

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
