#include "FingerAbductionProcessor.h"

#include <Eigen/Geometry>
#include <cmath>

const int FingerAbductionProcessor::CALIBRATION_SAMPLE_COUNT = 100;
const Eigen::Vector3d FingerAbductionProcessor::GRAVITY(0.0, 0.0, -9.81);
const float FingerAbductionProcessor::VELOCITY_VARIANCE = 0.0f;
const float FingerAbductionProcessor::POSITION_VARIANCE = 0.0f;


bool FingerAbductionProcessor::calibrate(Eigen::Vector3d& accels, Eigen::Vector3d& gyro){
    if(sampleCalibrationCount >= CALIBRATION_SAMPLE_COUNT){
        return true;
    }

    // Current accelerometer reading = R_inertial_respect_body * GRAVITY (assume small bias for now)
    // Solve for R_inertial_respect_body and take transpose of that to get orientation reading
    // Correct orientation reading using correctOrthoReading
    // Use initialOrientation and average orientation reading

    // Use gyro value (user should be still) and average reading using initialGyroBias
    Eigen::Vector3d correctedAccel = accels;
    correctOrthoOfReading(correctedAccel);

    const double accelNorm = correctedAccel.norm();
    if (accelNorm <= 1e-9) {
        return false;
    }

    // R_inertial_respect_body maps inertial gravity vector to measured body-frame accel.
    Eigen::Matrix3d rInertialRespectBody = Eigen::Quaterniond::FromTwoVectors(GRAVITY, correctedAccel).toRotationMatrix();
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

void FingerAbductionProcessor::correctOrthoOfReading(Eigen::Vector3d& accel){
    // Use the Imu Processing config correction matrix and bias to correct accel
    if(config == nullptr){
        return;
    }
    accel = (config->orthoCorrectionMat * accel) + config->orthoCorrectionBias;
}


bool FingerAbductionProcessor::initialize(ImuProcessingConfig * config){
    if(config == nullptr){
        return false;
    }
    this->config = config;

    Eigen::Matrix<double, 12, 1> xi = Eigen::Matrix<double, 12, 1>::Zero();
    xi.segment<3>(6) = initialGyroBias;
    xi.tail<3>() = Eigen::Vector3d::Zero();

    InEKF::SE3<2, 6>::MatrixCov initialCov = InEKF::SE3<2, 6>::MatrixCov::Zero();
    initialCov.block<3,3>(0,0) = config->orientationVariance.asDiagonal();
    initialCov.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * VELOCITY_VARIANCE;
    initialCov.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * POSITION_VARIANCE;
    initialCov.block<3,3>(9,9) = config->gyroBiasNoise.cwiseProduct(config->gyroBiasNoise).asDiagonal();
    initialCov.block<3,3>(12,12) = config->accelsBiasNoise.cwiseProduct(config->accelsBiasNoise).asDiagonal();

    currentState = InEKF::SE3<2, 6>(InEKF::SO3<>(initialOrientation), xi, initialCov);

    InEKF::InertialProcess::MatrixCov q = InEKF::InertialProcess::MatrixCov::Zero();
    q.block<3,3>(0,0) = config->gyroProcessNoise.cwiseProduct(config->gyroProcessNoise).asDiagonal();
    q.block<3,3>(3,3) = config->accelProcessNoise.cwiseProduct(config->accelProcessNoise).asDiagonal();
    q.block<3,3>(9,9) = config->gyroBiasNoise.cwiseProduct(config->gyroBiasNoise).asDiagonal();
    q.block<3,3>(12,12) = config->accelsBiasNoise.cwiseProduct(config->accelsBiasNoise).asDiagonal();
    pModel.setQ(q);

    mModel.setGravity(GRAVITY);
    mModel.setCovariance(config->accelMeasurementCovariance);

    ekf.emplace(&pModel, currentState, InEKF::ERROR::RIGHT);
    ekf->addMeasureModel("accel", &mModel);

    return true;
}

bool FingerAbductionProcessor::reset() {
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


void FingerAbductionProcessor::setInitialTimestamp(uint32_t timestamp){
    currentTimestamp = timestamp;
}

void FingerAbductionProcessor::setGyro(const Eigen::Vector3d& gyro, uint32_t timestamp){
    currentGyro = gyro;
    currentGyroTimestamp = timestamp;
}

void FingerAbductionProcessor::setAccel(const Eigen::Vector3d& accel, uint32_t timestamp){
    currentAccel = accel;
    currentAccelTimestamp = timestamp;
}

bool FingerAbductionProcessor::hasGyroAndAccel() const{
    return currentGyro.has_value() && currentAccel.has_value() &&
           currentGyroTimestamp.has_value() && currentAccelTimestamp.has_value();
}

void FingerAbductionProcessor::predict(){
    if(!ekf.has_value()){
        return;
    }

    if(!hasGyroAndAccel()){
        return;
    }

    const uint64_t timestampAverage = (static_cast<uint64_t>(currentGyroTimestamp.value()) +
                                       static_cast<uint64_t>(currentAccelTimestamp.value())) / 2ULL;
    const uint32_t timestamp = static_cast<uint32_t>(timestampAverage);

    Eigen::Vector6d u;
    u.head<3>() = currentGyro.value();
    u.tail<3>() = currentAccel.value();

    // Apply ortho correction to accels
    Eigen::Vector6d correctedU = u;
    Eigen::Vector3d correctedAccel = correctedU.tail<3>();
    correctOrthoOfReading(correctedAccel);
    correctedU.tail<3>() = correctedAccel;

    // Call predict and set currentState to that value
    if(currentTimestamp == 0){
        currentTimestamp = timestamp;
    }
    const double dt = static_cast<double>(timestamp - currentTimestamp) * 1e-6;
    currentTimestamp = timestamp;
    currentState = ekf->predict(correctedU, dt);
    hasPredictedThisCycle = true;
}

void FingerAbductionProcessor::update(){
    if(!ekf.has_value()){
        return;
    }

    if(!hasPredictedThisCycle || !currentAccel.has_value()){
        return;
    }

    // Apply ortho correction to accels
    Eigen::Vector3d correctedAccel = currentAccel.value();
    correctOrthoOfReading(correctedAccel);

    // Call update and set currentState to that value
    Eigen::VectorXd z(3);
    z = correctedAccel;
    currentState = ekf->update("accel", z);

    currentGyro.reset();
    currentAccel.reset();
    currentGyroTimestamp.reset();
    currentAccelTimestamp.reset();
    hasPredictedThisCycle = false;
}

void FingerAbductionProcessor::getAngle(float& angle, Eigen::Matrix3d handOrientation){
    // State being a 5 x 5 matrix I believe, I need to extract just the orientation
    // Use orientation of hand imu and current finger imu orientation to get the relative orientation 
    // Convert into yaw angle (degrees), about positive z axis, and finger relative to hand, 0 to 360
    const Eigen::Matrix3d fingerOrientation = currentState.R()();
    const Eigen::Matrix3d fingerRespectHand = handOrientation.transpose() * fingerOrientation;

    double yawDegrees = std::atan2(fingerRespectHand(1, 0), fingerRespectHand(0, 0)) *
                        (180.0 / 3.14159265358979323846);

    angle = static_cast<float>(abs(yawDegrees)); // Absolute value because angle from center line away
}
