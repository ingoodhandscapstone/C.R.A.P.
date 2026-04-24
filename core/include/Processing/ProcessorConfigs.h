#ifndef PROCESSOR_CONFIGS_H
#define PROCESSOR_CONFIGS_H

#include "Eigen/Core"
#include <map>
#include <vector>

struct ImuProcessingConfig {
    Eigen::Vector3d gyroProcessNoise; // standard deviation
    Eigen::Vector3d accelProcessNoise; // standard deviation
    Eigen::Vector3d accelsBiasNoise; // standard deviation
    Eigen::Vector3d gyroBiasNoise; // standard deviation
    Eigen::Matrix3d orthoCorrectionMat;
    Eigen::Vector3d orthoCorrectionBias;
    Eigen::Vector3d orientationVariance;
    Eigen::Matrix3d accelMeasurementCovariance;
};

struct ResistiveSensorConfig {
    float noiseFloor;
    float adcLSB;
    float deadband;
    float calibratePositionVoltage;
    std::vector<float> loadingPieceCoef; // The internal vector will have know amount of coefficients
    std::vector<float> unloadingPieceCoef; // The internal vector will have know amount of coefficients
};






#endif
