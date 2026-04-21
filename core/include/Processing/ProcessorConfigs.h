#ifndef PROCESSOR_CONFIGS_H
#define PROCESSOR_COFNIGS_H

#include "Eigen/Core"
#include <map>

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
    // Upper voltage limit of individual piecewise function is what is the key for said individual piecewise coefficient vector
    std::map<float, std::vector<float>> loadingPieceCoef; // The internal vector will have know amount of coefficients
    std::map<float, std::vector<float>> unloadingPieceCoef; // The internal vector will have know amount of coefficients
};






#endif
