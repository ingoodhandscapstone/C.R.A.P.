#ifndef FINGER_ABDUCTION_PROCESSOR_H
#define FINGER_ABDUCTION_PROCESSOR_H

#include "ProcessorConfigs.h"
#include "InEKF/Core"
#include "InEKF/Inertial"
#include "AccelGravityMeasureModel.h"
#include "QueueMessageTypes.h"

#include <cstdint>
#include <optional>




class FingerAbductionProcessor {


    static const int CALIBRATION_SAMPLE_COUNT;
    static const Eigen::Vector3d GRAVITY;

    static const float VELOCITY_VARIANCE;
    static const float POSITION_VARIANCE;

    ImuProcessingConfig * config;

    std::optional<InEKF::InEKF<InEKF::InertialProcess>> ekf;
    InEKF::InertialProcess pModel;
    AccelGravityMeasureModel mModel;

    float gyroBias;
    float accelBias;

    int sampleCalibrationCount;

    Eigen::Matrix3d initialOrientation;
    Eigen::Vector3d initialGyroBias;

    InEKF::SE3<2, 6> currentState;
    uint32_t currentTimestamp;

    void correctOrthoOfReading(Eigen::Vector3d& accel);


    public:

        FingerAbductionProcessor()
            : config(nullptr),
              ekf(std::nullopt),
              pModel(),
              mModel(),
              gyroBias(0.0f),
              accelBias(0.0f),
              sampleCalibrationCount(0),
              initialOrientation(Eigen::Matrix3d::Identity()),
              initialGyroBias(Eigen::Vector3d::Zero()),
              currentState(),
              currentTimestamp(0) {}

        bool initialize(ImuProcessingConfig * config);


        // Going to get initial position and gyro bias
        // Initial position is with respect to the global frame (which is going to be the hand imu with possibly some constant transformation)
        bool calibrate(Eigen::Vector3d& accels, Eigen::Vector3d& gyro); 

        bool reset();

        void setInitialTimestamp(uint32_t timestamp);

        // Sensor Processor Work has to know that predict is called and then update
        void predict(Eigen::Vector6d& u, uint32_t timestamp); // Takes in gyro and accel

        void update(Eigen::Vector3d& accel); // Takes in accelerometer reading
        
        // Only updates after predict() & update() called since last getAngle()
        void getAngle(float& angle, Eigen::Matrix3d handOrientation);

};






#endif
