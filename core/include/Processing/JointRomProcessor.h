#ifndef JOINT_ROM_PROCESSOR_H
#define JOINT_ROM_H

#include "ProcessorConfigs.h"
#include <deque>

class JointRomProcessor {

    int numberOfSamples;

    int calibrationSampleCount;

    float movingAverageVoltage;
    float inputVoltageOffset;

    std::deque<float> previousVoltages;


    ResistiveSensorConfig config;

    void digitalToFloat(int& digVolt, float& voltage){voltage = digVolt * config.adcLSB;}
    void applyMovingAverage(float& voltage);
    bool isNoise(float voltageDif) {return abs(voltageDif) > (config.noiseFloor / std::sqrt(numberOfSamples));}
    bool isGreaterThanDeadband(float voltageDif){ return abs(voltageDif) > config.deadband;} // Only needed if greater than noise floor lsb in which case this becomes lsb
    void voltageToAngle(float& angle, std::vector<float>& funcCoef);

    public:

        JointRomProcessor();

        bool initialize(ResistiveSensorConfig& config, int numberOfSamples);


        bool getJointAngle(int& digVolt, float& angle);


        void setNumberOfSamples(int samples){numberOfSamples = samples;} // Used session to session and can be optimized

        bool calibration(int& digVoltage);


};












#endif 