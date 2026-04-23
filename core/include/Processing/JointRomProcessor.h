#ifndef JOINT_ROM_PROCESSOR_H
#define JOINT_ROM_PROCESSOR_H

#include "ProcessorConfigs.h"
#include <deque>
#include <math.h>
#include <vector>
#include <map>
#include <iterator>


class JointRomProcessor {

    int numberOfSamples;

    int calibrationSampleCount;

    float movingAverageVoltage;
    float inputVoltageOffset;

    std::deque<float> previousVoltages;


    ResistiveSensorConfig * config;

    void digitalToFloat(int& digVolt, float& voltage){
        if(config == nullptr){
            voltage = 0.0f;
            return;
        }
        voltage = digVolt * config->adcLSB;
    }
    void applyMovingAverage(float& voltage);
    bool isNoise(float voltageDif) {
        if(config == nullptr){
            return true;
        }
        return abs(voltageDif) < (config->noiseFloor / std::sqrt(numberOfSamples));
    }
    bool isGreaterThanDeadband(float voltageDif){
        if(config == nullptr){
            return false;
        }
        return abs(voltageDif) > config->deadband;
    } // Only needed if greater than noise floor lsb in which case this becomes lsb
    void voltageToAngle(float& angle, std::vector<float>& funcCoef);

    public:

        JointRomProcessor()  : numberOfSamples(1),
            calibrationSampleCount(0),
            movingAverageVoltage(0.0f),
            inputVoltageOffset(0.0f),
            previousVoltages(),
            config(nullptr){};


        void initialize(ResistiveSensorConfig * config);


        bool getJointAngle(int& digVolt, float& angle);


        void setNumberOfSamples(int samples){numberOfSamples = samples;} // Used session to session and can be optimized

        bool calibration(int& digVoltage);

        bool isCalibrated() {return calibrationSampleCount > (numberOfSamples - 1);}

        bool reset();


};












#endif 
