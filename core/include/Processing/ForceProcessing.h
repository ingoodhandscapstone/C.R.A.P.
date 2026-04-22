#ifndef FORCE_PROCESSING_H
#define FORCE_PROCESSING_H

#include "ProcessorConfigs.h"
#include <deque>
#include <math.h>
#include <vector>
#include <map>
#include <iterator>

class ForceProcessing {

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
    void voltageToForce(float& force, std::vector<float>& funcCoef);

    public:

        ForceProcessing()  : numberOfSamples(1),
            calibrationSampleCount(0),
            movingAverageVoltage(0.0f),
            inputVoltageOffset(0.0f),
            previousVoltages(),
            config(nullptr) {};


        void initialize(ResistiveSensorConfig * config);


        bool getForceOutput(int& digVolt, float& force);


        void setNumberOfSamples(int samples){numberOfSamples = samples;} // Used session to session and can be optimized

        bool calibration(int& digVoltage);

};

#endif
