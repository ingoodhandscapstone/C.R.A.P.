#include "ForceProcessing.h"

void ForceProcessing::initialize(ResistiveSensorConfig * config){
    this->config = config;
}


bool ForceProcessing::calibration(int& digVoltage){
    if(config == nullptr){
        return false;
    }

    if(calibrationSampleCount > (numberOfSamples - 1)){ // Number of samples not indexing from 0, whereas calibrationSampleCount starts at 0
        return true;
    }

    float voltage;

    digitalToFloat(digVoltage, voltage);

    previousVoltages.push_front(voltage);

    movingAverageVoltage += (voltage / numberOfSamples);


    // Get voltage difference for input offset
    if(calibrationSampleCount == (numberOfSamples - 1)){
        inputVoltageOffset = config->calibratePositionVoltage - movingAverageVoltage;
    }

    calibrationSampleCount++;

    return calibrationSampleCount > (numberOfSamples - 1);

}


void ForceProcessing::applyMovingAverage(float& voltage){

    previousVoltages.push_front(voltage);

    float endOfMovingAvgSample = previousVoltages.back();

    previousVoltages.pop_back();

    movingAverageVoltage += ((voltage - endOfMovingAvgSample) / numberOfSamples);


}


void ForceProcessing::voltageToForce(float& force, std::vector<float>& funcCoef){
    // Assume first coef is highest order

    // I think will be linear, y = mx + b
    force = funcCoef.at(0) * movingAverageVoltage + funcCoef.at(1);
}


bool ForceProcessing::getForceOutput(int& digVolt, float& force){
    if(config == nullptr){
        return false;
    }

    float voltage;

    digitalToFloat(digVolt, voltage);

    applyMovingAverage(voltage);

    float voltDiff = voltage - movingAverageVoltage;

    if(isNoise(voltDiff) || !isGreaterThanDeadband(voltDiff)){
        return false;
    }

    std::vector<float>& curve = (voltDiff > 0) ? config->loadingPieceCoef : config->unloadingPieceCoef;


    voltageToForce(force, curve);

    return true;

}
