#include "JointRomProcessor.h"


void JointRomProcessor::initialize(ResistiveSensorConfig * config){
    this->config = config;
}


bool JointRomProcessor::calibration(int& digVoltage){
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


void JointRomProcessor::applyMovingAverage(float& voltage){

    previousVoltages.push_front(voltage);

    float endOfMovingAvgSample = previousVoltages.back();

    previousVoltages.pop_back();

    movingAverageVoltage += ((voltage - endOfMovingAvgSample) / numberOfSamples);


}


void JointRomProcessor::voltageToAngle(float& angle, std::vector<float>& funcCoef){
    // 2 order poly assume highest order is first index
    // a * x^2 + bx + c
    angle = funcCoef.at(0) * std::pow(movingAverageVoltage, 2) + funcCoef.at(1) * movingAverageVoltage + funcCoef.at(2);
    
}


bool JointRomProcessor::getJointAngle(int& digVolt, float& angle){
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


    voltageToAngle(angle, curve);

    return true;

}

bool JointRomProcessor::getCalibrationAngle(float& angle) const{
    if(config == nullptr || config->loadingPieceCoef.size() < 3){
        return false;
    }

    angle = config->loadingPieceCoef.at(0) * std::pow(movingAverageVoltage, 2) +
            config->loadingPieceCoef.at(1) * movingAverageVoltage +
            config->loadingPieceCoef.at(2);
    return true;
}

bool JointRomProcessor::reset(){
    calibrationSampleCount = 0;
    movingAverageVoltage = 0.0f;
    inputVoltageOffset = 0.0f;
    previousVoltages.clear();
    return true;
}
