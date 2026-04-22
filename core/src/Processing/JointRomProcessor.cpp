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
    // For now I am going to assume 3rd order poly for each piecewise
    // index(3)x^3 + index(2)x^2 + index(1)x + index(0)

    angle = funcCoef.at(3) * std::pow(movingAverageVoltage, 3) + funcCoef.at(2) * std::pow(movingAverageVoltage, 2)
                + funcCoef.at(1) * movingAverageVoltage + funcCoef.at(0);
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

    std::map<float, std::vector<float>>& curve = (voltDiff > 0) ? config->loadingPieceCoef : config->unloadingPieceCoef;


    auto it = curve.lower_bound(movingAverageVoltage + inputVoltageOffset);

    if(it == curve.end()){
        it = std::prev(it);
    } else if(it == curve.begin()){
        it = std::next(it);
    }

    voltageToAngle(angle, it->second);

    return true;

}
