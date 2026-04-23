#include "SensorProcessingLaneWorker.h"

#include <algorithm>
#include <cstring>

std::mutex SensorProcessingLaneWorker::calibrationEpochMutex;
uint32_t SensorProcessingLaneWorker::calibrationEpochCounter = 0;
uint32_t SensorProcessingLaneWorker::calibrationEpochJoinCount = 0;
uint32_t SensorProcessingLaneWorker::calibrationEpochRequiredCount = 0;
bool SensorProcessingLaneWorker::calibrationEpochOpen = false;
const std::chrono::milliseconds SensorProcessingLaneWorker::CALIBRATION_TIMEOUT_MS(30000);

void SensorProcessingLaneWorker::initialize(ProcessingGroup processingGroup,
                                            BloodOxygenProcessor * wristSPO2Processor,
                                            WristOrientationProcessor * handIMUProcessor,
                                            FingerAbductionProcessor * pointerIMUProcessor,
                                            FingerAbductionProcessor * middleIMUProcessor,
                                            FingerAbductionProcessor * thumbIMUProcessor,
                                            FingerAbductionProcessor * ringIMUProcessor,
                                            FingerAbductionProcessor * pinkyIMUProcessor,
                                            JointRomProcessor * pointerMCPFlexProcessor,
                                            JointRomProcessor * pointerPIPFlexProcessor,
                                            JointRomProcessor * pointerDIPFlexProcessor,
                                            JointRomProcessor * middleMCPFlexProcessor,
                                            JointRomProcessor * middlePIPFlexProcessor,
                                            JointRomProcessor * middleDIPFlexProcessor,
                                            JointRomProcessor * ringMCPFlexProcessor,
                                            JointRomProcessor * ringPIPFlexProcessor,
                                            JointRomProcessor * ringDIPFlexProcessor,
                                            JointRomProcessor * pinkyMCPFlexProcessor,
                                            JointRomProcessor * pinkyPIPFlexProcessor,
                                            JointRomProcessor * pinkyDIPFlexProcessor,
                                            JointRomProcessor * thumbMCPFlexProcessor,
                                            JointRomProcessor * thumbPIPFlexProcessor,
                                            ForceProcessing * pointerForceProcessor,
                                            ForceProcessing * middleForceProcessor,
                                            ForceProcessing * thumbForceProcessor,
                                            ForceProcessing * ringForceProcessor,
                                            ForceProcessing * pinkyForceProcessor,
                                            std::unordered_map<SensorID, ImuProcessingConfig> * imuConfigs,
                                            std::unordered_map<SensorID, ResistiveSensorConfig> * resistiveSensorConfigs,
                                            std::queue<DataOutputElement> * forwardMQTTQueue,
                                            std::queue<SessionCommand> * commandQueue,
                                            std::queue<DataToProcessorElement> * sensorDataQueue,
                                            std::queue<CalibrationStatusMessage> * calibrationStatusQueue,
                                            std::mutex * forwardMQTTMutex,
                                            std::mutex * commandMutex,
                                            std::mutex * sensorDataMutex,
                                            std::mutex * calibrationStatusMutex){
    this->processingGroup = processingGroup;
    this->wristSPO2Processor = wristSPO2Processor;
    this->handIMUProcessor = handIMUProcessor;
    this->pointerIMUProcessor = pointerIMUProcessor;
    this->middleIMUProcessor = middleIMUProcessor;
    this->thumbIMUProcessor = thumbIMUProcessor;
    this->ringIMUProcessor = ringIMUProcessor;
    this->pinkyIMUProcessor = pinkyIMUProcessor;
    this->pointerMCPFlexProcessor = pointerMCPFlexProcessor;
    this->pointerPIPFlexProcessor = pointerPIPFlexProcessor;
    this->pointerDIPFlexProcessor = pointerDIPFlexProcessor;
    this->middleMCPFlexProcessor = middleMCPFlexProcessor;
    this->middlePIPFlexProcessor = middlePIPFlexProcessor;
    this->middleDIPFlexProcessor = middleDIPFlexProcessor;
    this->ringMCPFlexProcessor = ringMCPFlexProcessor;
    this->ringPIPFlexProcessor = ringPIPFlexProcessor;
    this->ringDIPFlexProcessor = ringDIPFlexProcessor;
    this->pinkyMCPFlexProcessor = pinkyMCPFlexProcessor;
    this->pinkyPIPFlexProcessor = pinkyPIPFlexProcessor;
    this->pinkyDIPFlexProcessor = pinkyDIPFlexProcessor;
    this->thumbMCPFlexProcessor = thumbMCPFlexProcessor;
    this->thumbPIPFlexProcessor = thumbPIPFlexProcessor;
    this->pointerForceProcessor = pointerForceProcessor;
    this->middleForceProcessor = middleForceProcessor;
    this->thumbForceProcessor = thumbForceProcessor;
    this->ringForceProcessor = ringForceProcessor;
    this->pinkyForceProcessor = pinkyForceProcessor;
    this->imuConfigs = imuConfigs;
    this->resistiveSensorConfigs = resistiveSensorConfigs;
    this->forwardMQTTQueue = forwardMQTTQueue;
    this->commandQueue = commandQueue;
    this->sensorDataQueue = sensorDataQueue;
    this->calibrationStatusQueue = calibrationStatusQueue;
    this->forwardMQTTMutex = forwardMQTTMutex;
    this->commandMutex = commandMutex;
    this->sensorDataMutex = sensorDataMutex;
    this->calibrationStatusMutex = calibrationStatusMutex;

    mostRecentConfigCommand = SessionCommand::NONE;
    state = SensorProcessingState::IDLE;
    currentCalibrationEpoch = 0;
    calibrationStartTime = std::chrono::steady_clock::time_point::min();
    resetSessionSelections();
    resetImuCalibrationState();
    resetForceCalibrationState();

    if(imuConfigs != nullptr){
        if(handIMUProcessor != nullptr){
            handIMUProcessor->initialize(&(imuConfigs->at(SensorID::HAND_IMU)));
        }
        if(pointerIMUProcessor != nullptr){
            pointerIMUProcessor->initialize(&(imuConfigs->at(SensorID::POINTER_IMU)));
        }
        if(middleIMUProcessor != nullptr){
            middleIMUProcessor->initialize(&(imuConfigs->at(SensorID::MIDDLE_IMU)));
        }
        if(thumbIMUProcessor != nullptr){
            thumbIMUProcessor->initialize(&(imuConfigs->at(SensorID::THUMB_IMU)));
        }
        if(ringIMUProcessor != nullptr){
            ringIMUProcessor->initialize(&(imuConfigs->at(SensorID::RING_IMU)));
        }
        if(pinkyIMUProcessor != nullptr){
            pinkyIMUProcessor->initialize(&(imuConfigs->at(SensorID::PINKY_IMU)));
        }
    }

    if(resistiveSensorConfigs != nullptr){
        if(pointerMCPFlexProcessor != nullptr){
            pointerMCPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::POINTER_MCP_FLEX)));
        }
        if(pointerPIPFlexProcessor != nullptr){
            pointerPIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::POINTER_PIP_FLEX)));
        }
        if(pointerDIPFlexProcessor != nullptr){
            pointerDIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::POINTER_DIP_FLEX)));
        }
        if(middleMCPFlexProcessor != nullptr){
            middleMCPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::MIDDLE_MCP_FLEX)));
        }
        if(middlePIPFlexProcessor != nullptr){
            middlePIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::MIDDLE_PIP_FLEX)));
        }
        if(middleDIPFlexProcessor != nullptr){
            middleDIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::MIDDLE_DIP_FLEX)));
        }
        if(ringMCPFlexProcessor != nullptr){
            ringMCPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::RING_MCP_FLEX)));
        }
        if(ringPIPFlexProcessor != nullptr){
            ringPIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::RING_PIP_FLEX)));
        }
        if(ringDIPFlexProcessor != nullptr){
            ringDIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::RING_DIP_FLEX)));
        }
        if(pinkyMCPFlexProcessor != nullptr){
            pinkyMCPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::PINKY_MCP_FLEX)));
        }
        if(pinkyPIPFlexProcessor != nullptr){
            pinkyPIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::PINKY_PIP_FLEX)));
        }
        if(pinkyDIPFlexProcessor != nullptr){
            pinkyDIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::PINKY_DIP_FLEX)));
        }
        if(thumbMCPFlexProcessor != nullptr){
            thumbMCPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::THUMB_MCP_FLEX)));
        }
        if(thumbPIPFlexProcessor != nullptr){
            thumbPIPFlexProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::THUMB_PIP_FLEX)));
        }

        if(pointerForceProcessor != nullptr){
            pointerForceProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::POINTER_FORCE)));
        }
        if(middleForceProcessor != nullptr){
            middleForceProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::MIDDLE_FORCE)));
        }
        if(thumbForceProcessor != nullptr){
            thumbForceProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::THUMB_FORCE)));
        }
        if(ringForceProcessor != nullptr){
            ringForceProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::RING_FORCE)));
        }
        if(pinkyForceProcessor != nullptr){
            pinkyForceProcessor->initialize(&(resistiveSensorConfigs->at(SensorID::PINKY_FORCE)));
        }
    }
}

bool SensorProcessingLaneWorker::isSessionConfigCommand(SessionCommand command){
    switch(command){
        case SessionCommand::SESSION_CONFIG_WRIST:
        case SessionCommand::SESSION_CONFIG_POINTER:
        case SessionCommand::SESSION_CONFIG_SPO2:
        case SessionCommand::SESSION_CONFIG_MIDDLE:
        case SessionCommand::SESSION_CONFIG_RING:
        case SessionCommand::SESSION_CONFIG_PINKY:
        case SessionCommand::SESSION_CONFIG_THUMB:
        case SessionCommand::SESSION_CONFIG_POINTER_MIDDLE:
        case SessionCommand::SESSION_CONFIG_POINTER_WRIST:
        case SessionCommand::SESSION_CONFIG_GRIPPER_POINTER:
        case SessionCommand::SESSION_CONFIG_GRIPPER_MIDDLE:
        case SessionCommand::SESSION_CONFIG_GRIPPER_RING:
        case SessionCommand::SESSION_CONFIG_GRIPPER_PINKY:
        case SessionCommand::SESSION_CONFIG_GRIPPER_THUMB:
        case SessionCommand::SESSION_CONFIG_GRIPPER_POINTER_MIDDLE:
        case SessionCommand::SESSION_CONFIG_GRIPPER_ALL:
            return true;
        default:
            return false;
    }
}

bool SensorProcessingLaneWorker::usesFlexSpo2ForCommand(SessionCommand command){
    switch(command){
        case SessionCommand::SESSION_CONFIG_POINTER:
        case SessionCommand::SESSION_CONFIG_SPO2:
        case SessionCommand::SESSION_CONFIG_MIDDLE:
        case SessionCommand::SESSION_CONFIG_RING:
        case SessionCommand::SESSION_CONFIG_PINKY:
        case SessionCommand::SESSION_CONFIG_THUMB:
        case SessionCommand::SESSION_CONFIG_POINTER_MIDDLE:
        case SessionCommand::SESSION_CONFIG_POINTER_WRIST:
            return true;
        default:
            return false;
    }
}

bool SensorProcessingLaneWorker::usesImuForceForCommand(SessionCommand command){
    switch(command){
        case SessionCommand::SESSION_CONFIG_WRIST:
        case SessionCommand::SESSION_CONFIG_POINTER:
        case SessionCommand::SESSION_CONFIG_MIDDLE:
        case SessionCommand::SESSION_CONFIG_RING:
        case SessionCommand::SESSION_CONFIG_PINKY:
        case SessionCommand::SESSION_CONFIG_THUMB:
        case SessionCommand::SESSION_CONFIG_POINTER_MIDDLE:
        case SessionCommand::SESSION_CONFIG_POINTER_WRIST:
        case SessionCommand::SESSION_CONFIG_GRIPPER_POINTER:
        case SessionCommand::SESSION_CONFIG_GRIPPER_MIDDLE:
        case SessionCommand::SESSION_CONFIG_GRIPPER_RING:
        case SessionCommand::SESSION_CONFIG_GRIPPER_PINKY:
        case SessionCommand::SESSION_CONFIG_GRIPPER_THUMB:
        case SessionCommand::SESSION_CONFIG_GRIPPER_POINTER_MIDDLE:
        case SessionCommand::SESSION_CONFIG_GRIPPER_ALL:
            return true;
        default:
            return false;
    }
}

bool SensorProcessingLaneWorker::isRelevantConfigCommand(SessionCommand command){
    if(processingGroup == ProcessingGroup::FLEX_SPO2){
        return usesFlexSpo2ForCommand(command);
    }

    return usesImuForceForCommand(command);
}

uint32_t SensorProcessingLaneWorker::getRequiredCalibrationCount(SessionCommand command){
    const bool usesFlexSpo2 = usesFlexSpo2ForCommand(command);
    const bool usesImuForce = usesImuForceForCommand(command);

    if(!usesFlexSpo2 && !usesImuForce){
        return 0;
    }

    if(usesFlexSpo2 && usesImuForce){
        return 2;
    }

    return 1;
}

uint32_t SensorProcessingLaneWorker::acquireCalibrationEpoch(uint32_t requiredCount){
    std::lock_guard guard(calibrationEpochMutex);
    if(requiredCount <= 1){
        calibrationEpochCounter++;
        return calibrationEpochCounter;
    }

    if(!calibrationEpochOpen || calibrationEpochJoinCount >= calibrationEpochRequiredCount){
        calibrationEpochCounter++;
        calibrationEpochOpen = true;
        calibrationEpochRequiredCount = requiredCount;
        calibrationEpochJoinCount = 1;
        return calibrationEpochCounter;
    }

    calibrationEpochJoinCount++;
    if(calibrationEpochJoinCount >= calibrationEpochRequiredCount){
        calibrationEpochOpen = false;
    }

    return calibrationEpochCounter;
}

bool SensorProcessingLaneWorker::readFrontCommand(SessionCommand& command){
    std::lock_guard guard(*commandMutex);
    if(commandQueue->empty()){
        return false;
    }

    command = commandQueue->front();
    return true;
}

void SensorProcessingLaneWorker::popFrontCommand(){
    std::lock_guard guard(*commandMutex);
    if(!commandQueue->empty()){
        commandQueue->pop();
    }
}

void SensorProcessingLaneWorker::clearSensorDataQueue(){
    std::lock_guard guard(*sensorDataMutex);
    while(!sensorDataQueue->empty()){
        sensorDataQueue->pop();
    }
}

bool SensorProcessingLaneWorker::runCalibration(){
    if(calibrationFunc == nullptr){
        return false;
    }
    return (this->*calibrationFunc)();
}

void SensorProcessingLaneWorker::runSession(){
    if(sessionFunc == nullptr){
        return;
    }
    (this->*sessionFunc)();
}

void SensorProcessingLaneWorker::runReset(){
    if(resetFunc == nullptr){
        if(processingGroup == ProcessingGroup::FLEX_SPO2){
            resetFlexSPO2Data();
        } else {
            resetImuForceData();
        }
        return;
    }
    (this->*resetFunc)();
}

void SensorProcessingLaneWorker::resetProcessingData(){
    runReset();
}

void SensorProcessingLaneWorker::run(std::stop_token stopToken){
    while(!stopToken.stop_requested()){
        switch(state){
            case SensorProcessingState::IDLE: {
                SessionCommand command = SessionCommand::NONE;
                if(!readFrontCommand(command)){
                    break;
                }

                if(isSessionConfigCommand(command) && isRelevantConfigCommand(command)){
                    mostRecentConfigCommand = command;
                    bool success = false;
                    if(processingGroup == ProcessingGroup::FLEX_SPO2){
                        success = setConfigFunctionFlexSpo2();
                    } else {
                        success = setConfigFunctionImuForce();
                    }

                    if(!success){
                        return;
                    }

                    state = SensorProcessingState::CONFIGURED;
                }

                // Pop commands in IDLE when they are not relevant to this worker.
                popFrontCommand();
                break;
            }

            case SensorProcessingState::CONFIGURED: {
                SessionCommand command = SessionCommand::NONE;
                if(!readFrontCommand(command)){
                    break;
                }

                if(command == SessionCommand::CALIBRATE_SESSION){
                    const uint32_t requiredCount = getRequiredCalibrationCount(mostRecentConfigCommand);
                    currentCalibrationEpoch = acquireCalibrationEpoch(requiredCount);
                    calibrationStartTime = std::chrono::steady_clock::now();
                    state = SensorProcessingState::CALIBRATING;
                    popFrontCommand();
                }
                break;
            }

            case SensorProcessingState::CALIBRATING: {
                if(calibrationStartTime != std::chrono::steady_clock::time_point::min()){
                    const auto elapsed = std::chrono::steady_clock::now() - calibrationStartTime;
                    if(elapsed >= CALIBRATION_TIMEOUT_MS){
                        return;
                    }
                }

                if(runCalibration()){
                    CalibrationStatusMessage message;
                    message.epoch = currentCalibrationEpoch;
                    message.requiredCount = getRequiredCalibrationCount(mostRecentConfigCommand);
                    {
                        std::lock_guard guard(*calibrationStatusMutex);
                        calibrationStatusQueue->push(message);
                    }
                    calibrationStartTime = std::chrono::steady_clock::time_point::min();
                    state = SensorProcessingState::WAITING_TO_RUN;
                }
                break;
            }

            case SensorProcessingState::WAITING_TO_RUN: {
                SessionCommand command = SessionCommand::NONE;
                if(!readFrontCommand(command)){
                    break;
                }

                clearSensorDataQueue();

                if(command == SessionCommand::SESSION_START){
                    state = SensorProcessingState::RUNNING;
                    popFrontCommand();
                }
                break;
            }

            case SensorProcessingState::RUNNING: {
                SessionCommand command = SessionCommand::NONE;
                if(readFrontCommand(command) && command == SessionCommand::SESSION_STOP){
                    clearSensorDataQueue();
                    state = SensorProcessingState::RESETTING;
                    popFrontCommand();
                } else {
                    runSession();
                }
                break;
            }

            case SensorProcessingState::RESETTING: {
                resetProcessingData();
                mostRecentConfigCommand = SessionCommand::NONE;
                currentCalibrationEpoch = 0;
                calibrationStartTime = std::chrono::steady_clock::time_point::min();
                calibrationFunc = nullptr;
                sessionFunc = nullptr;
                resetFunc = nullptr;
                resetSessionSelections();
                state = SensorProcessingState::IDLE;
                break;
            }

            default:
                break;
        }
    }
}

void SensorProcessingLaneWorker::resetSessionSelections(){
    activeFlexSensors.clear();
    activeFingerImuSensors.clear();
    activeForceSensors.clear();
    activeSpo2Session = false;
    activeWristOrientationOutput = false;
}

void SensorProcessingLaneWorker::resetImuCalibrationState(){
    handImuCalibrationState = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false};
    pointerImuCalibrationState = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false};
    middleImuCalibrationState = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false};
    thumbImuCalibrationState = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false};
    ringImuCalibrationState = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false};
    pinkyImuCalibrationState = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), false, false, false};
    handHasUpdatedInSession = false;
}

void SensorProcessingLaneWorker::resetForceCalibrationState(){
    pointerForceCalibrationDone = false;
    middleForceCalibrationDone = false;
    thumbForceCalibrationDone = false;
    ringForceCalibrationDone = false;
    pinkyForceCalibrationDone = false;
}

void SensorProcessingLaneWorker::resetFlexSPO2Data(){
    clearSensorDataQueue();
}

void SensorProcessingLaneWorker::resetImuForceData(){
    clearSensorDataQueue();
    resetImuCalibrationState();
    resetForceCalibrationState();
}

void SensorProcessingLaneWorker::resetFlexSPO2ConfigData(){
    clearSensorDataQueue();

    if(activeSpo2Session && wristSPO2Processor != nullptr){
        wristSPO2Processor->reset();
    }

    for(const SensorID id : activeFlexSensors){
        JointRomProcessor * processor = findFlexSensorProcessor(id);
        if(processor != nullptr){
            processor->reset();
        }
    }
}

void SensorProcessingLaneWorker::resetImuForceConfigData(){
    clearSensorDataQueue();
    resetImuCalibrationState();
    resetForceCalibrationState();

    if(isHandImuActive() && handIMUProcessor != nullptr){
        handIMUProcessor->reset();
    }

    for(const SensorID id : activeFingerImuSensors){
        FingerAbductionProcessor * processor = findFingerImuProcessor(id);
        if(processor != nullptr){
            processor->reset();
        }
    }

    for(const SensorID id : activeForceSensors){
        ForceProcessing * processor = findForceProcessor(id);
        if(processor != nullptr){
            processor->reset();
        }
    }
}

void SensorProcessingLaneWorker::configureFlexSession(SessionCommand command){
    switch(command){
        case SessionCommand::SESSION_CONFIG_POINTER:
        case SessionCommand::SESSION_CONFIG_POINTER_WRIST:
            activeFlexSensors = {SensorID::POINTER_MCP_FLEX,
                                 SensorID::POINTER_PIP_FLEX,
                                 SensorID::POINTER_DIP_FLEX};
            break;

        case SessionCommand::SESSION_CONFIG_SPO2:
            activeSpo2Session = true;
            break;

        case SessionCommand::SESSION_CONFIG_MIDDLE:
            activeFlexSensors = {SensorID::MIDDLE_MCP_FLEX,
                                 SensorID::MIDDLE_PIP_FLEX,
                                 SensorID::MIDDLE_DIP_FLEX};
            break;

        case SessionCommand::SESSION_CONFIG_RING:
            activeFlexSensors = {SensorID::RING_MCP_FLEX,
                                 SensorID::RING_PIP_FLEX,
                                 SensorID::RING_DIP_FLEX};
            break;

        case SessionCommand::SESSION_CONFIG_PINKY:
            activeFlexSensors = {SensorID::PINKY_MCP_FLEX,
                                 SensorID::PINKY_PIP_FLEX,
                                 SensorID::PINKY_DIP_FLEX};
            break;

        case SessionCommand::SESSION_CONFIG_THUMB:
            activeFlexSensors = {SensorID::THUMB_MCP_FLEX,
                                 SensorID::THUMB_PIP_FLEX};
            break;

        case SessionCommand::SESSION_CONFIG_POINTER_MIDDLE:
            activeFlexSensors = {SensorID::POINTER_MCP_FLEX,
                                 SensorID::POINTER_PIP_FLEX,
                                 SensorID::POINTER_DIP_FLEX,
                                 SensorID::MIDDLE_MCP_FLEX,
                                 SensorID::MIDDLE_PIP_FLEX,
                                 SensorID::MIDDLE_DIP_FLEX};
            break;

        default:
            break;
    }
}

void SensorProcessingLaneWorker::configureImuSession(SessionCommand command){
    switch(command){
        case SessionCommand::SESSION_CONFIG_WRIST:
            activeWristOrientationOutput = true;
            break;

        case SessionCommand::SESSION_CONFIG_POINTER:
            activeFingerImuSensors = {SensorID::POINTER_IMU};
            break;

        case SessionCommand::SESSION_CONFIG_MIDDLE:
            activeFingerImuSensors = {SensorID::MIDDLE_IMU};
            break;

        case SessionCommand::SESSION_CONFIG_RING:
            activeFingerImuSensors = {SensorID::RING_IMU};
            break;

        case SessionCommand::SESSION_CONFIG_PINKY:
            activeFingerImuSensors = {SensorID::PINKY_IMU};
            break;

        case SessionCommand::SESSION_CONFIG_THUMB:
            activeFingerImuSensors = {SensorID::THUMB_IMU};
            break;

        case SessionCommand::SESSION_CONFIG_POINTER_MIDDLE:
            activeFingerImuSensors = {SensorID::POINTER_IMU,
                                      SensorID::MIDDLE_IMU};
            break;

        case SessionCommand::SESSION_CONFIG_POINTER_WRIST:
            activeFingerImuSensors = {SensorID::POINTER_IMU};
            activeWristOrientationOutput = true;
            break;

        case SessionCommand::SESSION_CONFIG_GRIPPER_POINTER:
            activeForceSensors = {SensorID::POINTER_FORCE};
            break;

        case SessionCommand::SESSION_CONFIG_GRIPPER_MIDDLE:
            activeForceSensors = {SensorID::MIDDLE_FORCE};
            break;

        case SessionCommand::SESSION_CONFIG_GRIPPER_RING:
            activeForceSensors = {SensorID::RING_FORCE};
            break;

        case SessionCommand::SESSION_CONFIG_GRIPPER_PINKY:
            activeForceSensors = {SensorID::PINKY_FORCE};
            break;

        case SessionCommand::SESSION_CONFIG_GRIPPER_THUMB:
            activeForceSensors = {SensorID::THUMB_FORCE};
            break;

        case SessionCommand::SESSION_CONFIG_GRIPPER_POINTER_MIDDLE:
            activeForceSensors = {SensorID::POINTER_FORCE,
                                  SensorID::MIDDLE_FORCE};
            break;

        case SessionCommand::SESSION_CONFIG_GRIPPER_ALL:
            activeForceSensors = {SensorID::POINTER_FORCE,
                                  SensorID::MIDDLE_FORCE,
                                  SensorID::THUMB_FORCE,
                                  SensorID::RING_FORCE,
                                  SensorID::PINKY_FORCE};
            break;

        default:
            break;
    }
}

bool SensorProcessingLaneWorker::setConfigFunctionFlexSpo2(){
    resetSessionSelections();
    calibrationFunc = nullptr;
    sessionFunc = nullptr;
    resetFunc = nullptr;

    configureFlexSession(mostRecentConfigCommand);

    if(!usesFlexSpo2ForCommand(mostRecentConfigCommand)){
        return false;
    }

    if(activeSpo2Session){
        calibrationFunc = &SensorProcessingLaneWorker::calibrateSpo2Session;
        sessionFunc = &SensorProcessingLaneWorker::runSpo2Session;
        resetFunc = &SensorProcessingLaneWorker::resetFlexSPO2ConfigData;
        return true;
    }

    if(!activeFlexSensors.empty()){
        calibrationFunc = &SensorProcessingLaneWorker::calibrateFlexSession;
        sessionFunc = &SensorProcessingLaneWorker::runFlexSession;
        resetFunc = &SensorProcessingLaneWorker::resetFlexSPO2ConfigData;
        return true;
    }

    return false;
}

bool SensorProcessingLaneWorker::setConfigFunctionImuForce(){
    resetSessionSelections();
    resetImuCalibrationState();
    resetForceCalibrationState();
    calibrationFunc = nullptr;
    sessionFunc = nullptr;
    resetFunc = nullptr;

    configureImuSession(mostRecentConfigCommand);

    if(!usesImuForceForCommand(mostRecentConfigCommand)){
        return false;
    }

    if(!activeForceSensors.empty()){
        calibrationFunc = &SensorProcessingLaneWorker::calibrateForceSession;
        sessionFunc = &SensorProcessingLaneWorker::runForceSession;
        resetFunc = &SensorProcessingLaneWorker::resetImuForceConfigData;
        return true;
    }

    if(!activeFingerImuSensors.empty() || activeWristOrientationOutput){
        calibrationFunc = &SensorProcessingLaneWorker::calibrateImuSession;
        sessionFunc = &SensorProcessingLaneWorker::runImuSession;
        resetFunc = &SensorProcessingLaneWorker::resetImuForceConfigData;
        return true;
    }

    return false;
}

JointRomProcessor * SensorProcessingLaneWorker::findFlexSensorProcessor(const SensorID& id){
    switch(id){
        case SensorID::POINTER_MCP_FLEX:
            return pointerMCPFlexProcessor;
        case SensorID::POINTER_PIP_FLEX:
            return pointerPIPFlexProcessor;
        case SensorID::POINTER_DIP_FLEX:
            return pointerDIPFlexProcessor;
        case SensorID::MIDDLE_MCP_FLEX:
            return middleMCPFlexProcessor;
        case SensorID::MIDDLE_PIP_FLEX:
            return middlePIPFlexProcessor;
        case SensorID::MIDDLE_DIP_FLEX:
            return middleDIPFlexProcessor;
        case SensorID::RING_MCP_FLEX:
            return ringMCPFlexProcessor;
        case SensorID::RING_PIP_FLEX:
            return ringPIPFlexProcessor;
        case SensorID::RING_DIP_FLEX:
            return ringDIPFlexProcessor;
        case SensorID::PINKY_MCP_FLEX:
            return pinkyMCPFlexProcessor;
        case SensorID::PINKY_PIP_FLEX:
            return pinkyPIPFlexProcessor;
        case SensorID::PINKY_DIP_FLEX:
            return pinkyDIPFlexProcessor;
        case SensorID::THUMB_MCP_FLEX:
            return thumbMCPFlexProcessor;
        case SensorID::THUMB_PIP_FLEX:
            return thumbPIPFlexProcessor;
        default:
            return nullptr;
    }
}

FingerAbductionProcessor * SensorProcessingLaneWorker::findFingerImuProcessor(const SensorID& id){
    switch(id){
        case SensorID::POINTER_IMU:
            return pointerIMUProcessor;
        case SensorID::MIDDLE_IMU:
            return middleIMUProcessor;
        case SensorID::THUMB_IMU:
            return thumbIMUProcessor;
        case SensorID::RING_IMU:
            return ringIMUProcessor;
        case SensorID::PINKY_IMU:
            return pinkyIMUProcessor;
        default:
            return nullptr;
    }
}

ForceProcessing * SensorProcessingLaneWorker::findForceProcessor(const SensorID& id){
    switch(id){
        case SensorID::POINTER_FORCE:
            return pointerForceProcessor;
        case SensorID::MIDDLE_FORCE:
            return middleForceProcessor;
        case SensorID::THUMB_FORCE:
            return thumbForceProcessor;
        case SensorID::RING_FORCE:
            return ringForceProcessor;
        case SensorID::PINKY_FORCE:
            return pinkyForceProcessor;
        default:
            return nullptr;
    }
}

SensorProcessingLaneWorker::ImuCalibrationState *
SensorProcessingLaneWorker::findImuCalibrationState(const SensorID& id){
    switch(id){
        case SensorID::HAND_IMU:
            return &handImuCalibrationState;
        case SensorID::POINTER_IMU:
            return &pointerImuCalibrationState;
        case SensorID::MIDDLE_IMU:
            return &middleImuCalibrationState;
        case SensorID::THUMB_IMU:
            return &thumbImuCalibrationState;
        case SensorID::RING_IMU:
            return &ringImuCalibrationState;
        case SensorID::PINKY_IMU:
            return &pinkyImuCalibrationState;
        default:
            return nullptr;
    }
}

bool SensorProcessingLaneWorker::isActiveFlexSensor(const SensorID& id){
    return std::find(activeFlexSensors.begin(), activeFlexSensors.end(), id) != activeFlexSensors.end();
}

bool SensorProcessingLaneWorker::isActiveFingerImuSensor(const SensorID& id){
    return std::find(activeFingerImuSensors.begin(), activeFingerImuSensors.end(), id) != activeFingerImuSensors.end();
}

bool SensorProcessingLaneWorker::isActiveForceSensor(const SensorID& id){
    return std::find(activeForceSensors.begin(), activeForceSensors.end(), id) != activeForceSensors.end();
}

bool SensorProcessingLaneWorker::isHandImuActive(){
    return !activeFingerImuSensors.empty() || activeWristOrientationOutput;
}

bool SensorProcessingLaneWorker::areActiveFlexSensorsCalibrated(){
    if(activeFlexSensors.empty()){
        return false;
    }

    for(const SensorID id : activeFlexSensors){
        JointRomProcessor * processor = findFlexSensorProcessor(id);
        if(processor == nullptr || !processor->isCalibrated()){
            return false;
        }
    }

    return true;
}

bool SensorProcessingLaneWorker::areActiveImuSensorsCalibrated(){
    if(isHandImuActive()){
        if(handIMUProcessor == nullptr || !handImuCalibrationState.done){
            return false;
        }
    }

    for(const SensorID id : activeFingerImuSensors){
        FingerAbductionProcessor * processor = findFingerImuProcessor(id);
        ImuCalibrationState * calState = findImuCalibrationState(id);
        if(processor == nullptr || calState == nullptr || !calState->done){
            return false;
        }
    }

    return isHandImuActive() || !activeFingerImuSensors.empty();
}

bool SensorProcessingLaneWorker::areActiveForceSensorsCalibrated(){
    if(activeForceSensors.empty()){
        return false;
    }

    for(const SensorID id : activeForceSensors){
        bool calibrated = false;
        switch(id){
            case SensorID::POINTER_FORCE:
                calibrated = pointerForceCalibrationDone;
                break;
            case SensorID::MIDDLE_FORCE:
                calibrated = middleForceCalibrationDone;
                break;
            case SensorID::THUMB_FORCE:
                calibrated = thumbForceCalibrationDone;
                break;
            case SensorID::RING_FORCE:
                calibrated = ringForceCalibrationDone;
                break;
            case SensorID::PINKY_FORCE:
                calibrated = pinkyForceCalibrationDone;
                break;
            default:
                calibrated = false;
                break;
        }

        if(!calibrated){
            return false;
        }
    }

    return true;
}

bool SensorProcessingLaneWorker::calibrateFlexSession(){
    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return false;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if(elem.type != SensorType::FLEX || !isActiveFlexSensor(elem.id)){
        return false;
    }

    JointRomProcessor * processor = findFlexSensorProcessor(elem.id);
    if(processor == nullptr){
        return false;
    }

    int digVoltage = 0;
    convertToFlexInputData(digVoltage, elem);
    processor->calibration(digVoltage);

    return areActiveFlexSensorsCalibrated();
}

bool SensorProcessingLaneWorker::calibrateSpo2Session(){
    return true;
}

bool SensorProcessingLaneWorker::calibrateImuSession(){
    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return false;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if((elem.type != SensorType::IMU_ACCEL && elem.type != SensorType::IMU_GYRO) ||
       (elem.id != SensorID::HAND_IMU && !isActiveFingerImuSensor(elem.id))){
        return false;
    }

    Eigen::Vector3d accels;
    Eigen::Vector3d gyro;
    convertToImuInputData(accels, gyro, elem);

    ImuCalibrationState * calibrationState = findImuCalibrationState(elem.id);
    if(calibrationState == nullptr){
        return false;
    }

    if(elem.type == SensorType::IMU_ACCEL){
        calibrationState->accel = accels;
        calibrationState->hasAccel = true;
    } else {
        calibrationState->gyro = gyro;
        calibrationState->hasGyro = true;
    }

    if(calibrationState->hasAccel && calibrationState->hasGyro){
        bool done = false;
        if(elem.id == SensorID::HAND_IMU){
            if(handIMUProcessor == nullptr){
                return false;
            }
            done = handIMUProcessor->calibrate(calibrationState->accel, calibrationState->gyro);
        } else {
            FingerAbductionProcessor * processor = findFingerImuProcessor(elem.id);
            if(processor == nullptr){
                return false;
            }
            done = processor->calibrate(calibrationState->accel, calibrationState->gyro);
        }

        calibrationState->done = calibrationState->done || done;
        calibrationState->hasAccel = false;
        calibrationState->hasGyro = false;
    }

    return areActiveImuSensorsCalibrated();
}

bool SensorProcessingLaneWorker::calibrateForceSession(){
    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return false;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if(elem.type != SensorType::FORCE || !isActiveForceSensor(elem.id)){
        return false;
    }

    ForceProcessing * processor = findForceProcessor(elem.id);
    if(processor == nullptr){
        return false;
    }

    int digVoltage = 0;
    convertToFlexInputData(digVoltage, elem);
    const bool done = processor->calibration(digVoltage);

    switch(elem.id){
        case SensorID::POINTER_FORCE:
            pointerForceCalibrationDone = pointerForceCalibrationDone || done;
            break;
        case SensorID::MIDDLE_FORCE:
            middleForceCalibrationDone = middleForceCalibrationDone || done;
            break;
        case SensorID::THUMB_FORCE:
            thumbForceCalibrationDone = thumbForceCalibrationDone || done;
            break;
        case SensorID::RING_FORCE:
            ringForceCalibrationDone = ringForceCalibrationDone || done;
            break;
        case SensorID::PINKY_FORCE:
            pinkyForceCalibrationDone = pinkyForceCalibrationDone || done;
            break;
        default:
            return false;
    }

    return areActiveForceSensorsCalibrated();
}

void SensorProcessingLaneWorker::runFlexSession(){
    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if(elem.type != SensorType::FLEX || !isActiveFlexSensor(elem.id)){
        return;
    }

    JointRomProcessor * processor = findFlexSensorProcessor(elem.id);
    if(processor == nullptr){
        return;
    }

    int digVoltage = 0;
    convertToFlexInputData(digVoltage, elem);

    float angle = 0.0f;
    if(!processor->getJointAngle(digVoltage, angle)){
        return;
    }

    DataOutputElement outputElem;
    outputElem.id = elem.id;
    outputElem.data = std::to_string(angle);

    {
        std::lock_guard guard(*forwardMQTTMutex);
        forwardMQTTQueue->push(outputElem);
    }
}

void SensorProcessingLaneWorker::runSpo2Session(){
    if(!activeSpo2Session || wristSPO2Processor == nullptr){
        return;
    }

    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if(elem.type != SensorType::SPO2 || elem.id != SensorID::WRIST_SPO2){
        return;
    }

    int ir = 0;
    int red = 0;
    convertToSpo2InputData(ir, red, elem);

    int spo2 = 0;
    if(!wristSPO2Processor->getSPO2(spo2, ir, red)){
        return;
    }

    DataOutputElement outputElem;
    outputElem.id = SensorID::WRIST_SPO2;
    outputElem.data = std::to_string(spo2);
    {
        std::lock_guard guard(*forwardMQTTMutex);
        forwardMQTTQueue->push(outputElem);
    }
}

void SensorProcessingLaneWorker::runImuSession(){
    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if((elem.type != SensorType::IMU_ACCEL && elem.type != SensorType::IMU_GYRO) ||
       (elem.id != SensorID::HAND_IMU && !isActiveFingerImuSensor(elem.id))){
        return;
    }

    Eigen::Vector3d accels;
    Eigen::Vector3d gyro;
    convertToImuInputData(accels, gyro, elem);

    if(elem.id == SensorID::HAND_IMU){
        if(handIMUProcessor == nullptr){
            return;
        }

        if(elem.type == SensorType::IMU_ACCEL){
            handIMUProcessor->setAccel(accels, elem.timestamp);
        } else {
            handIMUProcessor->setGyro(gyro, elem.timestamp);
        }
    } else {
        FingerAbductionProcessor * processor = findFingerImuProcessor(elem.id);
        if(processor == nullptr){
            return;
        }

        if(elem.type == SensorType::IMU_ACCEL){
            processor->setAccel(accels, elem.timestamp);
        } else {
            processor->setGyro(gyro, elem.timestamp);
        }
    }

    bool handUpdated = false;
    if(isHandImuActive() && handIMUProcessor != nullptr && handIMUProcessor->hasGyroAndAccel()){
        handIMUProcessor->predict();
        handIMUProcessor->update();
        handHasUpdatedInSession = true;
        handUpdated = true;
    }

    std::vector<SensorID> updatedFingerOutputs;
    for(const SensorID fingerId : activeFingerImuSensors){
        FingerAbductionProcessor * processor = findFingerImuProcessor(fingerId);
        if(processor == nullptr || !processor->hasGyroAndAccel()){
            continue;
        }

        processor->predict();
        processor->update();
        updatedFingerOutputs.push_back(fingerId);
    }

    if(handHasUpdatedInSession && handIMUProcessor != nullptr){
        const Eigen::Matrix3d handOrientation = handIMUProcessor->getHandOrientationMatrix();
        for(const SensorID fingerId : updatedFingerOutputs){
            FingerAbductionProcessor * processor = findFingerImuProcessor(fingerId);
            if(processor == nullptr){
                continue;
            }

            float angle = 0.0f;
            processor->getAngle(angle, handOrientation);

            DataOutputElement outputElem;
            outputElem.id = fingerId;
            outputElem.data = std::to_string(angle);

            std::lock_guard guard(*forwardMQTTMutex);
            forwardMQTTQueue->push(outputElem);
        }
    }

    if(activeWristOrientationOutput && handUpdated && handIMUProcessor != nullptr){
        float xAngle = 0.0f;
        float yAngle = 0.0f;
        handIMUProcessor->getXAxisAngle(xAngle);
        handIMUProcessor->getYAxisAngle(yAngle);

        DataOutputElement xOutput;
        xOutput.id = SensorID::HAND_IMU;
        xOutput.data = std::to_string(xAngle);

        DataOutputElement yOutput;
        yOutput.id = SensorID::HAND_IMU_Y;
        yOutput.data = std::to_string(yAngle);

        std::lock_guard guard(*forwardMQTTMutex);
        forwardMQTTQueue->push(xOutput);
        forwardMQTTQueue->push(yOutput);
    }
}

void SensorProcessingLaneWorker::runForceSession(){
    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if(elem.type != SensorType::FORCE || !isActiveForceSensor(elem.id)){
        return;
    }

    ForceProcessing * processor = findForceProcessor(elem.id);
    if(processor == nullptr){
        return;
    }

    int digVoltage = 0;
    convertToFlexInputData(digVoltage, elem);

    float force = 0.0f;
    if(!processor->getForceOutput(digVoltage, force)){
        return;
    }

    DataOutputElement outputElem;
    outputElem.id = elem.id;
    outputElem.data = std::to_string(force);

    {
        std::lock_guard guard(*forwardMQTTMutex);
        forwardMQTTQueue->push(outputElem);
    }
}

void SensorProcessingLaneWorker::convertToFlexInputData(int& digVoltage, const DataToProcessorElement& elem){
    digVoltage = static_cast<int>(elem.data[0]);
}

namespace {
float u32BitsToFloat(uint32_t value){
    float out = 0.0f;
    std::memcpy(&out, &value, sizeof(float));
    return out;
}
}

void SensorProcessingLaneWorker::convertToImuInputData(Eigen::Vector3d& accels,
                                                       Eigen::Vector3d& gyro,
                                                       const DataToProcessorElement& elem){
    accels = Eigen::Vector3d::Zero();
    gyro = Eigen::Vector3d::Zero();

    if(elem.data.size() < 3){
        return;
    }

    Eigen::Vector3d reading;
    reading(0) = static_cast<double>(u32BitsToFloat(elem.data[0]));
    reading(1) = static_cast<double>(u32BitsToFloat(elem.data[1]));
    reading(2) = static_cast<double>(u32BitsToFloat(elem.data[2]));

    if(elem.type == SensorType::IMU_ACCEL){
        accels = reading;
    } else if(elem.type == SensorType::IMU_GYRO){
        gyro = reading;
    }
}

void SensorProcessingLaneWorker::convertToSpo2InputData(int& ir, int& red, const DataToProcessorElement& elem){
    ir = 0;
    red = 0;

    if(elem.data.size() < 2){
        return;
    }

    ir = static_cast<int>(elem.data[0]);
    red = static_cast<int>(elem.data[1]);
}
