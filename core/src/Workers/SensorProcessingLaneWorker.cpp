#include "SensorProcessingLaneWorker.h"
#include <cstring>

std::mutex SensorProcessingLaneWorker::calibrationEpochMutex;
uint32_t SensorProcessingLaneWorker::calibrationEpochCounter = 0;
uint32_t SensorProcessingLaneWorker::calibrationEpochJoinCount = 0;
uint32_t SensorProcessingLaneWorker::calibrationEpochRequiredCount = 0;
bool SensorProcessingLaneWorker::calibrationEpochOpen = false;


void SensorProcessingLaneWorker::initialize(std::unordered_map<SensorID, ImuProcessingConfig> * imuConfigs,
                                            std::unordered_map<SensorID, ResistiveSensorConfig> * resistiveSensorConfigs,
                                            std::queue<DataOutputElement> * forwardMQTTQueue,
                                            std::queue<SessionCommand> * commandQueue,
                                            std::queue<DataToProcessorElement> * sensorDataQueue,
                                            std::queue<CalibrationStatusMessage> * calibrationStatusQueue,
                                            std::mutex * forwardMQTTMutex,
                                            std::mutex * commandMutex,
                                            std::mutex * sensorDataMutex,
                                            std::mutex * calibrationStatusMutex){
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

    // WRIST_SPO2 is intentionally skipped: BloodOxygenProcessor currently has no initialize() API.
    handIMUProcessor.initialize(&(imuConfigs->at(SensorID::HAND_IMU)));
    pointerIMUProcessor.initialize(&(imuConfigs->at(SensorID::POINTER_IMU)));
    middleIMUProcessor.initialize(&(imuConfigs->at(SensorID::MIDDLE_IMU)));
    thumbIMUProcessor.initialize(&(imuConfigs->at(SensorID::THUMB_IMU)));
    ringIMUProcessor.initialize(&(imuConfigs->at(SensorID::RING_IMU)));
    pinkyIMUProcessor.initialize(&(imuConfigs->at(SensorID::PINKY_IMU)));

    pointerMCPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::POINTER_MCP_FLEX)));
    pointerPIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::POINTER_PIP_FLEX)));
    pointerDIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::POINTER_DIP_FLEX)));
    middleMCPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::MIDDLE_MCP_FLEX)));
    middlePIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::MIDDLE_PIP_FLEX)));
    middleDIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::MIDDLE_DIP_FLEX)));
    ringMCPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::RING_MCP_FLEX)));
    ringPIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::RING_PIP_FLEX)));
    ringDIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::RING_DIP_FLEX)));
    pinkyMCPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::PINKY_MCP_FLEX)));
    pinkyPIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::PINKY_PIP_FLEX)));
    pinkyDIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::PINKY_DIP_FLEX)));
    thumbMCPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::THUMB_MCP_FLEX)));
    thumbPIPFlexProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::THUMB_PIP_FLEX)));

    pointerForceProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::POINTER_FORCE)));
    middleForceProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::MIDDLE_FORCE)));
    thumbForceProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::THUMB_FORCE)));
    ringForceProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::RING_FORCE)));
    pinkyForceProcessor.initialize(&(resistiveSensorConfigs->at(SensorID::PINKY_FORCE)));

 
}

bool SensorProcessingLaneWorker::isRelevantConfigCommand(SessionCommand command){
    if(command == SessionCommand::SESSION_CONFIG_ALL){
        return true;
    }

    if(processingGroup == ProcessingGroup::FLEX_SPO2){
        return command == SessionCommand::SESSION_CONFIG_WRIST ||
               command == SessionCommand::SESSION_CONFIG_POINTER ||
               command == SessionCommand::SESSION_CONFIG_SPO2;
    }

    return command == SessionCommand::SESSION_CONFIG_WRIST ||
           command == SessionCommand::SESSION_CONFIG_POINTER ||
           command == SessionCommand::SESSION_CONFIG_GRIPPER;
}

uint32_t SensorProcessingLaneWorker::getRequiredCalibrationCount(SessionCommand command){
    // Add to this as more session configs are added
    if(command == SessionCommand::SESSION_CONFIG_SPO2 || command == SessionCommand::SESSION_CONFIG_GRIPPER){
        return 1;
    }
    return 2;
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

void SensorProcessingLaneWorker::resetProcessingData(){
    if(processingGroup == ProcessingGroup::FLEX_SPO2){
        resetFlexSPO2Data();
    } else {
        resetImuForceData();
    }
}

void SensorProcessingLaneWorker::run(){
    while(true){
        switch(state){
            case SensorProcessingState::IDLE: {
                SessionCommand command = SessionCommand::NONE;
                if(!readFrontCommand(command)){
                    break;
                }

                if(isRelevantConfigCommand(command)){
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
                    popFrontCommand();
                }
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
                    state = SensorProcessingState::CALIBRATING;
                    popFrontCommand();
                }
                break;
            }

            case SensorProcessingState::CALIBRATING: {
                if(runCalibration()){
                    CalibrationStatusMessage message;
                    message.epoch = currentCalibrationEpoch;
                    message.requiredCount = getRequiredCalibrationCount(mostRecentConfigCommand);
                    {
                        std::lock_guard guard(*calibrationStatusMutex);
                        calibrationStatusQueue->push(message);
                    }
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
                state = SensorProcessingState::IDLE;
                break;
            }

            default:
                break;
        }
    }
}



void SensorProcessingLaneWorker::resetFlexSPO2Data(){
    clearSensorDataQueue();
}

void SensorProcessingLaneWorker::resetImuForceData(){
    clearSensorDataQueue();
}


// This will be added to as more session config commands are added
bool SensorProcessingLaneWorker::setConfigFunctionFlexSpo2(){
    switch(mostRecentConfigCommand){
        case SessionCommand::SESSION_CONFIG_WRIST:
            break;  
        case SessionCommand::SESSION_CONFIG_ALL:
            break;
        case SessionCommand::SESSION_CONFIG_POINTER:
            calibrationFunc = &SensorProcessingLaneWorker::pointerCalibrationFuncFlexSpo2;
            sessionFunc = &SensorProcessingLaneWorker::pointerSessionFuncFlexSpo2;
            break;
        case SessionCommand::SESSION_CONFIG_SPO2:
            break;
        case SessionCommand::SESSION_CONFIG_GRIPPER:
            break;

        default:
            return false;

    }

    return true;
}

bool SensorProcessingLaneWorker::setConfigFunctionImuForce(){
    switch(mostRecentConfigCommand){
        case SessionCommand::SESSION_CONFIG_WRIST:
            break;
        case SessionCommand::SESSION_CONFIG_ALL:
            break;
        case SessionCommand::SESSION_CONFIG_POINTER:
            calibrationFunc = &SensorProcessingLaneWorker::pointerCalibrationFuncImuForce;
            sessionFunc = &SensorProcessingLaneWorker::pointerSessionFuncImuForce;
            break;
        case SessionCommand::SESSION_CONFIG_SPO2:
            break;
        case SessionCommand::SESSION_CONFIG_GRIPPER:
            break;

        default:
            return false;

    }

    return true;
}

bool SensorProcessingLaneWorker::pointerCalibrationFuncFlexSpo2(){
    // We need to call each of the relevant sensors (for this session config) calibrate functions
    // Pointer finger uses three flex sensors and one imu

    bool calibrationDone = true;
    DataToProcessorElement elem;

    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return false;
        }

        elem = sensorDataQueue->front();


        sensorDataQueue->pop();

    }

    // Shouldnt be but in case 
    if(elem.id == SensorID::WRIST_SPO2){
        return false;
    }

    int digVolt;

    convertToFlexInputData(digVolt, elem);
    
    bool success = true;
    JointRomProcessor& pointerProc = findPointerSensor(elem.id, success);

    if(!success){
        return false; 
    }

    pointerProc.calibration(digVolt);

    return isFlexSpo2PointerConfigCalibrated();

    // Grab mutex and attempt to read from flexSpo2 queue
    // if empty, do nothing
    // else
    // convert data into usable form to pass to calibrate
    // call calibrate function of each of the three flex sensors. calibrationDone = calibrationDone && flex1.calibrate() && ...

}


JointRomProcessor& SensorProcessingLaneWorker::findPointerSensor(SensorID& id, bool& success){
    if(SensorID::POINTER_MCP_FLEX == id){
        return pointerMCPFlexProcessor;
    } else if(SensorID::POINTER_PIP_FLEX == id){
        return pointerPIPFlexProcessor;
    } else if(SensorID::POINTER_DIP_FLEX == id){
        return pointerDIPFlexProcessor;
    }

    success = false;
    return pointerDIPFlexProcessor; // return something that will not be used
}


bool SensorProcessingLaneWorker::isFlexSpo2PointerConfigCalibrated(){
    return pointerDIPFlexProcessor.isCalibrated() && pointerPIPFlexProcessor.isCalibrated() && pointerMCPFlexProcessor.isCalibrated();
}


void SensorProcessingLaneWorker::convertToFlexInputData(int& digVoltage, DataToProcessorElement& elem){
    // I know this reading is less than rollover of int, so conversion as such is fine
    digVoltage = elem.data[0]; 
}

namespace {
float u32BitsToFloat(uint32_t value){
    float out = 0.0f;
    std::memcpy(&out, &value, sizeof(float));
    return out;
}
}

void SensorProcessingLaneWorker::convertToImuInputData(Eigen::Vector3d& accels, Eigen::Vector3d& gyro, DataToProcessorElement& elem){
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

void SensorProcessingLaneWorker::convertToSpo2InputData(int& ir, int& red, DataToProcessorElement& elem){
    ir = 0;
    red = 0;
    if(elem.data.size() < 2){
        return;
    }
    ir = static_cast<int>(elem.data[0]);
    red = static_cast<int>(elem.data[1]);
}

bool SensorProcessingLaneWorker::pointerCalibrationFuncImuForce(){
    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return false;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if(elem.id != SensorID::POINTER_IMU && elem.id != SensorID::HAND_IMU){
        return false;
    }

    Eigen::Vector3d accels;
    Eigen::Vector3d gyro;
    convertToImuInputData(accels, gyro, elem);

    static Eigen::Vector3d pointerAccel = Eigen::Vector3d::Zero();
    static Eigen::Vector3d pointerGyro = Eigen::Vector3d::Zero();
    static Eigen::Vector3d handAccel = Eigen::Vector3d::Zero();
    static Eigen::Vector3d handGyro = Eigen::Vector3d::Zero();
    static bool pointerHasAccel = false;
    static bool pointerHasGyro = false;
    static bool handHasAccel = false;
    static bool handHasGyro = false;
    static bool pointerDone = false;
    static bool handDone = false;

    if(elem.id == SensorID::POINTER_IMU){
        if(elem.type == SensorType::IMU_ACCEL){
            pointerAccel = accels;
            pointerHasAccel = true;
        } else if(elem.type == SensorType::IMU_GYRO){
            pointerGyro = gyro;
            pointerHasGyro = true;
        }
    } else if(elem.id == SensorID::HAND_IMU){
        if(elem.type == SensorType::IMU_ACCEL){
            handAccel = accels;
            handHasAccel = true;
        } else if(elem.type == SensorType::IMU_GYRO){
            handGyro = gyro;
            handHasGyro = true;
        }
    }

    if(pointerHasAccel && pointerHasGyro){
        pointerDone = pointerIMUProcessor.calibrate(pointerAccel, pointerGyro);
        pointerHasAccel = false;
        pointerHasGyro = false;
    }

    if(handHasAccel && handHasGyro){
        handDone = handIMUProcessor.calibrate(handAccel, handGyro);
        handHasAccel = false;
        handHasGyro = false;
    }

    return pointerDone && handDone;
}

void SensorProcessingLaneWorker::pointerSessionFuncImuForce(){
    DataToProcessorElement elem;
    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return;
        }

        elem = sensorDataQueue->front();
        sensorDataQueue->pop();
    }

    if(elem.id != SensorID::POINTER_IMU && elem.id != SensorID::HAND_IMU){
        return;
    }

    Eigen::Vector3d accels;
    Eigen::Vector3d gyro;
    convertToImuInputData(accels, gyro, elem);

    if(elem.id == SensorID::POINTER_IMU){
        if(elem.type == SensorType::IMU_ACCEL){
            pointerIMUProcessor.setAccel(accels, elem.timestamp);
        } else if(elem.type == SensorType::IMU_GYRO){
            pointerIMUProcessor.setGyro(gyro, elem.timestamp);
        } else {
            return;
        }
    } else {
        if(elem.type == SensorType::IMU_ACCEL){
            handIMUProcessor.setAccel(accels, elem.timestamp);
        } else if(elem.type == SensorType::IMU_GYRO){
            handIMUProcessor.setGyro(gyro, elem.timestamp);
        } else {
            return;
        }
    }

    static bool handHasUpdated = false;

    if(handIMUProcessor.hasGyroAndAccel()){
        handIMUProcessor.predict();
        handIMUProcessor.update();
        handHasUpdated = true;
    }

    bool pointerUpdated = false;
    if(pointerIMUProcessor.hasGyroAndAccel()){
        pointerIMUProcessor.predict();
        pointerIMUProcessor.update();
        pointerUpdated = true;
    }

    if(!pointerUpdated || !handHasUpdated){
        return;
    }

    float angle = 0.0f;
    Eigen::Matrix3d handOrientation = handIMUProcessor.getHandOrientationMatrix();
    pointerIMUProcessor.getAngle(angle, handOrientation);

    DataOutputElement outputElem;
    outputElem.id = SensorID::POINTER_IMU;
    outputElem.data = std::to_string(angle);
    {
        std::lock_guard guard(*forwardMQTTMutex);
        forwardMQTTQueue->push(outputElem);
    }
}


void SensorProcessingLaneWorker::pointerSessionFuncFlexSpo2(){
    DataToProcessorElement elem;

    {
        std::lock_guard guard(*sensorDataMutex);
        if(sensorDataQueue->empty()){
            return;
        }

        elem = sensorDataQueue->front();


        sensorDataQueue->pop();

    }

    // Only pointer flex sensors are valid for this session function.
    if(elem.id == SensorID::WRIST_SPO2){
        return;
    }

    int digVolt;

    convertToFlexInputData(digVolt, elem);
    
    bool success = true;
    JointRomProcessor& pointerProc = findPointerSensor(elem.id, success);

    if(!success){
        return; 
    }

    float angle = 0.0f;
    if(!pointerProc.getJointAngle(digVolt, angle)){
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



 // IDLE
            // Grab flexSpo2 mutex and read from corresponding comCommand queue
            // if session config has message and message is relevant, 
                // grab sharedMemberAccess mutex
                // set the session command
                // set the flex spo2 calibrationFunc pointer
                // set the flex spo2 sessionFunc pointer
                // pop command from queue
                // got to CONFIGURED (by setting flexSpo2 SensorProcessingState)
            // otherwise, stay in this state
        // CONFIGURED
            // Grab flexSpo2 mutex and read from corresponding comCommand queue
            // if CALIBRATE_SESSION command 
                // pop command from queue
                // go to CALIBRATE
            // otherwise
                // do nothing
        // CALIBRATE
            // Run the flexSpo2CalibrationFunc
            // if it returns true
                // go to WAITING_TO RUN
            // otherwise 
                // do nothing
        // WAITING_TO_RUN
            // grab comCommandFlexSpo2 Mutex and read from it 
            // if start session command
                // grab sharedMemberAccessMutex and read imuForceState
                // if in calibrate 
                    // grab the sensorDataProcessingFlexSpo2 mutex and empty the queue 
                // if in IDLE || WAITING_TO_RUN || RUNNING
                    // grab flexSpo2 mqtt mutex
                    // send command calibration completed in respective mqtt queue
                    // pop command from queue
                    // go to RUNNING
        // RUNNING
            // grab mutex for com command flexSpo2 and read
            // if session stop command
                // pop command from queue
                // grab the sensorDataProcessingFlexSpo2 mutex and empty the queue 
                // go to RESETTING state
             // otherwise 
                // grab sensorDataProcessing flexSpo2 mutex and read it 
                // if not empty
                    // run the flexSpo2 session func pointer and pass argument
                // otherwise
                    // do nothing
        // RESETTING
            // run the respective resetting function for flexSpo2
            // go to IDLE


// question is how do we relay to the esp devices that we are done calibrating?
// Does the esp ever need to "know know" what calibrating is because its function of sending data doesnt change
// from calibrating to start_session. 
// I say this because of stale data in the comSensorDataProcessingQueue from calibration even one a evice is done calibrating (other one may not be done yet)
// In waiting to run we can pop all data in the com queue as it comes (stale data) while waiting and on exit to the start session (I am assuming on the user interface as soon as it receives calibration done it will transiition into start of state immediately)


// These 2 runSensorProcess functions are running in separate threads
