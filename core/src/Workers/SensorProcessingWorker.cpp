#include "SensorProcessingWorker.h"




void SensorProcessingWorker::initialize(std::unordered_map<SensorID, ImuProcessingConfig> * imuConfigs, std::unordered_map<SensorID, ResistiveSensorConfig> * resistiveSensorConfigs){
    this->imuConfigs = imuConfigs;
    this->resistiveSensorConfigs = resistiveSensorConfigs;

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




// CHANGE TO HAVE TO QUEUES GOING FROM COM TO THIS FOR EACH INDIVIDUAL PROCESSING SENSOR
bool SensorProcessingWorker::runSensorProcessFlexSPO2(){
    // Go into switch based on state
    // Whenever access any members EXCEPT in RUNNING, must use sharedAccessMutex
    // IDLE
        // grab lock and read comCommand queue (both member acqusition and lock for that specific queue)
        // if session config data, read it, set mostRecentConfigCommand, and pop it
        // if other data ignore it, do not pop
        // if relevant, transition to configured in switch state otherwise stay idle
    
    // CONFIGURED
        // grab lock and read comCommand queue (both member acqusition and lock for that specific queue)
        // if calibrate, transition to calibrate, and pop from command if other runSensorProcess is IDLE || CALIBRATE, if other sensor is !IDLE && !clibate then do not pop
        // Additionally, if other process is not in Calibrated then send command CALIBRATION_IN_PROGRESS through your queue (but for this you must block using shared member access as well)
        // Otherwise stay in this state
    // CALIBRATE
        // Run calibration loop
        // Once exited the loop 
        // if other process is in calibrated, the go to WAITING_TO_RUN
        // if other process is IDLE or in WAITING_TO_RUN, then use your queue to send a CALIBRATE_COMPLETED command and transition to RUNNING
    // WAITING_TO_RUN
        // Check state of other sensor to see once they leave calibrated
        // Once other state is in RUNNING, then this may transition to RUNNING
    
    // RUNNING
        // I will actually 

    
}


bool SensorProcessingWorker::runSensorProcessImuForce(){

}





// Process 2 & 3 - Sensor Processing 1 & 2
    // Sensor Processing 1 - Responsible for 6 IMUs (wrist orientation, finger abduction) and 5 force sensors (grip strength)
    // Sensor Processing 2 - Responsible for 14 Flex Sensors (rom at each finger joint) and 1 oximeter (SPO2)

    // Each of these processing units are geared towards a specific measurement (ex PINKY_MCP, Wrist Orientation, POINTER_ABDUCTION)
    // Notably this means each unit is not limited to access to a single sensor (in this case only finger abduction I believe needs 2 IMUs)

    // Each will have access to configuration object setting the necessary coefficients for its processing routine allowing customizability between units

    // Each of the sensor processes will contain a state machine enforcing particular action of the processors based on the state
        // This is how it will know during a calibration state to take data from queue from communication process and particularly gather calibration constants
        // On the start of a new session (WHICH IS REALLY SESSION CONFIG COMMAND) - Clear/reset members, coefficients, stored data, etc
        // On start calibration - It will interpret incoming data for the purpose of setting initial parameters (not output to UI)
        // On start session - It will interpret incoming data for the purpose of sending to MQTT process
    // The actual session config (what sensors will be used) are unnecessary for this to know because it is simply reacting to incoming data addressed to it
    // If no data sent is addressed to it, it does nothing

    // Each process will have its own output queue to the MQTT Process

