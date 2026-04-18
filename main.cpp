


int main() {

    return 0;
}


// What libraries do I need ?


// What interfaces do I need?




// Process 1 - Communication
    // Communication object with read, write, isConnected -> list of bytes
    // Underlying BLE implementation
        // Receives information through notification from server
        // Writes to command char
    // If disconnected, always attempt to reconnect

    // Once data is read it should be deserialized into particular struct format for use
    // Placed into either one of two queues (depending on the sensor id and groupping of sensor processing associated with that queue)

    // Has queue from MQTT communication process. Currently, commands are in same (1 byte) format with same universal meaning per code across devices
    // We basically have session config, start, stop, calibration start, and calibration stop
    // Communication will forward all commands to the esp32 which enforces state machine logic to maintain expected program flow 
    // Calibration start will be forwaded to both sensor processing processes


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

// Process 4 - MQTT Process 

    // This will read from each of the Sensor Processing Queues and publish to the corresponding topic
    // Each specific measurement type has its own topic that the MQTT will publish to accordingly
    
    // The MQTT process will read from a command topic where the UI sends it topics. 
    // From here it should be able to directly send it to the communication process through a queue
        // Shouldnt be any conversion necessary (single byte, universal meaning of codes)