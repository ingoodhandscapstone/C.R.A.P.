#ifndef SESSION_CONFIG_H
#define SESSION_CONFIG_H


enum class SessionCommand {
    NONE = 0, // This is never sent by PI. This is just initial value and can be used for errors
    SESSION_CONFIG_WRIST,
    SESSION_CONFIG_ALL,
    SESSION_CONFIG_POINTER,
    SESSION_CONFIG_SPO2,
    SESSION_START,
    SESSION_STOP,
    CALIBRATE_SESSION,
    CALIBRATION_IN_PROGRESS,
    CALIBRATION_COMPLETED,
    SESSION_CONFIG_GRIPPER

    


};



#endif