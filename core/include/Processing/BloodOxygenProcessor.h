#ifndef BLOOD_OXYGEN_PROCESSOR_H
#define BLOOD_OXYGEN_PROCESSOR_H




class BloodOxygenProcessor {


    public:
        // Returns true once conversion takes place which requires a certain amount of samples
        bool getSPO2(int& spo2, int& ir, int& red);



};














#endif