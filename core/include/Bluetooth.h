#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "Communication.h"
#include "simpleble/SimpleBLE.h"
#include "simpleble/Adapter.h"
#include "simpleble/Utils.h"

#include <vector>
#include <cstdint>
#include <mutex>


class Bluetooth : public Communication {


    static const int SCAN_ATTEMPTS;
    static const int SCAN_TIMEOUT_MS;
    static const SimpleBLE::BluetoothAddress GLOVE_ADDRESS;
    static const SimpleBLE::BluetoothAddress GRIPPER_ADDRESS;

    // Glove (no logging)
    // Service
    static const SimpleBLE::BluetoothUUID commandUUID; 
    static const SimpleBLE::BluetoothUUID sensorUUID;

    // Glove (no logging)
    // Characteristics
    static const SimpleBLE::BluetoothUUID gyroUUID;
    static const SimpleBLE::BluetoothUUID flexUUID;
    static const SimpleBLE::BluetoothUUID spo2UUID;
    static const SimpleBLE::BluetoothUUID accelUUID;
    static const SimpleBLE::BluetoothUUID commandCharUUID;


    SimpleBLE::Adapter adapter;
    SimpleBLE::Peripheral glove;
    SimpleBLE::Peripheral gripper;

    std::vector<std::vector<uint8_t>> gyroMessageStorage;
    std::vector<std::vector<uint8_t>> accelMessageStorage;
    std::vector<std::vector<uint8_t>> flexMessageStorage;
    std::vector<std::vector<uint8_t>> spo2MessageStorage;

    std::mutex gyroDataMutex;
    std::mutex accelDataMutex;
    std::mutex spo2DataMutex;
    std::mutex flexDataMutex;

    std::mutex gloveAccessMutex;
    std::mutex gripperAccessMutex;


    public:
        bool initialization();
        bool read(const Endpoint& endpoint, uint8_t * message, uint16_t size) override;
        bool write(const Endpoint& endpoint, uint8_t * message, uint16_t size ) override;
        bool isConnected() override;


    private:

        void addMessageToGyroStorage(SimpleBLE::ByteArray payload);
        void addMessageToAccelStorage(SimpleBLE::ByteArray payload);
        void addMessageToSpo2Storage(SimpleBLE::ByteArray payload);
        void addMessageToFlexStorage(SimpleBLE::ByteArray payload);

        void connect(SimpleBLE::Peripheral& peripheral);
        


};






#endif