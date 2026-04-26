#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "Communication.h"
#include "Endpoints.h"
#include "simpleble/SimpleBLE.h"
#include "simpleble/Adapter.h"
#include "simpleble/Utils.h"


#include <vector>
#include <queue>
#include <cstdint>
#include <mutex>
#include <atomic>



class Bluetooth : public Communication {


    static const int SCAN_ATTEMPTS;
    static const int RECONNECT_ATTEMPTS;
    static const int SCAN_TIMEOUT_MS;
    static const SimpleBLE::BluetoothAddress GLOVE_ADDRESS;
    static const SimpleBLE::BluetoothAddress GRIPPER_ADDRESS;

    // Glove (no logging)
    // Service
    static const SimpleBLE::BluetoothUUID gloveCommandUUID; 
    static const SimpleBLE::BluetoothUUID gloveSensorUUID;

    // Glove (no logging)
    // Characteristics
    static const SimpleBLE::BluetoothUUID gyroUUID;
    static const SimpleBLE::BluetoothUUID flexUUID;
    static const SimpleBLE::BluetoothUUID spo2UUID;
    static const SimpleBLE::BluetoothUUID accelUUID;
    static const SimpleBLE::BluetoothUUID gloveCommandCharUUID;

    // Gripper (no logging)
    // Service
    static const SimpleBLE::BluetoothUUID gripperCommandUUID; 
    static const SimpleBLE::BluetoothUUID gripperSensorUUID;

    // Gripper (no logging)
    // Characteristics
    static const SimpleBLE::BluetoothUUID gripperCommandCharUUID;
    static const SimpleBLE::BluetoothUUID forceUUID;


    SimpleBLE::Safe::Adapter adapter;
    SimpleBLE::Safe::Peripheral glove;
    SimpleBLE::Safe::Peripheral gripper;

    std::queue<std::vector<uint8_t>> gyroMessageStorage;
    std::queue<std::vector<uint8_t>> accelMessageStorage;
    std::queue<std::vector<uint8_t>> flexMessageStorage;
    std::queue<std::vector<uint8_t>> spo2MessageStorage;
    std::queue<std::vector<uint8_t>> forceMessageStorage;

    std::mutex gyroDataMutex;
    std::mutex accelDataMutex;
    std::mutex spo2DataMutex;
    std::mutex flexDataMutex;
    std::mutex forceDataMutex;

    std::mutex gloveAccessMutex;
    std::mutex gripperAccessMutex;
    std::atomic_bool gloveReconnectRequested;


    public:
        Bluetooth() :
            adapter(SimpleBLE::Adapter()),
            glove(SimpleBLE::Peripheral()),
            gripper(SimpleBLE::Peripheral()),
            gyroMessageStorage(),
            accelMessageStorage(),
            flexMessageStorage(),
            spo2MessageStorage(),
            forceMessageStorage(),
            gyroDataMutex(),
            accelDataMutex(),
            spo2DataMutex(),
            flexDataMutex(),
            forceDataMutex(),
            gloveAccessMutex(),
            gripperAccessMutex(),
            gloveReconnectRequested(false) {}

        bool initialize();
        bool read(const Endpoints& endpoint, std::vector<uint8_t>& message) override;
        bool write(const Endpoints& endpoint, std::vector<uint8_t>& message) override;
        bool isConnected() override;


    private:

        void addMessageToGyroStorage(SimpleBLE::ByteArray payload);
        void addMessageToAccelStorage(SimpleBLE::ByteArray payload);
        void addMessageToSpo2Storage(SimpleBLE::ByteArray payload);
        void addMessageToFlexStorage(SimpleBLE::ByteArray payload);
        void addMessageToForceStorage(SimpleBLE::ByteArray payload);

        bool scanForPeripheral(const SimpleBLE::BluetoothAddress& targetAddress,
                               const char * peripheralName,
                               SimpleBLE::Safe::Peripheral& peripheral);
        bool connectPeripheral(SimpleBLE::Safe::Peripheral& peripheral, const char * peripheralName);
        bool setupGloveNotifications();
        bool setupGloveDisconnectCallback();
        void requestGloveReconnect();
        void serviceReconnects();
        bool reconnectGlove();
        bool readMessageStorage(std::mutex& mutex, std::queue<std::vector<uint8_t>>& messageStorage, std::vector<uint8_t>& message);
        

};






#endif
