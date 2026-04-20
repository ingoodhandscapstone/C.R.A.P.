#include "Bluetooth.h"

const int Bluetooth::SCAN_ATTEMPTS = 5;
const int Bluetooth::SCAN_TIMEOUT_MS = 5000;

const SimpleBLE::BluetoothAddress Bluetooth::GLOVE_ADDRESS = SimpleBLE::BluetoothAddress();
const SimpleBLE::BluetoothAddress Bluetooth::GRIPPER_ADDRESS = SimpleBLE::BluetoothAddress();

// Glove (no logging)
// Service
const SimpleBLE::BluetoothUUID commandUUID = SimpleBLE::BluetoothUUID("20d54f7d-de91-4cb3-9298-6144c02f8e38"); 
const SimpleBLE::BluetoothUUID sensorUUID = SimpleBLE::BluetoothUUID("9f185e32-8c9b-4e75-a4ad-fd38a798f267");

// Glove (no logging)
// Characteristics
const SimpleBLE::BluetoothUUID gyroUUID = SimpleBLE::BluetoothUUID("b74041dd-d096-430d-bc5e-863b4bf82a25");
const SimpleBLE::BluetoothUUID flexUUID = SimpleBLE::BluetoothUUID("0a5d19b7-c3f9-4e77-a054-b70ae8205af4");
const SimpleBLE::BluetoothUUID spo2UUID = SimpleBLE::BluetoothUUID("db73f8c5-c18a-4586-88b1-b0cba8a4d3a5");
const SimpleBLE::BluetoothUUID accelUUID = SimpleBLE::BluetoothUUID("8b57dd02-4fd4-4d61-ad77-1feaa12f4f4e");
const SimpleBLE::BluetoothUUID commandCharUUID = SimpleBLE::BluetoothUUID("fae5ce88-733c-460c-bff5-a3c7a61de43f");


bool Bluetooth::initialization(){

    auto adapters = SimpleBLE::Adapter::get_adapters();

    if(adapters.empty()){
        return false;
    }

    adapter = adapters[0];

    bool scanSuccess = false;

    for(int attempts = 0; attempts < SCAN_ATTEMPTS; attempts++){

        adapter.scan_for(SCAN_TIMEOUT_MS);

        auto peripherals = adapter.scan_get_results();

        for(int i = 0; i < peripherals.size(); i++){
            if(peripherals[i].address() == GLOVE_ADDRESS && !glove.initialized()){
                glove = peripherals[i];

            } else if (peripherals[i].address() == GRIPPER_ADDRESS && !gripper.initialized()){
                gripper = peripherals[i];
            }
        }

        if(glove.initialized() && gripper.initialized()){
            scanSuccess = true;
            break;
        }

            
    }

    if(!scanSuccess){
        return false;
    }


    glove.connect();
    gripper.connect();

    glove.notify(sensorUUID, gyroUUID, [this](SimpleBLE::ByteArray payload){this->addMessageToGyroStorage(payload);});
    glove.notify(sensorUUID, accelUUID, [this](SimpleBLE::ByteArray payload){this->addMessageToAccelStorage(payload);});
    glove.notify(sensorUUID, spo2UUID, [this](SimpleBLE::ByteArray payload){this->addMessageToSpo2Storage(payload);});
    glove.notify(sensorUUID, flexUUID, [this](SimpleBLE::ByteArray payload){this->addMessageToFlexStorage(payload);});

    // Still need to do the gripper stuff

    
    glove.set_callback_on_disconnected();


}


void Bluetooth::addMessageToGyroStorage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> gyroGuard(gyroDataMutex);
    gyroMessageStorage.push_back(payload);
}

void Bluetooth::addMessageToAccelStorage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> accelGuard(accelDataMutex);
    accelMessageStorage.push_back(payload);
}

void Bluetooth::addMessageToSpo2Storage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> spo2Guard(spo2DataMutex);
    spo2MessageStorage.push_back(payload);
}

void Bluetooth::addMessageToFlexStorage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> flexGuard(flexDataMutex);
    flexMessageStorage.push_back(payload);
}


void Bluetooth::connect(SimpleBLE::Peripheral& peripheral){
    std::lock_guard<std::mutex> flexGuard(flexDataMutex);

    if(peripheral.is_connected()){
        return;
    }


    
}
