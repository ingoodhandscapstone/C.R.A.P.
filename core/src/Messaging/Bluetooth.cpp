#include "Bluetooth.h"
#include "Logger.h"

#include <string>

const int Bluetooth::SCAN_ATTEMPTS = 10;
const int Bluetooth::RECONNECT_ATTEMPTS = 10;
const int Bluetooth::SCAN_TIMEOUT_MS = 4000;

const SimpleBLE::BluetoothAddress Bluetooth::GLOVE_ADDRESS = SimpleBLE::BluetoothAddress("C2:12:34:56:78:9A");
const SimpleBLE::BluetoothAddress Bluetooth::GRIPPER_ADDRESS = SimpleBLE::BluetoothAddress();

// Glove (no logging)
// Service
const SimpleBLE::BluetoothUUID Bluetooth::gloveCommandUUID = SimpleBLE::BluetoothUUID("20d54f7d-de91-4cb3-9298-6144c02f8e38"); 
const SimpleBLE::BluetoothUUID Bluetooth::gloveSensorUUID = SimpleBLE::BluetoothUUID("9f185e32-8c9b-4e75-a4ad-fd38a798f267");

// Glove (no logging)
// Characteristics
const SimpleBLE::BluetoothUUID Bluetooth::gyroUUID = SimpleBLE::BluetoothUUID("b74041dd-d096-430d-bc5e-863b4bf82a25");
const SimpleBLE::BluetoothUUID Bluetooth::flexUUID = SimpleBLE::BluetoothUUID("0a5d19b7-c3f9-4e77-a054-b70ae8205af4");
const SimpleBLE::BluetoothUUID Bluetooth::spo2UUID = SimpleBLE::BluetoothUUID("db73f8c5-c18a-4586-88b1-b0cba8a4d3a5");
const SimpleBLE::BluetoothUUID Bluetooth::accelUUID = SimpleBLE::BluetoothUUID("fae5ce88-733c-460c-bff5-a3c7a61de43f");
const SimpleBLE::BluetoothUUID Bluetooth::gloveCommandCharUUID = SimpleBLE::BluetoothUUID("8b57dd02-4fd4-4d61-ad77-1feaa12f4f4e");

// Gripper (no logging)
// Service
const SimpleBLE::BluetoothUUID Bluetooth::gripperCommandUUID = SimpleBLE::BluetoothUUID(); 
const SimpleBLE::BluetoothUUID Bluetooth::gripperSensorUUID = SimpleBLE::BluetoothUUID();

// Gripper (no logging)
// Characteristics
const SimpleBLE::BluetoothUUID Bluetooth::gripperCommandCharUUID = SimpleBLE::BluetoothUUID();
const SimpleBLE::BluetoothUUID Bluetooth::forceUUID = SimpleBLE::BluetoothUUID();


bool Bluetooth::initialize(){
    Logger::instance().info("Bluetooth", "Initializing Bluetooth communication.");

    auto adapters = SimpleBLE::Safe::Adapter::get_adapters();

    if(!adapters || adapters->empty()){
        Logger::instance().error("Bluetooth", "No Bluetooth adapters found.", true);
        return false;
    }

    adapter = adapters.value()[0];

    bool success = scanForPeripheral(GLOVE_ADDRESS, "glove", glove);
    // success = success && scanForPeripheral(GRIPPER_ADDRESS, "gripper", gripper);
    if(!success){
        return false;
    }

    success = connectPeripheral(glove, "glove");
    // success = success && connectPeripheral(gripper, "gripper");
    if(!success){
        return false;
    }

    success = success && glove.notify(gloveSensorUUID, gyroUUID, [this](SimpleBLE::ByteArray payload){this->addMessageToGyroStorage(payload);});
    success = success && glove.notify(gloveSensorUUID, accelUUID, [this](SimpleBLE::ByteArray payload){this->addMessageToAccelStorage(payload);});
    success = success && glove.notify(gloveSensorUUID, spo2UUID, [this](SimpleBLE::ByteArray payload){this->addMessageToSpo2Storage(payload);});
    success = success && glove.notify(gloveSensorUUID, flexUUID, [this](SimpleBLE::ByteArray payload){this->addMessageToFlexStorage(payload);});
    //success = success && gripper.notify(gripperSensorUUID, forceUUID, [this](SimpleBLE::ByteArray payload){this->addMessageToForceStorage(payload);});

    success = success && glove.set_callback_on_disconnected([this](){this->reconnect(glove, "glove");});
   // success = success && gripper.set_callback_on_disconnected([this](){this->reconnect(gripper, "gripper");});

    if(!success){
        Logger::instance().error("Bluetooth", "Bluetooth setup failed after initial connection.", false);
        return false;
    }

    return success;


}


bool Bluetooth::scanForPeripheral(const SimpleBLE::BluetoothAddress& targetAddress,
                                  const char * peripheralName,
                                  SimpleBLE::Safe::Peripheral& peripheral) {
    for(int attempt = 0; attempt < SCAN_ATTEMPTS; attempt++){
        const int scanAttemptNumber = attempt + 1;

        adapter.scan_for(SCAN_TIMEOUT_MS);
        auto peripherals = adapter.scan_get_results();

        std::vector<std::string> foundAddresses;

        if(peripherals.has_value() && !peripherals->empty()){
            for(int i = 0; i < peripherals->size(); i++){
                auto address = peripherals->at(i).address();
                if(!address.has_value()){
                    continue;
                }

                foundAddresses.push_back(address.value());

                if(address.value() == targetAddress){
                    peripheral = peripherals->at(i);
                }
            }
        }

        if(foundAddresses.empty()){
            Logger::instance().info(
                "Bluetooth",
                "No addresses found for scan attempt " + std::to_string(scanAttemptNumber) + ".",
                true);
            continue;
        }

        std::string foundAddressesMessage =
            "Scan attempt " + std::to_string(scanAttemptNumber) + " found addresses: ";
        for(std::size_t i = 0; i < foundAddresses.size(); i++){
            foundAddressesMessage += foundAddresses.at(i);
            if(i + 1 < foundAddresses.size()){
                foundAddressesMessage += ", ";
            }
        }
        Logger::instance().info("Bluetooth", foundAddressesMessage, false);

        auto foundAddress = peripheral.address();
        if(foundAddress.has_value() && foundAddress.value() == targetAddress){
            Logger::instance().info(
                "Bluetooth",
                std::string("Found ") + peripheralName + " on scan attempt " +
                    std::to_string(scanAttemptNumber) + ".",
                true);
            return true;
        }
    }

    Logger::instance().error(
        "Bluetooth",
        std::string("Failed to find ") + peripheralName + " after " +
            std::to_string(SCAN_ATTEMPTS) + " scan attempts.",
        true);
    return false;
}

bool Bluetooth::connectPeripheral(SimpleBLE::Safe::Peripheral& peripheral, const char * peripheralName) {
    for(int attempt = 0; attempt < RECONNECT_ATTEMPTS; attempt++){
        const int connectAttemptNumber = attempt + 1;

        if(peripheral.connect()){
            Logger::instance().info(
                "Bluetooth",
                std::string("Connected to ") + peripheralName + " on attempt " +
                    std::to_string(connectAttemptNumber) + ".",
                true);
            return true;
        }

        Logger::instance().warn(
            "Bluetooth",
            std::string("Connection attempt ") + std::to_string(connectAttemptNumber) +
                " failed for " + peripheralName + ".",
            true);
    }

    Logger::instance().error(
        "Bluetooth",
        std::string("Failed to connect to ") + peripheralName + " after " +
            std::to_string(RECONNECT_ATTEMPTS) + " attempts.",
        true);
    return false;
}

void Bluetooth::addMessageToGyroStorage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> gyroGuard(gyroDataMutex);
    gyroMessageStorage.push(payload);
}

void Bluetooth::addMessageToAccelStorage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> accelGuard(accelDataMutex);
    accelMessageStorage.push(payload);
}

void Bluetooth::addMessageToSpo2Storage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> spo2Guard(spo2DataMutex);
    spo2MessageStorage.push(payload);
}

void Bluetooth::addMessageToFlexStorage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> flexGuard(flexDataMutex);
    flexMessageStorage.push(payload);
}

void Bluetooth::addMessageToForceStorage(SimpleBLE::ByteArray payload) {
    std::lock_guard<std::mutex> forceGuard(forceDataMutex);
    forceMessageStorage.push(payload);
}


void Bluetooth::reconnect(SimpleBLE::Safe::Peripheral& peripheral, const char * peripheralName){
    Logger::instance().warn("Bluetooth",
                            std::string("Disconnected from ") + peripheralName +
                                ". Attempting reconnect.",
                            true);

    for(int i = 0; i < RECONNECT_ATTEMPTS; i++){
        if(peripheral.connect()) {
            Logger::instance().info("Bluetooth",
                                    std::string("Reconnected to ") + peripheralName + ".",
                                    true);
            return;
        }
    }

    Logger::instance().error("Bluetooth",
                             std::string("Reconnect failed for ") + peripheralName + ".",
                             true);

}


// This will need to be extended for the gripper (one gripper enums are completed)
bool Bluetooth::read(const Endpoints& endpoint, std::vector<uint8_t>& message){
    bool success = false;

    switch(endpoint){
        case Endpoints::IMU_GYRO_CHAR:
            return readMessageStorage(gyroDataMutex, gyroMessageStorage, message);
            break;
        case Endpoints::IMU_ACCEL_CHAR:
            return readMessageStorage(accelDataMutex, accelMessageStorage, message);
            break;
        case Endpoints::FLEX_CHAR:
            return readMessageStorage(flexDataMutex, flexMessageStorage, message);
            break;
        case Endpoints::SPO2_CHAR:
            return readMessageStorage(spo2DataMutex, spo2MessageStorage, message);
            break;
        case Endpoints::FORCE_CHAR:
            return readMessageStorage(forceDataMutex, forceMessageStorage, message);
            break;
        case Endpoints::COMMAND_CHAR_GLOVE: // We only write to this, we do not read
        case Endpoints::COMMAND_CHAR_GRIPPER:
        default:
            return success;
    }
}

bool Bluetooth::readMessageStorage(std::mutex& mutex, std::queue<std::vector<uint8_t>>& messageStorage, std::vector<uint8_t>& message){
    std::lock_guard readLock(mutex);

    if(messageStorage.empty()){
        return false;
    }

    message = messageStorage.front();

    messageStorage.pop();

    return true;
}




bool Bluetooth::write(const Endpoints& endpoint, std::vector<uint8_t>& message) {

    bool success = false;
    if(endpoint == Endpoints::COMMAND_CHAR_GLOVE){
        success = glove.write_request(gloveCommandUUID, gloveCommandCharUUID, message);
    } else if (endpoint == Endpoints::COMMAND_CHAR_GRIPPER){
        success = gripper.write_request(gripperCommandUUID, gripperCommandCharUUID, message);
    }

    return success;

    // Each peripheral device has a state machine to reglate its behavior (so if messages are sent that do not match expected flow) it should have no impact
    // Therefore I can just send message to both devices

}

bool Bluetooth::isConnected(){
    auto gloveConnected = glove.is_connected();
    // auto gripperConnected = gripper.is_connected();

    return gloveConnected.has_value() && gloveConnected.value();
    // return gloveConnected.has_value() && gloveConnected.value()
    //         && gripperConnected.has_value() && gripperConnected.value();
}
