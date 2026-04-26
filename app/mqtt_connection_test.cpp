#include <cstdlib>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <random>
#include <string>
#include <thread>

#include "Logger.h"
#include "MQTTWorker.h"

namespace {

bool readInt(int& value) {
    while (true) {
        if (std::cin >> value) {
            return true;
        }

        if (std::cin.eof()) {
            return false;
        }

        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input. Enter a number: ";
    }
}

bool selectTopic(int choice, DataOutputElement& outputElement, bool& useFlexSpo2Queue) {
    switch (choice) {
        case 1:
            outputElement = DataOutputElement{SensorID::WRIST_SPO2, ""};
            useFlexSpo2Queue = true;
            return true;
        case 2:
            outputElement = DataOutputElement{SensorID::POINTER_MCP_FLEX, ""};
            useFlexSpo2Queue = true;
            return true;
        case 3:
            outputElement = DataOutputElement{SensorID::HAND_IMU, ""};
            useFlexSpo2Queue = false;
            return true;
        case 4:
            outputElement = DataOutputElement{SensorID::POINTER_IMU, ""};
            useFlexSpo2Queue = false;
            return true;
        case 5:
            outputElement = DataOutputElement{SensorID::POINTER_FORCE, ""};
            useFlexSpo2Queue = false;
            return true;
        default:
            return false;
    }
}

void printMainMenu() {
    std::cout << '\n'
              << "==== MQTT Connection Test Menu ====\n"
              << "1. Queue random data publish\n"
              << "2. Queue calibration completed publish\n"
              << "3. Read next UI command from queue\n"
              << "4. Exit\n"
              << "Choose option: ";
}

void printTopicMenu() {
    std::cout << "Select topic:\n"
              << "1. spo2/WRIST\n"
              << "2. joint/POINTER_MCP\n"
              << "3. orientation/WRIST_X\n"
              << "4. abduction/POINTER\n"
              << "5. force/POINTER\n"
              << "Choose topic: ";
}

} // namespace

int main() {
    Logger::instance().initialize("logs");

    std::queue<SessionCommand> mqttForwardCommandQueue;
    std::queue<DataOutputElement> flexSPO2ForwardMQTTQueue;
    std::queue<DataOutputElement> imuForceForwardMQTTQueue;
    std::queue<CalibrationStatusMessage> calibrationStatusQueue;

    std::mutex mqttForwardCommandMutex;
    std::mutex flexSPO2ForwardMQTTMutex;
    std::mutex imuForceForwardMQTTMutex;
    std::mutex calibrationStatusMutex;

    MQTTWorker mqttWorker;
    const bool connected = mqttWorker.initialize(
        &mqttForwardCommandQueue,
        &flexSPO2ForwardMQTTQueue,
        &imuForceForwardMQTTQueue,
        &calibrationStatusQueue,
        &mqttForwardCommandMutex,
        &flexSPO2ForwardMQTTMutex,
        &imuForceForwardMQTTMutex,
        &calibrationStatusMutex);

    if (!connected) {
        std::string failureReason = mqttWorker.getFailureReason();
        if (failureReason.empty()) {
            failureReason = "MQTT connection timed out or failed.";
        }

        Logger::instance().error("MQTTConnectionTest", failureReason, true);
        return EXIT_FAILURE;
    }

    Logger::instance().info("MQTTConnectionTest",
                            "MQTT connection succeeded using existing MQTTWorker/client setup.",
                            true);

    std::jthread mqttWorkerThread([&mqttWorker](std::stop_token stopToken) {
        mqttWorker.run(stopToken);
    });

    std::mt19937 randomEngine(std::random_device{}());
    std::uniform_int_distribution<int> randomDataDistribution(0, 360);
    uint32_t calibrationEpoch = 0;

    while (true) {
        if (mqttWorker.hasFailure()) {
            const std::string failureReason = mqttWorker.getFailureReason();
            Logger::instance().error(
                "MQTTConnectionTest",
                failureReason.empty() ? "MQTT worker reported failure." : failureReason,
                true);
            break;
        }

        printMainMenu();
        int option = 0;
        if (!readInt(option)) {
            std::cout << "\nInput stream closed. Exiting.\n";
            break;
        }

        if (option == 1) {
            printTopicMenu();
            int topicChoice = 0;
            if (!readInt(topicChoice)) {
                std::cout << "\nInput stream closed. Exiting.\n";
                break;
            }

            DataOutputElement outputElement{};
            bool useFlexSpo2Queue = false;
            if (!selectTopic(topicChoice, outputElement, useFlexSpo2Queue)) {
                std::cout << "Invalid topic option.\n";
                continue;
            }

            outputElement.data = std::to_string(randomDataDistribution(randomEngine));
            if (useFlexSpo2Queue) {
                std::lock_guard guard(flexSPO2ForwardMQTTMutex);
                flexSPO2ForwardMQTTQueue.push(outputElement);
            } else {
                std::lock_guard guard(imuForceForwardMQTTMutex);
                imuForceForwardMQTTQueue.push(outputElement);
            }

            std::cout << "Queued random payload: " << outputElement.data << '\n';
            continue;
        }

        if (option == 2) {
            CalibrationStatusMessage calibrationStatusMessage{
                .epoch = ++calibrationEpoch,
                .requiredCount = 1,
            };
            {
                std::lock_guard guard(calibrationStatusMutex);
                calibrationStatusQueue.push(calibrationStatusMessage);
            }

            std::cout << "Queued calibration completed publish message.\n";
            continue;
        }

        if (option == 3) {
            std::lock_guard guard(mqttForwardCommandMutex);
            if (mqttForwardCommandQueue.empty()) {
                std::cout << "No UI command available in queue.\n";
                continue;
            }

            const SessionCommand command = mqttForwardCommandQueue.front();
            mqttForwardCommandQueue.pop();
            std::cout << "Received UI command value: " << static_cast<int>(command) << '\n';
            continue;
        }

        if (option == 4) {
            break;
        }

        std::cout << "Invalid option.\n";
    }

    mqttWorkerThread.request_stop();
    Logger::instance().info("MQTTConnectionTest", "Exiting MQTT connection test.", true);
    return EXIT_SUCCESS;
}
