#include <cstdlib>

#include "Bluetooth.h"
#include "Logger.h"

int main() {
    Logger::instance().initialize("logs");

    Bluetooth bluetooth;
    const bool connected = bluetooth.initialize();

    if (!connected) {
        Logger::instance().error("BluetoothConnectionTest",
                                 "Bluetooth connection timed out or failed.",
                                 true);
        return EXIT_FAILURE;
    }

    Logger::instance().info("BluetoothConnectionTest", "Bluetooth connection succeeded.", true);
    return EXIT_SUCCESS;
}
