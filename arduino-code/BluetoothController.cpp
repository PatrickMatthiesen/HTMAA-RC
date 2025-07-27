#include "BluetoothController.h"

// Static instance for callbacks
BluetoothController* BluetoothController::instance = nullptr;

// Global instance
BluetoothController bluetoothController;

BluetoothController::BluetoothController() {
    instance = this;
    // Initialize controller array
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        myControllers[i] = nullptr;
    }
}

void BluetoothController::begin() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Forget Bluetooth keys to fix connection issues
    BP32.forgetBluetoothKeys();

    // Disable virtual device (mouse/touchpad) support
    BP32.enableVirtualDevice(false);
}

void BluetoothController::update() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        // Process controllers if needed
        for (auto controller : myControllers) {
            if (controller && controller->isConnected() && controller->hasData()) {
                if (controller->isGamepad()) {
                    // Could add debug output here if needed
                    // dumpGamepad(controller);
                }
            }
        }
    }
}

bool BluetoothController::isControllerConnected() {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
            return true;
        }
    }
    return false;
}

int BluetoothController::getThrottle() {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected() && myControllers[i]->hasData()) {
            return myControllers[i]->throttle();
        }
    }
    return 0;
}

int BluetoothController::getLeftStickY() {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected() && myControllers[i]->hasData()) {
            return myControllers[i]->axisY();
        }
    }
    return 0;
}

int BluetoothController::getLeftStickX() {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected() && myControllers[i]->hasData()) {
            return myControllers[i]->axisX();
        }
    }
    return 0;
}

bool BluetoothController::isButtonPressed(int button) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected() && myControllers[i]->hasData()) {
            return (myControllers[i]->buttons() & button) != 0;
        }
    }
    return false;
}

int BluetoothController::getDpad()
{
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected() && myControllers[i]->hasData()) {
            return myControllers[i]->dpad();
        }
    }
    return 0;
}

void BluetoothController::dumpGamepads() {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected() && myControllers[i]->hasData()) {
            ControllerPtr ctl = myControllers[i];
            Serial.printf(
                "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
                "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
                ctl->index(),        // Controller Index
                ctl->dpad(),         // D-pad
                ctl->buttons(),      // bitmask of pressed buttons
                ctl->axisX(),        // (-511 - 512) left X Axis
                ctl->axisY(),        // (-511 - 512) left Y axis
                ctl->axisRX(),       // (-511 - 512) right X axis
                ctl->axisRY(),       // (-511 - 512) right Y axis
                ctl->brake(),        // (0 - 1023): brake button
                ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
                ctl->miscButtons(),  // bitmask of pressed "misc" buttons
                ctl->gyroX(),        // Gyro X
                ctl->gyroY(),        // Gyro Y
                ctl->gyroZ(),        // Gyro Z
                ctl->accelX(),       // Accelerometer X
                ctl->accelY(),       // Accelerometer Y
                ctl->accelZ()        // Accelerometer Z
            );
            Serial.flush();
            return;
        }
    }
}

// Static callback functions
void BluetoothController::onConnectedController(ControllerPtr ctl) {
    if (instance == nullptr) return;
    
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (instance->myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", 
                         ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            instance->myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void BluetoothController::onDisconnectedController(ControllerPtr ctl) {
    if (instance == nullptr) return;
    
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (instance->myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            instance->myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}
