#ifndef BLUETOOTH_CONTROLLER_H
#define BLUETOOTH_CONTROLLER_H

#include <Bluepad32.h>

class BluetoothController {
private:
    ControllerPtr myControllers[BP32_MAX_GAMEPADS];
    
    // Callback functions
    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);
    
    // Static instance for callbacks
    static BluetoothController* instance;

public:
    BluetoothController();
    void begin();
    void update();
    
    // Controller data getters
    bool isControllerConnected();
    int getThrottle();           // Returns throttle value (0-1023)
    int getLeftStickY();         // Returns left stick Y axis (-511 to 512)
    int getLeftStickX();         // Returns left stick X axis (-511 to 512)
    bool isButtonPressed(int button);
    int getDpad();
    
    // Debug functions
    void dumpGamepads();
};

// Global instance declaration
extern BluetoothController bluetoothController;

#endif
