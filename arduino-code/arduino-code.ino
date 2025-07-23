#include "BluetoothController.h"

// Motor control pins
const int motorPin1 = 18; // A-1A
const int motorPin2 = 19; // A-1B

// PWM channels for ESP32
const int motorChannel1 = 0;
const int motorChannel2 = 1;
const int pwmFreq = 1000;
const int pwmResolution = 8;

// Motor control variables
int motorSpeed = 0;
bool motorDirection = true; // true = forward, false = reverse

// Throttle control parameters
const int MIN_MOTOR_PWM = 100;  // Minimum PWM to overcome friction
const float THROTTLE_CURVE = 1.8;  // Exponential curve factor

void setup() {
  Serial.begin(115200);
  
  // Initialize Bluetooth controller
  bluetoothController.begin();
  
  // Setup PWM for motor control
  // We need to use ledc functions for ESP32
  // Because ESP32 does not support analogWrite like Arduino
  ledcSetup(motorChannel1, pwmFreq, pwmResolution);
  ledcSetup(motorChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(motorPin1, motorChannel1);
  ledcAttachPin(motorPin2, motorChannel2);
  
  Serial.println("RC Car initialized. Waiting for controller connection...");
}

void loop() {
  // Update controller data
  bluetoothController.update();
  
  if (bluetoothController.isControllerConnected()) {
    // Get controller input
    int throttle = bluetoothController.getThrottle();      // 0-1023
    int leftStickY = bluetoothController.getLeftStickY();  // -511 to 512
    
    // Calculate motor speed with exponential curve
    motorSpeed = mapThrottleExponential(throttle);
    
    // Determine direction based on left stick Y axis
    if (leftStickY > 100) {
      // Forward
      motorDirection = true;
    } else if (leftStickY < -100) {
      // Reverse
      motorDirection = false;
    } else {
      // Stop (neutral zone)
      motorSpeed = 0;
    }
    
    // Apply motor control
    controlMotor(motorSpeed, motorDirection);
    
    // Debug output
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.printf("Throttle: %d, LeftY: %d, Speed: %d, Dir: %s\n", 
                    throttle, leftStickY, motorSpeed, motorDirection ? "Forward" : "Reverse");
      lastDebug = millis();
    }
  } else {
    // No controller connected, stop motor
    controlMotor(0, true);
    
    static unsigned long lastMessage = 0;
    if (millis() - lastMessage > 3000) {
      Serial.println("Waiting for controller connection...");
      lastMessage = millis();
    }
  }
  
  delay(50); // Small delay for stability
}

// Exponential throttle mapping function
int mapThrottleExponential(int throttle) {
  if (throttle == 0) return 0;
  if (throttle < 30) return 0;  // Small dead zone
  
  // Normalize throttle to 0-1
  float normalized = (float)(throttle - 30) / (1023.0 - 30.0);
  
  // Apply exponential curve
  float curved = pow(normalized, THROTTLE_CURVE);
  
  // Map to PWM range with minimum threshold
  int pwmValue = (int)(curved * (255 - MIN_MOTOR_PWM)) + MIN_MOTOR_PWM;
  
  return constrain(pwmValue, 0, 255);
}

void controlMotor(int speed, bool direction) {
  if (speed == 0) {
    // Stop motor
    ledcWrite(motorChannel1, 0);
    ledcWrite(motorChannel2, 0);
  } else if (direction) {
    // Forward
    ledcWrite(motorChannel1, speed);
    ledcWrite(motorChannel2, 0);
  } else {
    // Reverse
    ledcWrite(motorChannel1, 0);
    ledcWrite(motorChannel2, speed);
  }
}
