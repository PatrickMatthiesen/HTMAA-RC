#include "BluetoothController.h"
#include "BuzzerPlayer.h"
#include <ESP32Servo.h>
#include <Arduino.h>

// Motor control pins
const int motorPin1 = 18; // A-1A
const int motorPin2 = 19; // A-1B

// PWM channels for ESP32
const int motorChannel1 = 2; // I think channel 0 is being used by the servo, and that messes stuff up
const int motorChannel2 = 3;
const int pwmFreq = 1000;
const int pwmResolution = 8;

// Motor control variables
int motorSpeed = 0;
bool motorDirection = true; // true = forward, false = reverse

// Throttle control parameters
const int MIN_MOTOR_PWM = 0;  // Minimum PWM to overcome friction
const float THROTTLE_CURVE = 1.8;  // Exponential curve factor

// Steering servo
Servo servo;
const int steeringPin = 13; // Pin for steering servo
const int steeringMin = 30; // Minimum angle for steering
const int steeringMax = 180 - steeringMin; // Maximum angle for steering

// Buzzer
const int buzzerPin = 12; // Pin for buzzer
const int buzzerChannel = 4; // Buzzer channel for ESP32
const int buzzerFreq = 1000; // Frequency for buzzer
PresetMelody buzzerMelody = PresetMelody::MELODY_NONE; // Default melody to play

BuzzerPlayer buzzer(buzzerPin, buzzerChannel);

void setup() {
  Serial.begin(115200);

  // Allow allocation of all timers - Something from the servo examples
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  // Initialize Bluetooth controller
  bluetoothController.begin();
  
  // Setup PWM for motor control
  // We need to use ledc functions for ESP32
  // Because ESP32 does not support analogWrite like Arduino
  ledcSetup(motorChannel1, pwmFreq, pwmResolution);
  ledcSetup(motorChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(motorPin1, motorChannel1);
  ledcAttachPin(motorPin2, motorChannel2);

  // Initialize steering servo
  // servo.setPeriodHertz(50);      // Standard 50hz servo
  int channel = servo.attach(steeringPin);
  Serial.println("Steering servo initialized on channel " + String(channel));


  Serial.println("RC Car initialized. Waiting for controller connection...");
}

void loop() {
  // Update controller data
  bluetoothController.update();
  
  if (bluetoothController.isControllerConnected()) {
    // Get controller input
    int throttle = bluetoothController.getThrottle();      // 0-1023
    int leftStickY = bluetoothController.getLeftStickY();  // -511 to 512
    int leftStickX = bluetoothController.getLeftStickX();  // -511 to 512
    
    bluetoothController.dumpGamepads();

    int dpad = bluetoothController.getDpad();
    bool leftDpad = (dpad & DPAD_LEFT) != 0;
    bool rightDpad = (dpad & DPAD_RIGHT) != 0;

    Serial.print("Left D-Pad: ");
    Serial.print(leftDpad ? "Pressed" : "Not Pressed");
    Serial.print(", Right D-Pad: ");
    Serial.println(rightDpad ? "Pressed" : "Not Pressed");
    // Change buzzer melody if D-pad buttons are pressed
    changeBuzzerMelody(leftDpad, rightDpad);

    // Calculate motor speed with exponential curve
    motorSpeed = map(throttle, 0, 1023, MIN_MOTOR_PWM, 255);
    int motorSpeedTest = mapThrottleExponential(throttle);
    
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

    // Map left stick X to steering angle
    int angle = map(leftStickX, -511, 512, steeringMin, steeringMax);
    // Set steering servo position
    servo.write(angle);

    // Debug output
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
      Serial.printf("Throttle: %d, LeftX: %d, LeftY: %d, Speed: %d, MotorSpeedTest: %d, Dir: %s, Angle: %d\n",
                    throttle, leftStickX, leftStickY, motorSpeed, motorSpeedTest, motorDirection ? "Forward" : "Reverse", angle);
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
  if (throttle > 1023) throttle = 1023; // Clamp to max value
  
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

void changeBuzzerMelody(bool leftDpad, bool rightDpad) {
  if (leftDpad && rightDpad) {
    buzzer.playPresetMelody(MELODY_NONE); // Stop playing melody
    Serial.println("Buzzer stopped.");
    return;
  } else if (leftDpad) {
    buzzerMelody = static_cast<PresetMelody>((buzzerMelody - 1) % (MELODY_BEEP_ERROR + 1));
  } else if (rightDpad) {
    buzzerMelody = static_cast<PresetMelody>((buzzerMelody + 1) % (MELODY_BEEP_ERROR + 1));
  } else {
    buzzer.update(); // Just update the buzzer state if no buttons pressed
    return;
  }

  buzzer.playPresetMelody(buzzerMelody);
  Serial.println("Changed buzzer melody to: " + String(static_cast<int>(buzzerMelody)));
}