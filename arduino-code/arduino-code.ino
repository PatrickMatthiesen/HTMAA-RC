#include "BluetoothController.h"
#include "BuzzerPlayer.h"
#include <ESP32Servo.h>
#include <Arduino.h>
#include <esp32-hal-gpio.h>

// Motor control pins
const int motorPin1 = 25; // A-1A // 18
const int motorPin2 = 33; // A-1B // 19
const int enablePin = 32; // A-ENA // 23

// PWM channels for ESP32
// const int motorChannel1 = 2; // I think channel 0 is being used by the servo, and that messes stuff up
// const int motorChannel2 = 3;
const int motorPwmChannel = 2; // Channel for motor pin 1
const int pwmFreq = 1000;
const int pwmResolution = 8;

// Motor control variables
int motorSpeed = 0;
bool motorDirection = true; // true = forward, false = reverse

// Throttle control parameters
const int MIN_MOTOR_PWM = 150;  // Minimum PWM to overcome friction

// Steering servo
Servo servo;
const int steeringPin = 13; // Pin for steering servo
const int steeringMin = 50; // Minimum angle for steering
const int steeringMax = 180 - steeringMin; // Maximum angle for steering
const int steeringDeadzone = 80; // Deadzone for incorrect controller input

// Buzzer
const int buzzerPin = 12; // Pin for buzzer
const int buzzerChannel = 4; // Buzzer channel for ESP32
const int buzzerFreq = 1000; // Frequency for buzzer
PresetMelody buzzerMelody = PresetMelody::MELODY_NONE; // Default melody to play

BuzzerPlayer buzzer(buzzerPin, buzzerChannel);

// Ultra-sonic sensors
const int forwardTrig = 5;
const int forwardEcho = 17;
const int leftTrig = 0; // 0
const int leftEcho = 2; // 2
const int rightTrig = 14;
const int rightEcho = 27;
const int backTrig = 16;
const int backEcho = 4;

// Collision avoidance parameters
const float COLLISION_DISTANCE = 15.0; // Stop if obstacle within 15cm
const float WARNING_DISTANCE = 25.0;   // Warn if obstacle within 25cm
const int COLLISION_TIMEOUT = 10000; // Collision detection timeout in ms (about 1m range)
int collisionEnabled = 1; // Enable collision detection by default

// debug variables
static unsigned long lastDebug = 0;

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
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  ledcSetup(motorPwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(enablePin, motorPwmChannel);

  // Initialize Ultra-sonic sensors
  pinMode(forwardTrig, OUTPUT);
  pinMode(forwardEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(rightTrig, OUTPUT);
  pinMode(rightEcho, INPUT);
  pinMode(backTrig, OUTPUT);
  pinMode(backEcho, INPUT);

  // Initialize steering servo
  // servo.setPeriodHertz(50);      // Standard 50hz servo
  int channel = servo.attach(steeringPin);
  Serial.println("Steering servo initialized on channel " + String(channel));


  Serial.println("RC Car initialized. Waiting for controller connection...");
  buzzer.playPresetMelody(MELODY_BEEP_SUCCESS); // Play startup melody
}

void loop() {
  // Update controller data
  bluetoothController.update();
  
  if (!bluetoothController.isControllerConnected()) {
    // No controller connected, stop motor
    controlMotor(0, true);
    buzzer.update();
    
    static unsigned long lastMessage = 0;
    if (millis() - lastMessage > 3000) {
      Serial.println("Waiting for controller connection...");
      lastMessage = millis();
    }
    delay(100);
    return; // Skip the rest of the loop if no controller is connected
  }

  // Get controller input
  int throttle = bluetoothController.getThrottle();      // 0-1023
  int leftStickY = bluetoothController.getLeftStickY();  // -511 to 512
  int leftStickX = bluetoothController.getLeftStickX();  // -511 to 512
  int dpad = bluetoothController.getDpad();

  // Change buzzer melody if D-pad buttons are pressed
  updateBuzzerMelody(dpad);

  if (bluetoothController.isButtonPressed(BUTTON_A)) {
    collisionEnabled = !collisionEnabled; // Toggle collision detection
    Serial.println("Collision detection " + String(collisionEnabled ? "enabled" : "disabled"));
  }

  // Calculate motor speed based on throttle
  motorSpeed = throttle == 0 ? 0 : map(throttle, 0, 1023, MIN_MOTOR_PWM, 255);
  
  // Determine direction based on left stick Y axis
  if (leftStickY < -50) {
    // Forward
    motorDirection = true;
  } else if (leftStickY > 50) {
    // Reverse
    motorDirection = false;
  } else {
    // Stop (neutral zone)
    motorSpeed = 0;
  }
  
  // Read ultrasonic sensors
  float forwardDistance = readUltrasonicDistance(forwardTrig, forwardEcho);
  float leftDistance = readUltrasonicDistance(leftTrig, leftEcho);
  float rightDistance = readUltrasonicDistance(rightTrig, rightEcho);
  float backDistance = readUltrasonicDistance(backTrig, backEcho);

  bool collisionDetected = false;
  char collisionDirection = -1;
  if (motorDirection && (forwardDistance < COLLISION_DISTANCE)) {
    collisionDetected = true;
    collisionDirection = 'F';
  } else if (!motorDirection && (backDistance < COLLISION_DISTANCE)) {
    collisionDetected = true;
    collisionDirection = 'B';
  } else if ((motorDirection && (forwardDistance < WARNING_DISTANCE)) 
          || (!motorDirection && (backDistance < WARNING_DISTANCE))) {
    if (millis() - lastDebug > 500) {
      // buzzer.playPresetMelody(MELODY_BEEP_WARNING); // Play warning sound
      Serial.println("Obstacle detected! Warning sound played.");
    }
  }

  // Override motor control if collision detected
  if (collisionDetected && collisionEnabled) {
    if (millis() - lastDebug > 500) {
      Serial.print(collisionDirection == 'F' ? "Forward" : "Reverse");
      Serial.println(" collision detected! Stopping motor.");
      buzzer.playPresetMelody(MELODY_BEEP_ERROR); // Play error sound
    }

    controlMotor(0, true); // Stop motor
  } else {
    controlMotor(motorSpeed, motorDirection);
  }

  // Map left stick X to steering angle
  // Use a deadzone to avoid small stick movements affecting steering
  if (abs(leftStickX) < steeringDeadzone) {
    leftStickX = 0; // Deadzone for steering
  }
  int angle = map(leftStickX, -511, 512, steeringMax, steeringMin);
  // Set steering servo position
  servo.write(angle);

  // Debug output
  if (millis() - lastDebug > 500) {
    Serial.printf("\nThrottle: %d, LeftX: %d, LeftY: %d, Speed: %d, Dir: %s, Collision: %s, Angle: %d\n",
                  throttle, leftStickX, leftStickY, motorSpeed, motorDirection ? "Forward" : "Reverse", collisionDetected ? "Yes" : "No", angle);
    Serial.printf("Distances - Forward: %.2f cm, Left: %.2f cm, Right: %.2f cm, Back: %.2f cm\n",
                  forwardDistance, leftDistance, rightDistance, backDistance);

    if (bluetoothController.isButtonPressed(BUTTON_A) && bluetoothController.isButtonPressed(BUTTON_B)) {
      Serial.println("Button A and B pressed");
      bluetoothController.dumpGamepads();
    }
    lastDebug = millis();
  }

  delay(50); // Small delay for stability
}

void controlMotor(int speed, bool direction) {
  if (speed == 0) {
    // Stop motor
    ledcWrite(motorPwmChannel, 0); // Set PWM to 0
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  } else if (direction) {
    // Forward
    ledcWrite(motorPwmChannel, speed); // Set PWM to speed
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    // Reverse
    ledcWrite(motorPwmChannel, speed); // Set PWM to speed
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
}

void updateBuzzerMelody(int dpad) {
  bool leftDpad = (dpad & DPAD_LEFT) != 0;
  bool rightDpad = (dpad & DPAD_RIGHT) != 0;

  if (leftDpad && rightDpad) {
    buzzer.playPresetMelody(MELODY_NONE); // Stop playing melody
    Serial.println("Buzzer stopped.");
    return;
  } else if (leftDpad) {
    buzzerMelody = static_cast<PresetMelody>(abs((buzzerMelody - 1) % (MELODY_BEEP_ERROR + 1))); 
    Serial.print("Left D-Pad: Pressed");
  } else if (rightDpad) {
    buzzerMelody = static_cast<PresetMelody>((buzzerMelody + 1) % (MELODY_BEEP_ERROR + 1));
    Serial.print("Right D-Pad: Pressed");
  } else if (buzzer.isPlayingTone()) { // remove this if we dont want it to keep playing the last melody
    buzzer.update(); // Just update the buzzer state if no buttons pressed
    return;
  } else if (buzzerMelody < 1) {
    return;
  }

  buzzer.playPresetMelody(buzzerMelody);
  Serial.println("Changed buzzer melody to: " + String(static_cast<int>(buzzerMelody)));
}

// Add this new function
float readUltrasonicDistance(int trigPin, int echoPin) {
  // Send 10us pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo time with timeout
  unsigned long duration = pulseIn(echoPin, HIGH, COLLISION_TIMEOUT);

  // Convert to distance in cm (sound travels at ~343m/s)
  // Distance = (duration * 0.034) / 2
  if (duration == 0) {
    return 999.0; // No echo received, assume clear path
  }
  
  return (duration * 0.034) / 2.0;
}
