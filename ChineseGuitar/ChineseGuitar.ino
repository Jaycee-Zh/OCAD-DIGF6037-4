/*********************************************************************
 * DF Pong Controller
 *
 * This program implements a Bluetooth Low Energy controller for Pong.
 * It sends movement data to a central device running in the browser and
 * provides audio feedback through a buzzer.
 *
 * Game Link : https://digitalfuturesocadu.github.io/df-pong/
 *
 * Movement Values:
 * 0 = No movement / Neutral position
 * 1 = UP movement (paddle moves up)
 * 2 = DOWN movement (paddle moves down)
 * 3 = Handshake signal (used for initial connection verification)
 *
 * Key Functions:
 * - handleInput(): Process the inputs to generate the states
 * - sendMovement(): Sends movement data over BLE (0-3)
 * - updateBLE(): Handles BLE connection management and updates
 * - updateBuzzer(): Provides different buzzer patterns for different movements
 *
 * Key Variables:
 * - currentMovement: Stores current movement state (0-2)
 * - deviceName : GIVE YOUR DEVICE AN APPROPRIATE NAME
 * - LED_PIN : It is important to see the status of the arduino through the LED.
      if you can see the built-in add an external one and update the pin it is connected to
 *
 * References:
 * smooth the light sensor value: https://github.com/DigitalFuturesOCADU/CC2024/tree/main/experiment4/Arduino/Sensors/Light/ligh_raw_smoothed
 * capacitive sensors: https://github.com/DigitalFuturesOCADU/CC2024/tree/main/experiment4/Arduino/BeepBoop/touch5_BeepBoop
 * capacitive sensors: https://github.com/DigitalFuturesOCADU/CC2024/blob/main/experiment4/Arduino/BeepBoop/touch8_BeepBoop

 *********************************************************************/

// ------ the template starts------

#include <ArduinoBLE.h>
#include "ble_functions.h"
#include "buzzer_functions.h"
//Since code is split over multiple files, we have to include them here


//Name your controller!
const char* deviceName = "C&C: Chinese Guitar";

// Pin definitions buzzer/LED
const int BUZZER_PIN = 11;        // Pin for haptic feedback buzzer
const int LED_PIN = LED_BUILTIN;  // Status LED pin

// Movement state tracking
int currentMovement = 0;  // Current movement value (0=none, 1=up, 2=down, 3=handshake)

// ------ the template ends------

// ------ my codes starts------

// libraries for CAP1188
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>


// ------ Hardware

// for light sensors
const int lightSensorPinNum = 2;           // Number of pins for light sensors; for loop in array
const int lightSensorPins[] = { A7, A6 };  // Pins for light sensors
int smoothedLightValues[2] = { 0 };        // Filtered value
int startupLightValues[2] = { 0 };         // Calibration value from startup

// for touch sensors
#define CAP1188_RESET 12                                        //define pin
Adafruit_CAP1188 touchInput = Adafruit_CAP1188(CAP1188_RESET);  // Create the touch sensor object with reset pin
const int touchSensorPinNum = 4;  // Number of pins for touch sensors; for loop in array

// for LED out
// const int ledPinNum = 4;           // Number of pins for light sensors; for loop in array
// const int ledPins[] = {2, 3, 4, 5 };  // Pins for light sensors


// ------ Settings // change settings to change the UX
unsigned int readInterval = 50;         // Time between reads in milliseconds
const int lightReadAverageWindow = 10;  // Number of samples to average
const int lightSensorThreshold = 100;   // Tolerance for considering values "same"

// ------ Variables
unsigned long lastReadTime = 0;  // for control reading rate
// for light sensors
int lightValues[2] = { 0 };  // Raw value
int lightReadings[lightSensorPinNum][lightReadAverageWindow]; // Rolling average variables
int lightReadIndex[] = { 0, 0 };
long lightTotalValues[2] = { 0 };
// for touch sensors
bool touchStates[8] = { false };  // for touching the strings (left hand)
bool prevTouchStates[8] = { false };


// Input
// bool stringIsPressed[4] = { false };  // for touching the strings (left hand)
bool lightIsPressed[2] = { false };  // for pulling the strings: up and down
int lightDifferences[2] = { 0 };     // how much the light changed


// ------ my codes ends ------

// ------ my codes starts------

// Functions

// Function to initialize the rolling average array: reset all the variables back to 0
void initializeLightAverage() {
  // Initialize all readings to 0
  for (int i = 0; i < lightSensorPinNum; i++) {
    for (int j = 0; j < lightReadAverageWindow; j++) {
      lightReadings[i][j] = 0;
    }
    lightTotalValues[i] = 0;
    // lightReadIndex = 0;
  }
}

// Function to calibrate the sensors
void calibrateSensor() {
  Serial.println("Calibrating sensor...");
  // Take multiple readings and average them for startup value
  for (int i = 0; i < lightSensorPinNum; i++) {
    long total = 0;
    for (int j = 0; j < lightReadAverageWindow; j++) {
      total += analogRead(lightSensorPins[i]);
      delay(50);  // Short delay between readings
    }
    startupLightValues[i] = total / lightReadAverageWindow;
    Serial.print("Calibration complete. Startup value: ");
    Serial.print(i);
    Serial.println(startupLightValues[i]);
  }
}

// Function to update rolling average with new value
void updateLightAverage() {
  for (int i = 0; i < lightSensorPinNum; i++) {
    // Read the analog value
    int newValue = analogRead(lightSensorPins[i]);
    //
    lightTotalValues[i] -= lightReadings[i][lightReadIndex[i]];
    lightReadings[i][lightReadIndex[i]] = newValue;
    lightTotalValues[i] += newValue;
    lightReadIndex[i] = (lightReadIndex[i] + 1) % lightReadAverageWindow;
    smoothedLightValues[i] = lightTotalValues[i] / lightReadAverageWindow;
  }
}


// Function to print light sensor values
void printLightValue() {
  Serial.print("Light Raw A: ");
  Serial.print(lightValues[0]);
  Serial.print("\tLight Smoothed A: ");
  Serial.print(smoothedLightValues[0]);
  Serial.print("\tStartup Value A: ");
  Serial.println(startupLightValues[0]);
  Serial.print("Light Raw B: ");
  Serial.print(lightValues[1]);
  Serial.print("\tLight Smoothed B: ");
  Serial.print(smoothedLightValues[1]);
  Serial.print("\tStartup Value B: ");
  Serial.println(startupLightValues[1]);
  // Serial.print("\tCompared to Startup: ");
  // Serial.println(brightnessState);
}



// ------ my codes ends ------


void setup() {
  // ------ the template starts------

  Serial.begin(9600);

  // Configure LED for connection status indication
  pinMode(LED_PIN, OUTPUT);

  // Initialize Bluetooth Low Energy with device name and status LED
  setupBLE(deviceName, LED_PIN);

  // Initialize buzzer for feedback
  setupBuzzer(BUZZER_PIN);

  // ------ the template ends------

  // ------ my codes starts------

  // Initialize touch sensor
  if (!touchInput.begin()) {
    Serial.println("CAP1188 not found");
    while (1)
      ;
  }
  Serial.println("CAP1188 found!");

  // Initialize the rolling average
  initializeLightAverage();

  // Perform startup calibration
  calibrateSensor();

  // ------ my codes ends ------
}

void loop() {
  // ------ the template starts------

  // Update BLE connection status and handle incoming data
  updateBLE();

  //read the inputs te determine the current state
  //results in changing the value of currentMovement
  handleInput();

  //send the movement state to P5
  sendMovement(currentMovement);

  //make the correct noise
  updateBuzzer(currentMovement);

  // ------ the template ends------
}

void handleInput() {
  //put code here that reads the sensor input
  //and assigns currentMovement(0=stop, 1=up, 2=down)

  // ------ my codes starts------
  unsigned long currentTime = millis();              // get currentTime
  if (currentTime - lastReadTime >= readInterval) {  // if it has passed the calibration time

    // Read the analog value
    for (int i = 0; i < lightSensorPinNum; i++) {
      lightValues[i] = analogRead(lightSensorPins[i]);
    };

    // Update the rolling average
    updateLightAverage();

    // Store previous touch states
    for (uint8_t i = 0; i < 8; i++) {
      prevTouchStates[i] = touchStates[i];
    }

    // Read all touches at once
    uint8_t touched = touchInput.touched();

    // Convert the bit mask to individual boolean values
    for (uint8_t i = 0; i < 8; i++) {
      touchStates[i] = (touched & (1 << i)) != 0;
    }

    //Compare to startup value
    for (int i = 0; i < lightSensorPinNum; i++) {
      lightDifferences[i] = startupLightValues[i] - lightValues[i];
      if (lightDifferences[i] > lightSensorThreshold) {  // if darker than the beginning
        lightIsPressed[i] = true;                        // consider the button as being pressed
        // Serial.println(i);
        // Serial.print("/t is pressed");
      } else {
        lightIsPressed[i] = false;
      }
    }

    // if all the capacitive sensors are touched, send out number
    if (touchStates[0] && touchStates[1] && touchStates[2] && touchStates[3]) {
      if (lightIsPressed[0] && !lightIsPressed[1]) {  // if up is pressed, and down is not
        currentMovement = 1;                          // go up
        Serial.println("up");
      } else if (!lightIsPressed[0] && lightIsPressed[1]) {  // if down is pressed, and up is not
        currentMovement = 2;                                 // go down
        Serial.println("down");
      } else {  // if both are pressed, or both are not, stay
        currentMovement = 0;
      }
    } else {
      currentMovement = 0;
    }

    // Print the values
    // printLightValue();

    // Update the last read time
    lastReadTime = currentTime;
  }
  // ------ my codes ends ------
}
