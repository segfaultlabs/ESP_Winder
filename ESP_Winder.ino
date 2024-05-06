#include <Arduino.h>
#include <Preferences.h>  // Include the Preferences library to handle NVS

Preferences preferences;

// Pin definitions
const int relayPin1 = 5;
const int relayPin2 = 18;
const int sensorPin1 = 19;
const int sensorPin2 = 21;

// Rotation count variables
volatile int rotations1 = 0;
volatile int rotations2 = 0;
int targetRotations = 5;

// Flags for ISR to main loop communication
volatile bool rotation1Detected = false;
volatile bool rotation2Detected = false;

// Timing and debouncing variables
volatile unsigned long lastDebounceTime1 = 0;
volatile unsigned long lastDebounceTime2 = 0;
const unsigned long debounceDelay = 50;

unsigned long delayPeriod = 10000;  // Delay between cycles in milliseconds
bool debugMode = true;              // Debug mode status
bool menuActive = false;
unsigned long lastMsgTime1 = 0;
unsigned long lastMsgTime2 = 0;
const unsigned long msgInterval = 1000;  // Time interval in milliseconds (1000 ms = 1 second)
const unsigned long menuTimeout = 10000;  // Timeout period in milliseconds
unsigned long startTime;                 // To track the start time of the menu waiting period
bool menuMode = false;                   // Flag to check if the menu should be active
unsigned long nextRotationTime = 0;  // Time at which the next rotation will occur
const unsigned long countdownInterval = 1000;  // Update the countdown every second
unsigned long lastCountdownTime = 0;  // Last time the countdown was updated

// State machine for controlling the flow of the loop
enum State { IDLE,
             RUNNING,
             WAITING_FOR_NEXT_CYCLE };
State currentState = IDLE;
unsigned long lastCycleTime = 0;

void IRAM_ATTR detectRotation1() {
  if ((millis() - lastDebounceTime1) > debounceDelay) {
    lastDebounceTime1 = millis();
    rotation1Detected = true;
  }
}

void IRAM_ATTR detectRotation2() {
  if ((millis() - lastDebounceTime2) > debounceDelay) {
    lastDebounceTime2 = millis();
    rotation2Detected = true;
  }
}
void setup() {
  Serial.begin(115200);
  while (!Serial);  // Ensure the serial connection is established
  delay(3000); // Wait for the serial monitor to open
  
  Serial.println("\n\nWelcome to the ESP32 Motor Control System!");
  Serial.println("System initializing...");

  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);

  attachInterrupt(digitalPinToInterrupt(sensorPin1), detectRotation1, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensorPin2), detectRotation2, FALLING);

  digitalWrite(relayPin1, HIGH);  // Initially off
  digitalWrite(relayPin2, HIGH);  // Initially off

  nextRotationTime = millis() + delayPeriod;
  lastCountdownTime = millis();

  preferences.begin("settings", false);
  targetRotations = preferences.getInt("targetRotations", 5);
  delayPeriod = preferences.getUInt("delayPeriod", 10000);
  debugMode = preferences.getBool("debugMode", true);

  Serial.println("Press any key within 10 seconds to enter setup menu...");
  startTime = millis();
  menuMode = false;

  while (millis() - startTime < menuTimeout) {
    if (Serial.available()) {
      menuMode = true;
      break;
    }
  }

  if (menuMode) {
    Serial.println("Menu mode activated. Enter a number from below to configure the system.");
    printOptions();
  } else {
    Serial.println("No input detected. Continuing with normal operation...");
  }
}

void loop() {
  if (menuMode) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      handleMenuCommand(input);
    }
  } else {
    // Process rotations based on ISR flags
    if (rotation1Detected) {
      rotations1++;
      if (debugMode && millis() - lastMsgTime1 > msgInterval) {
        Serial.println("Motor 1 (LEFT) count: " + String(rotations1) + " of " + String(targetRotations));
        lastMsgTime1 = millis();
      }
      rotation1Detected = false;
    }

    if (rotation2Detected) {
      rotations2++;
      if (debugMode && millis() - lastMsgTime2 > msgInterval) {
        Serial.println("Motor 2 (RIGHT) count: " + String(rotations2) + " of " + String(targetRotations));
        lastMsgTime2 = millis();
      }
      rotation2Detected = false;
    }

    switch (currentState) {
      case IDLE:
        rotations1 = 0;
        rotations2 = 0;
        digitalWrite(relayPin1, LOW);
        digitalWrite(relayPin2, LOW);
        currentState = RUNNING;
        nextRotationTime = millis() + delayPeriod;  // Set the next rotation start time
        break;

      case RUNNING:
        if (rotations1 >= targetRotations) {
          digitalWrite(relayPin1, HIGH);
          if (debugMode && millis() - lastMsgTime1 > msgInterval) {
            Serial.println("Motor 1 (LEFT) stopped");
            lastMsgTime1 = millis();
          }
        }
        if (rotations2 >= targetRotations) {
          digitalWrite(relayPin2, HIGH);
          if (debugMode && millis() - lastMsgTime2 > msgInterval) {
            Serial.println("Motor 2 (RIGHT) stopped");
            lastMsgTime2 = millis();
          }
        }
        if (rotations1 >= targetRotations && rotations2 >= targetRotations) {
          currentState = WAITING_FOR_NEXT_CYCLE;
          lastCycleTime = millis();
          nextRotationTime = millis() + delayPeriod;  // Set the next rotation start time
          if (debugMode) Serial.println("Both motors stopped. Waiting for next cycle...");
        }
        break;

      case WAITING_FOR_NEXT_CYCLE:
        unsigned long currentTime = millis();
        if (currentTime < nextRotationTime && currentTime - lastCountdownTime >= countdownInterval) {
          unsigned long secondsUntilNextRotation = (nextRotationTime - currentTime) / 1000;
          Serial.print("Next rotation in: ");
          Serial.print(secondsUntilNextRotation);
          Serial.println(" seconds");
          lastCountdownTime = currentTime;
        }
        if (currentTime >= nextRotationTime) {
          if (debugMode) Serial.println("Delay complete. Restarting motors...");
          currentState = IDLE;
        }
        break;
    }
  }
}



void printOptions() {
    Serial.println("\nMenu Options:");
    Serial.println("  1 - Set number of rotations");
    Serial.println("  2 - Set delay period (seconds)");
    Serial.println("  3 - Toggle debug mode");
    Serial.println("  4 - Print current settings");
    Serial.println("  5 - Exit menu mode");
    Serial.println("  6 - Zero motors"); 
}

void handleMenuCommand(String input) {
    int command = input.toInt(); // Convert input to integer

    switch (command) {
        case 1: // Set number of rotations
            Serial.print("Enter new number of rotations: ");
            while (!Serial.available());
            targetRotations = Serial.readStringUntil('\n').toInt();
            preferences.putInt("targetRotations", targetRotations);
            Serial.println("Target rotations set to: " + String(targetRotations));
            printCurrentSettings();
            printOptions();
            break;

        case 2: // Set delay period in seconds
            Serial.print("Enter new delay period in seconds: ");
            while (!Serial.available());
            delayPeriod = Serial.readStringUntil('\n').toInt() * 1000; // Convert seconds to milliseconds
            preferences.putUInt("delayPeriod", delayPeriod);
            Serial.println("Delay period set to: " + String(delayPeriod / 1000) + " seconds");
            printCurrentSettings();
            printOptions();
            break;

        case 3: // Toggle debug mode
            debugMode = !debugMode;
            preferences.putBool("debugMode", debugMode);
            Serial.println("Debug mode " + String(debugMode ? "enabled" : "disabled"));
            printCurrentSettings();
            printOptions();
            break;

        case 4: // Print current settings
            printCurrentSettings();
            printOptions();
            break;

        case 5: // Exit menu
            menuMode = false;
            Serial.println("Exiting menu mode...");
            break;

        case 6: // Zero motors
            Serial.println("Zeroing motors...");
            zeroMotors();
            printOptions();
            break;    

        default:
            Serial.println("Invalid option. Please try again.");
            printOptions();
            break;
    }
}


void printCurrentSettings() {
    // Read settings from NVS
    int storedRotations = preferences.getInt("targetRotations", 5);
    unsigned long storedDelay = preferences.getUInt("delayPeriod", 10000);
    bool storedDebugMode = preferences.getBool("debugMode", true);

    // Print the settings
    Serial.println("\nCurrent Saved Settings:");
    Serial.println("  Rotations: " + String(storedRotations));
    Serial.println("  Delay Period (s): " + String(storedDelay / 1000)); // Convert ms to seconds for display
    Serial.println("  Debug Mode: " + String(storedDebugMode ? "Enabled" : "Disabled"));
}

void zeroMotors() {
    // Zero Motor 1
    digitalWrite(relayPin1, LOW);  // Assuming LOW starts the motor
    while (!rotation1Detected);  // Wait until the hall effect sensor triggers
    digitalWrite(relayPin1, HIGH);  // Stop the motor
    rotation1Detected = false;  // Reset the detection flag

    // Give a small delay between motor operations
    delay(1000);  // Adjust delay as needed for your application

    // Zero Motor 2
    digitalWrite(relayPin2, LOW);  // Start Motor 2
    while (!rotation2Detected);  // Wait for the sensor to trigger
    digitalWrite(relayPin2, HIGH);  // Stop the motor
    rotation2Detected = false;  // Reset the detection flag

    Serial.println("Motors zeroed successfully.");
}
