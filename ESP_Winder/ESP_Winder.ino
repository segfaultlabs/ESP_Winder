#include <Wire.h>               // For I2C communication with the LCD
#include <LiquidCrystal_I2C.h>  // Library for LCD
#include <EEPROM.h>             // For storing settings in EEPROM

// Pins for ESP32DevKit board
const int ROTARY_ENCODER_CLK_PIN = 32;    // CLK pin for rotary encoder
const int ROTARY_ENCODER_DT_PIN = 33;     // DT pin for rotary encoder
const int ROTARY_ENCODER_BUTTON_PIN = 25; // A pin for rotary encoder button
const int RELAY_PIN_1 = 26;               // GPIO pin for relay 1
const int RELAY_PIN_2 = 27;               // GPIO pin for relay 2
const int HALL_SENSOR_PIN_1 = 14;         // Pin for hall sensor 1
const int HALL_SENSOR_PIN_2 = 12;         // Pin for hall sensor 2
const int LCD_ADDRESS = 0x27;             // I2C address of the LCD (uses GPIO 21, 22 as SDA, SCL)

// SDA (Serial Data Line): GPIO 21 (often labeled as pin 21 or simply labeled as SDA)
// SCL (Serial Clock Line): GPIO 22 (often labeled as pin 22 or simply labeled as SCL)
// const int LCD_ADDRESS = 0x27;  // I2C address of the LCD

// LCD setup
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

// Global variables
int rotations = 0;  // Current rotation count
int setRotations = 5;  // User-defined number of rotations
int interval = 0;  // User-defined interval between rotations
unsigned long lastRotationTime = 0;  // Last time a rotation was completed
unsigned long lastScreenUpdateTime = 0; // Global variable to store the last time the screen was updated
int rotationsWatch1 = 0;           // Rotation count for watch 1
int rotationsWatch2 = 0;           // Rotation count for watch 2

// Variables to store previous state of the encoder
int lastEncoderState = 0;
int lastButtonState = HIGH;
unsigned long lastButtonPressTime = 0;


// Menu state
enum MenuState { MAIN, SET_ROTATIONS, SET_INTERVAL };
MenuState menuState = MAIN;

// Function prototypes
void displayMainScreen();
void displaySetRotationsScreen();
void displaySetIntervalScreen();
void readRotaryEncoder();
void handleMenuNavigation();
void handleMenuAdjustment();

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(11520);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize rotary encoder pins
  pinMode(ROTARY_ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_DT_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP);

  // Initialize relay pins
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);

  // Initialize hall sensor pins
  pinMode(HALL_SENSOR_PIN_1, INPUT);
  pinMode(HALL_SENSOR_PIN_2, INPUT);

  // Display welcome message
  lcd.clear();
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("Watch Winder!");
  delay(1000);

  // Read settings from EEPROM
  EEPROM.begin(512);
  EEPROM.get(0, setRotations);
  EEPROM.get(sizeof(int), interval);
  EEPROM.end();

  // Set default values if EEPROM is empty or values are 0
  if (setRotations <= 0) {
    setRotations = 5; // Default to 5 rotations
  }
  if (interval <= 0) {
    interval = 60; // Default to 60 seconds (1 minute interval)
  }

  // Display main screen
  displayMainScreen();
}

void loop() {
  if (menuState == MAIN || menuState == SET_ROTATIONS || menuState == SET_INTERVAL) {
    readRotaryEncoder();
    handleMenuNavigation();
    handleMenuAdjustment();

    // Check if it's time for a rotation
    if (millis() - lastRotationTime >= interval * 1000 && rotations < setRotations) {
      // Perform rotation
      // Code to activate relays and count rotations
      rotations++;
      lastRotationTime = millis();
    }

    // Update the screen once per second
    unsigned long currentTime = millis();
    if (currentTime - lastScreenUpdateTime >= 1000) {
      if (menuState == MAIN) {
        displayMainScreen();
      } else if (menuState == SET_ROTATIONS) {
        displaySetRotationsScreen();
      } else if (menuState == SET_INTERVAL) {
        displaySetIntervalScreen();
      }
      lastScreenUpdateTime = currentTime;
    }
  }
  delay(1);
}

// Function to read hall sensor values and update rotation count
void readHallSensors() {
  // Read hall sensor values for each watch
  int hallSensor1 = digitalRead(HALL_SENSOR_PIN_1);
  int hallSensor2 = digitalRead(HALL_SENSOR_PIN_2);
  
  // Update rotation count for each watch if a rotation is detected
  if (hallSensor1 == HIGH) {
    rotationsWatch1++;
  }
  if (hallSensor2 == HIGH) {
    rotationsWatch2++;
  }
}

// Function to control the relays
void controlRelays() {
  // Turn on/off relays based on rotation count for each watch
  if (rotationsWatch1 < setRotations) {
    digitalWrite(RELAY_PIN_1, HIGH);  // Turn on relay 1
  } else {
    digitalWrite(RELAY_PIN_1, LOW);   // Turn off relay 1
  }
  
  if (rotationsWatch2 < setRotations) {
    digitalWrite(RELAY_PIN_2, HIGH);  // Turn on relay 2
  } else {
    digitalWrite(RELAY_PIN_2, LOW);   // Turn off relay 2
  }
}

void saveSettingsToEEPROM() {
  EEPROM.begin(512);
  EEPROM.put(0, setRotations);
  EEPROM.put(sizeof(int), interval);
  EEPROM.commit();
  EEPROM.end();
}

void loadSettingsFromEEPROM() {
  EEPROM.begin(512);
  EEPROM.get(0, setRotations);
  EEPROM.get(sizeof(int), interval);
  EEPROM.end();
}

void displayMainScreen() {
  lcd.clear();
  lcd.print("Rotation: ");
  lcd.print(rotations);
  lcd.print(" of ");
  
  // Check if setRotations is less than 0, if so, display 0 instead
  if(setRotations < 0) {
    lcd.print("0");
  } else {
    lcd.print(setRotations);
  }

  lcd.setCursor(0, 1);
  lcd.print("Next in: ");
  lcd.print((interval * 1000 - (millis() - lastRotationTime)) / 1000);
  lcd.print(" sec");
}

void displaySetRotationsScreen() {
  lcd.clear();
  lcd.print("Set Rotations: ");
  lcd.print(setRotations);

  // Display cursor position based on menuState
  lcd.setCursor(0, 1);
  if (menuState == SET_ROTATIONS) {
    lcd.print(">");
  }
}

void displaySetIntervalScreen() {
  lcd.clear();
  lcd.print("Set Interval: ");
  lcd.print(interval);

  // Display cursor position based on menuState
  lcd.setCursor(0, 1);
  if (menuState == SET_INTERVAL) {
    lcd.print(">");
  }
}

void handleMenuNavigation() {
  // static int lastEncoded = 0;
  // static long lastMillis = 0;
  // int newEncoded = 0;
  // long millisNow = millis();
  // int direction = 0;
  
  // // Read the current state of the rotary encoder
  // int MSB = digitalRead(ROTARY_ENCODER_CLK_PIN);
  // int LSB = digitalRead(ROTARY_ENCODER_DT_PIN);
  // newEncoded = (MSB << 1) | LSB;
  // int sum = (lastEncoded << 2) | newEncoded;
  
  // // Determine the direction of rotation
  // if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
  //   direction = 1; // Clockwise
  // } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
  //   direction = -1; // Counter-clockwise
  // }
  
  // // Handle menu navigation
  // if (millisNow - lastMillis > 100) { // Debounce
  //   if (direction == 1) { // Clockwise
  //     if (menuState == MAIN) {
  //       menuState = SET_ROTATIONS;
  //     } else if (menuState == SET_ROTATIONS) {
  //       menuState = SET_INTERVAL;
  //     }
  //   } else if (direction == -1) { // Counter-clockwise
  //     if (menuState == SET_INTERVAL) {
  //       menuState = SET_ROTATIONS;
  //     } else if (menuState == SET_ROTATIONS) {
  //       menuState = MAIN;
  //     }
  //   }
  //   lastMillis = millisNow;
  // }
  // lastEncoded = newEncoded;
}

void handleMenuAdjustment() {
  // Read the current state of the rotary encoder button
  int buttonState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
  static int lastButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 50;
  unsigned long currentTime = millis();

  // Check if the button is pressed and handle adjustment accordingly
  if (buttonState != lastButtonState) {
    lastDebounceTime = currentTime;
  }

  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (buttonState == LOW) { // Button is pressed
      if (menuState == SET_ROTATIONS) {
        // Increase or decrease setRotations value based on rotary encoder direction
        if (rotations < 0xFFFF) { // Ensure value does not overflow
          setRotations += (rotations == 0 ? 1 : -1);
          // Update the current rotation count
          rotations = setRotations; // Update rotations whenever setRotations changes
        }
        displaySetRotationsScreen();
      } else if (menuState == SET_INTERVAL) {
        // Increase or decrease interval value based on rotary encoder direction
        interval += (interval == 0 ? 1 : -1);
        displaySetIntervalScreen();
      }
    }
  }
  lastButtonState = buttonState;
}

void readRotaryEncoder() {
  static int lastEncoded = 0;
  static unsigned long lastDebounceTime = 0;
  int newEncoded = 0;

  // Read the current state of the rotary encoder
  int MSB = digitalRead(ROTARY_ENCODER_CLK_PIN);
  int LSB = digitalRead(ROTARY_ENCODER_DT_PIN);
  newEncoded = (MSB << 1) | LSB;

  // Apply debounce logic
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > 5) {
    if (newEncoded != lastEncoded) {
      // Determine the direction of rotation
      int direction = (lastEncoded == 0b00 && newEncoded == 0b01) ||
                      (lastEncoded == 0b01 && newEncoded == 0b11) ||
                      (lastEncoded == 0b11 && newEncoded == 0b10) ||
                      (lastEncoded == 0b10 && newEncoded == 0b00) ? 1 : -1;

      // Handle rotation direction
      if (menuState == SET_ROTATIONS) {
        if (direction == 1) {
          setRotations++;
        } else {
          setRotations--;
        }
        // Ensure setRotations stays within bounds
        if (setRotations < 0) {
          setRotations = 0;
        }
        // Update the display
        displaySetRotationsScreen();
      }

      lastEncoded = newEncoded;
    }

    // Check if the button is pressed
    int buttonState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
    if (buttonState == LOW) {
      // Button is pressed
      if (menuState == MAIN) {
        menuState = SET_ROTATIONS;
        displaySetRotationsScreen();
      } else if (menuState == SET_ROTATIONS) {
        // Save settings and return to main menu
        saveSettingsToEEPROM();
        menuState = MAIN;
        displayMainScreen();
      }
      // Add debounce delay here if needed
    }

    lastDebounceTime = currentTime;
  }
}



