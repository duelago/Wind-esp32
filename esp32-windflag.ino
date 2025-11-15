#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <Arduino_GFX_Library.h>
#include <AccelStepper.h>
#include <SPI.h>

#define LED_PIN 8
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// ---------------- Display Configuration ----------------
// ESP32-C6-Touch-LCD-1.47 Correct Pins
#define TFT_CS   14
#define TFT_DC   15
#define TFT_RST  21
#define TFT_SCK  7
#define TFT_MOSI 6
#define TFT_BL   22

#define SCREEN_WIDTH 400
#define SCREEN_HEIGHT 320

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, -1);
Arduino_GFX *tft = new Arduino_ST7789(bus, TFT_RST, 1, true, SCREEN_WIDTH, SCREEN_HEIGHT);

#define MOTOR_PIN1 0
#define MOTOR_PIN2 1
#define MOTOR_PIN3 2
#define MOTOR_PIN4 3
#define HOME_SWITCH 18

// Motor parameters
#define STEPS_PER_ROTATION 4096
#define STEPPER_MAX_SPEED 400.0
#define STEPPER_HOMING_SPEED 100.0
#define STEPPER_ACC 100.0
#define STEPPER_HOMING_ACC 50.0

AccelStepper stepper(AccelStepper::HALF4WIRE, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4, true);

// Calibration variables
long initHoming = -1;
int stepperCalibrationStep = 0;
bool homeSwitchInverse = true;
bool isCalibrated = false;

// Wind data structure
struct WindInfo {
    float speed;
    float direction;
    float temperature;
} windInfo;

// Debug flag
#define DEBUG true
#define DEBUG_SERIAL if (DEBUG) Serial

// Calibration flag - set to false to skip motor calibration
#define ENABLE_CALIBRATION false

// Include FreeFonts
#include "Fonts/FreeSansBold24pt7b.h"
#include "Fonts/FreeSansBold18pt7b.h"

// Function to calculate shortest distance for stepper rotation
static long shortest_distance(long origin, long target) {
    auto signedDiff = 0l;
    auto raw_diff = origin > target ? origin - target : target - origin;
    auto mod_diff = ((raw_diff % STEPS_PER_ROTATION) + STEPS_PER_ROTATION) % STEPS_PER_ROTATION;

    if (mod_diff > (STEPS_PER_ROTATION / 2)) {
        signedDiff = STEPS_PER_ROTATION - mod_diff;
        if (target > origin)
            signedDiff = -signedDiff;
    } else {
        signedDiff = mod_diff;
        if (origin > target)
            signedDiff = -signedDiff;
    }
    DEBUG_SERIAL.printf("[STEPPER] Shortest distance from %ld to %ld = %ld steps\n", origin, target, signedDiff);
    return signedDiff;
}

// Function to run stepper motor
void runStepper(long steps) {
    if (steps != 0) {
        DEBUG_SERIAL.printf("[STEPPER] Moving %ld steps\n", steps);
        stepper.enableOutputs();
        stepper.move(steps);
        stepper.runToPosition();
        delay(2);
        stepper.disableOutputs();
        DEBUG_SERIAL.println("[STEPPER] Movement complete");
    } else {
        DEBUG_SERIAL.println("[STEPPER] No movement needed (0 steps)");
    }
}

// Stepper calibration/homing routine (non-blocking)
bool stepperPosCalibrate() {
    pinMode(HOME_SWITCH, INPUT);

    switch (stepperCalibrationStep) {
        case 0:
            DEBUG_SERIAL.println("[CALIBRATION] Step 0: Starting homing sequence");
            stepper.stop();
            stepper.setMaxSpeed(STEPPER_MAX_SPEED);
            stepper.setAcceleration(STEPPER_ACC);
            stepper.enableOutputs();
            DEBUG_SERIAL.println("[CALIBRATION] Stepper is Homing...");
            stepperCalibrationStep = 10;
            break;
            
        case 10:
            stepper.move(initHoming);
            initHoming--;
            stepper.run();
            delay(5);
            if ((digitalRead(HOME_SWITCH) == LOW && !homeSwitchInverse) || 
                (digitalRead(HOME_SWITCH) == HIGH && homeSwitchInverse)) {
                DEBUG_SERIAL.println("[CALIBRATION] Step 10: Home switch triggered");
                stepperCalibrationStep = 20;
            }
            break;
            
        case 20:
            DEBUG_SERIAL.println("[CALIBRATION] Step 20: Setting zero position");
            stepper.setCurrentPosition(0);
            stepper.setMaxSpeed(STEPPER_HOMING_SPEED);
            stepper.setAcceleration(STEPPER_HOMING_ACC);
            initHoming = 1;
            stepperCalibrationStep = 30;
            break;
            
        case 30:
            stepper.move(initHoming);
            stepper.run();
            initHoming++;
            delay(5);
            if ((digitalRead(HOME_SWITCH) == HIGH && !homeSwitchInverse) || 
                (digitalRead(HOME_SWITCH) == LOW && homeSwitchInverse)) {
                DEBUG_SERIAL.println("[CALIBRATION] Step 30: Home switch released");
                stepperCalibrationStep = 40;
            }
            break;
            
        case 40:
            stepper.setCurrentPosition(0);
            DEBUG_SERIAL.println("[CALIBRATION] Step 40: Homing Completed Successfully");
            stepper.setMaxSpeed(STEPPER_MAX_SPEED);
            stepper.setAcceleration(STEPPER_ACC);
            stepper.disableOutputs();
            initHoming = -1;
            stepperCalibrationStep = 0;
            return true;
            
        default:
            break;
    }
    return false;
}

// Move stepper to wind direction (0-360 degrees)
void moveStepperToDirection(float targetDirection) {
    DEBUG_SERIAL.printf("[STEPPER] Moving to wind direction: %.1f degrees\n", targetDirection);
    auto targetPosition = long(targetDirection * double(STEPS_PER_ROTATION) / 360.0);
    auto currentPosition = stepper.currentPosition();
    DEBUG_SERIAL.printf("[STEPPER] Current position: %ld, Target position: %ld\n", currentPosition, targetPosition);
    runStepper(shortest_distance(currentPosition, targetPosition));
}

// Function to fetch data from the API
String fetchData(const char* apiUrl) {
    DEBUG_SERIAL.println("[HTTP] Starting API request...");
    DEBUG_SERIAL.printf("[HTTP] URL: %s\n", apiUrl);
    
    HTTPClient http;
    http.begin(apiUrl);
    http.setTimeout(10000); // 10 second timeout
    
    int httpCode = http.GET();
    DEBUG_SERIAL.printf("[HTTP] Response code: %d\n", httpCode);
    
    if (httpCode > 0) {
        String payload = http.getString();
        DEBUG_SERIAL.printf("[HTTP] Response length: %d bytes\n", payload.length());
        DEBUG_SERIAL.println("[HTTP] Response data:");
        DEBUG_SERIAL.println(payload);
        http.end();
        return payload;
    } else {
        DEBUG_SERIAL.printf("[HTTP] Error: %s\n", http.errorToString(httpCode).c_str());
        http.end();
        return "";
    }
}

// Function to display centered wind speed with FreeSansBold
void displayCenteredWindSpeed(float speed, uint16_t textColor, uint16_t bgColor) {
    DEBUG_SERIAL.printf("[DISPLAY] Updating display - Speed: %.1f m/s\n", speed);
    tft->fillScreen(bgColor);
    
    // Convert speed to string with 1 decimal place
    char speedStr[8];
    dtostrf(speed, 0, 1, speedStr);
    char label[] = " m/s";
    
    // Set font for the speed number and get dimensions
    tft->setFont(&FreeSansBold18pt7b);
    tft->setTextColor(textColor);
    
    int16_t x1, y1;
    uint16_t w_speed, h_speed;
    tft->getTextBounds(speedStr, 0, 0, &x1, &y1, &w_speed, &h_speed);
    
    // Get dimensions for the smaller "m/s" label
    tft->setFont(&FreeSansBold18pt7b);
    uint16_t w_label, h_label;
    tft->getTextBounds(label, 0, 0, &x1, &y1, &w_label, &h_label);
    
    // Calculate total width and centered starting position
    int totalWidth = w_speed + w_label;
    int startX = (SCREEN_WIDTH - totalWidth) / 2;
    int yPos = (SCREEN_HEIGHT / 2);
    
    // Draw the wind speed number
    tft->setFont(&FreeSansBold24pt7b);
    tft->setCursor(startX, yPos);
    tft->print(speedStr);
    
    // Draw "m/s" label on the same line with smaller font
   // tft->setFont(&FreeSansBold18pt7b);
  //  tft->setCursor(startX + w_speed, yPos);
  //  tft->print(label);
    
    DEBUG_SERIAL.println("[DISPLAY] Display update complete");
}

// Function to process JSON and control LED, Display, and Stepper
void processJSON(String jsonResponse) {
    DEBUG_SERIAL.println("[JSON] Processing JSON response...");
    
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonResponse);
    
    if (error) {
        DEBUG_SERIAL.print("[JSON] Deserialization failed: ");
        DEBUG_SERIAL.println(error.c_str());
        return;
    }

    DEBUG_SERIAL.println("[JSON] JSON parsed successfully");

    // Extract wind data
    windInfo.speed = doc["wind"]["speed"];
    windInfo.direction = doc["wind"]["direction"];
    windInfo.temperature = doc["temperature"];

    DEBUG_SERIAL.println("[DATA] Wind Information:");
    DEBUG_SERIAL.printf("  Speed: %.1f m/s\n", windInfo.speed);
    DEBUG_SERIAL.printf("  Direction: %.1f degrees\n", windInfo.direction);
    DEBUG_SERIAL.printf("  Temperature: %.1f°C\n", windInfo.temperature);

    // Move stepper motor to wind direction (only if calibrated)
    if (isCalibrated) {
        DEBUG_SERIAL.println("[CONTROL] Motor is calibrated, moving to direction");
        moveStepperToDirection(windInfo.direction);
    } else {
        DEBUG_SERIAL.println("[CONTROL] WARNING: Motor not calibrated, skipping movement");
    }

    // Determine color based on conditions
    uint16_t backgroundColor;
    uint16_t textColor = WHITE;
    
    if (windInfo.speed > 10.0) {
        DEBUG_SERIAL.println("[CONTROL] Condition: HIGH WIND (>10 m/s) - Setting BLUE");
        leds[0] = CRGB::Blue;
        backgroundColor = BLUE;
    } else if (windInfo.speed >= 4.0 && windInfo.speed <= 8.0 && 
               windInfo.direction >= 160.0 && windInfo.direction <= 260.0) {
        DEBUG_SERIAL.println("[CONTROL] Condition: GOOD (4-8 m/s, 160-260°) - Setting GREEN");
        leds[0] = CRGB::Green;
        backgroundColor = GREEN;
    } else {
        DEBUG_SERIAL.println("[CONTROL] Condition: POOR - Setting RED");
        leds[0] = CRGB::Red;
        
        backgroundColor = RED;
    }
    FastLED.show();
    DEBUG_SERIAL.println("[LED] LED color updated");

    // Display centered wind speed
    displayCenteredWindSpeed(windInfo.speed, textColor, backgroundColor);
}

void setup() {
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize
    
    DEBUG_SERIAL.println("\n\n=================================");
    DEBUG_SERIAL.println("ESP32 Wind Monitor Starting...");
    DEBUG_SERIAL.println("=================================");
    
    // Initialize LED
    DEBUG_SERIAL.println("[INIT] Initializing LED...");
    FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);

    leds[0] = CRGB::Black;
    FastLED.show();
    DEBUG_SERIAL.println("[INIT] LED initialized");
    
    // Initialize Display
    DEBUG_SERIAL.println("[INIT] Initializing display...");
    
    // Turn on backlight first
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    DEBUG_SERIAL.println("[INIT] Backlight turned ON");
    
    tft->begin();
    DEBUG_SERIAL.println("[INIT] Display begin() called");
    
    delay(100); // Give display time to initialize
    
    tft->fillScreen(RED);
    DEBUG_SERIAL.println("[INIT] Filled screen RED - can you see this?");
    delay(1000);
    
    tft->fillScreen(GREEN);
    DEBUG_SERIAL.println("[INIT] Filled screen GREEN - can you see this?");
    delay(1000);
    
    tft->fillScreen(BLUE);
    DEBUG_SERIAL.println("[INIT] Filled screen BLUE - can you see this?");
    delay(1000);
    
    tft->fillScreen(BLACK);
    tft->setTextColor(WHITE);
    tft->setTextSize(2);
    tft->setCursor(20, 60);
    tft->println("Connecting WiFi...");
    DEBUG_SERIAL.println("[INIT] Display initialized and tested");
    
    // WiFiManager
    DEBUG_SERIAL.println("[WIFI] Starting WiFi connection...");
    DEBUG_SERIAL.println("[WIFI] AP Name: esp32-wind");
    WiFiManager wifiManager;
    wifiManager.autoConnect("esp32-wind");
    
    DEBUG_SERIAL.println("[WIFI] WiFi connected successfully!");
    DEBUG_SERIAL.print("[WIFI] IP Address: ");
    DEBUG_SERIAL.println(WiFi.localIP());
    DEBUG_SERIAL.print("[WIFI] SSID: ");
    DEBUG_SERIAL.println(WiFi.SSID());
    DEBUG_SERIAL.print("[WIFI] Signal Strength: ");
    DEBUG_SERIAL.print(WiFi.RSSI());
    DEBUG_SERIAL.println(" dBm");
    
    tft->fillScreen(BLACK);
    tft->setCursor(20, 60);
    tft->println("WiFi Connected!");
    delay(1000);
    
    // Initialize Stepper Motor
    #if ENABLE_CALIBRATION
        DEBUG_SERIAL.println("[INIT] Starting motor calibration...");
        tft->fillScreen(BLACK);
        tft->setCursor(20, 60);
        tft->println("Calibrating motor...");
        
        // Perform homing/calibration
        int calibrationAttempts = 0;
        while (!isCalibrated) {
            isCalibrated = stepperPosCalibrate();
            calibrationAttempts++;
            if (calibrationAttempts % 100 == 0) {
                DEBUG_SERIAL.printf("[CALIBRATION] Attempt %d, Step: %d\n", calibrationAttempts, stepperCalibrationStep);
            }
        }
        
        DEBUG_SERIAL.println("[INIT] Motor calibration complete!");
    #else
        DEBUG_SERIAL.println("[INIT] Motor calibration DISABLED (ENABLE_CALIBRATION = false)");
        DEBUG_SERIAL.println("[INIT] Motor will NOT move until calibration is enabled");
        isCalibrated = false; // Explicitly set to false
    #endif
    
    tft->fillScreen(BLACK);
    tft->setCursor(20, 60);
    tft->println("Ready!");
    delay(1000);
    
    DEBUG_SERIAL.println("=================================");
    DEBUG_SERIAL.println("Setup Complete - Entering Loop");
    DEBUG_SERIAL.println("=================================\n");
}

void loop() {
    static const char* apiUrl = "https://api.holfuy.com/live/?s=214&pw=correcthorsebatterystaple&m=JSON&tu=C&su=m/s";
    static unsigned long loopCount = 0;
    
    loopCount++;
    DEBUG_SERIAL.println("\n---------------------------------");
    DEBUG_SERIAL.printf("Loop iteration: %lu\n", loopCount);
    DEBUG_SERIAL.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    DEBUG_SERIAL.println("---------------------------------");
    
    if (WiFi.status() == WL_CONNECTED) {
        DEBUG_SERIAL.println("[WIFI] WiFi connected, fetching data...");
        String jsonResponse = fetchData(apiUrl);
        if (jsonResponse.length() > 0) {
            processJSON(jsonResponse);
        } else {
            DEBUG_SERIAL.println("[ERROR] Empty response from API");
        }
    } else {
        DEBUG_SERIAL.println("[ERROR] WiFi not connected. Reconnecting...");
        DEBUG_SERIAL.print("[WIFI] Status code: ");
        DEBUG_SERIAL.println(WiFi.status());
        
        tft->fillScreen(BLACK);
        tft->setCursor(20, 80);
        tft->setTextSize(2);
        tft->println("WiFi Error!");
        
        WiFi.reconnect();
    }
    
    DEBUG_SERIAL.println("[LOOP] Waiting 60 seconds before next update...\n");
    delay(60000); // Wait for 60 seconds before the next API call
}
