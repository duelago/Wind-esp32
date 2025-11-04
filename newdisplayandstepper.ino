#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <Arduino_GFX_Library.h>
#include <AccelStepper.h>
#include <SPI.h>

// LED Configuration
#define LED_PIN 8
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Display Configuration
#define TFT_CS     14
#define TFT_RST    21
#define TFT_DC     15
#define TFT_SCK    7
#define TFT_MOSI   6
#define TFT_BL 22

// Display dimensions
#define SCREEN_WIDTH 172
#define SCREEN_HEIGHT 320

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, -1);
Arduino_GFX *tft = new Arduino_ST7789(bus, TFT_RST, 1, true, SCREEN_WIDTH, SCREEN_HEIGHT);

// Stepper Motor Configuration
#define MOTOR_PIN1 14
#define MOTOR_PIN2 12
#define MOTOR_PIN3 13
#define MOTOR_PIN4 15
#define HOME_SWITCH 16

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
    return signedDiff;
}

// Function to run stepper motor
void runStepper(long steps) {
    if (steps != 0) {
        stepper.enableOutputs();
        stepper.move(steps);
        stepper.runToPosition();
        delay(2);
        stepper.disableOutputs();
    }
}

// Stepper calibration/homing routine (non-blocking)
bool stepperPosCalibrate() {
    pinMode(HOME_SWITCH, INPUT);

    switch (stepperCalibrationStep) {
        case 0:
            stepper.stop();
            stepper.setMaxSpeed(STEPPER_MAX_SPEED);
            stepper.setAcceleration(STEPPER_ACC);
            stepper.enableOutputs();
            DEBUG_SERIAL.println("Stepper is Homing...");
            stepperCalibrationStep = 10;
            break;
            
        case 10:
            stepper.move(initHoming);
            initHoming--;
            stepper.run();
            delay(5);
            if ((digitalRead(HOME_SWITCH) == LOW && !homeSwitchInverse) || 
                (digitalRead(HOME_SWITCH) == HIGH && homeSwitchInverse)) {
                stepperCalibrationStep = 20;
            }
            break;
            
        case 20:
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
                stepperCalibrationStep = 40;
            }
            break;
            
        case 40:
            stepper.setCurrentPosition(0);
            DEBUG_SERIAL.println("Homing Completed");
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
    auto targetPosition = long(targetDirection * double(STEPS_PER_ROTATION) / 360.0);
    auto currentPosition = stepper.currentPosition();
    runStepper(shortest_distance(currentPosition, targetPosition));
}

// Function to fetch data from the API
String fetchData(const char* apiUrl) {
    HTTPClient http;
    http.begin(apiUrl);
    int httpCode = http.GET();
    
    if (httpCode > 0) {
        return http.getString();
    } else {
        DEBUG_SERIAL.println("Error on HTTP request");
        return "";
    }
    http.end();
}

// Function to display centered wind speed with FreeSansBold
void displayCenteredWindSpeed(float speed, uint16_t textColor, uint16_t bgColor) {
    tft->fillScreen(bgColor);
    
    // Convert speed to string with 1 decimal place
    char speedStr[8];
    dtostrf(speed, 0, 1, speedStr);
    char label[] = " m/s";
    
    // Set font for the speed number and get dimensions
    tft->setFont(&FreeSansBold24pt7b);
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
    tft->setFont(&FreeSansBold18pt7b);
    tft->setCursor(startX + w_speed, yPos);
    tft->print(label);
}

// Function to process JSON and control LED, Display, and Stepper
void processJSON(String jsonResponse) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonResponse);
    
    if (error) {
        DEBUG_SERIAL.print("JSON Deserialization failed: ");
        DEBUG_SERIAL.println(error.c_str());
        return;
    }

    // Extract wind data
    windInfo.speed = doc["wind"]["speed"];
    windInfo.direction = doc["wind"]["direction"];
    windInfo.temperature = doc["temperature"];

    DEBUG_SERIAL.printf("Wind Speed: %.1f m/s, Direction: %.1f°, Temp: %.1f°C\n", 
                        windInfo.speed, windInfo.direction, windInfo.temperature);

    // Move stepper motor to wind direction (only if calibrated)
    if (isCalibrated) {
        moveStepperToDirection(windInfo.direction);
    }

    // Determine color based on conditions
    uint16_t backgroundColor;
    uint16_t textColor = WHITE;
    
    if (windInfo.speed > 10.0) {
        leds[0] = CRGB::Blue;
        backgroundColor = BLUE;
    } else if (windInfo.speed >= 4.0 && windInfo.speed <= 8.0 && 
               windInfo.direction >= 160.0 && windInfo.direction <= 260.0) {
        leds[0] = CRGB::Green;
        backgroundColor = GREEN;
    } else {
        leds[0] = CRGB::Red;
        backgroundColor = RED;
    }
    FastLED.show();

    // Display centered wind speed
    displayCenteredWindSpeed(windInfo.speed, textColor, backgroundColor);
}

void setup() {
    Serial.begin(115200);
    
    // Initialize LED
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB::Black;
    FastLED.show();
    
    // Initialize Display
    tft->begin();
    tft->fillScreen(BLACK);
    tft->setTextColor(WHITE);
    tft->setTextSize(2);
    tft->setCursor(10, 60);
    tft->println("Connecting WiFi...");
    
    // WiFiManager
    WiFiManager wifiManager;
    wifiManager.autoConnect("esp32-wind");
    DEBUG_SERIAL.println("WiFi connected");
    
    tft->fillScreen(BLACK);
    tft->setCursor(10, 60);
    tft->println("WiFi Connected!");
    delay(1000);
    
    // Initialize Stepper Motor
    tft->fillScreen(BLACK);
    tft->setCursor(10, 60);
    tft->println("Calibrating motor...");
    
    // Perform homing/calibration
    while (!isCalibrated) {
        isCalibrated = stepperPosCalibrate();
    }
    
    tft->fillScreen(BLACK);
    tft->setCursor(10, 60);
    tft->println("Ready!");
    delay(1000);
}

void loop() {
    static const char* apiUrl = "https://api.holfuy.com/live/?s=214&pw=correcthorsebatterystaple&m=JSON&tu=C&su=m/s";
    
    if (WiFi.status() == WL_CONNECTED) {
        String jsonResponse = fetchData(apiUrl);
        if (jsonResponse.length() > 0) {
            processJSON(jsonResponse);
        }
    } else {
        DEBUG_SERIAL.println("WiFi not connected. Reconnecting...");
        tft->fillScreen(BLACK);
        tft->setCursor(10, 80);
        tft->setTextSize(2);
        tft->println("WiFi Error!");
    }
    
    delay(60000); // Wait for 60 seconds before the next API call
}
