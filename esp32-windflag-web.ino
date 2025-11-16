#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <Arduino_GFX_Library.h>
#include <AccelStepper.h>
#include <SPI.h>
#include <EEPROM.h>
#include <WebServer.h>

#define LED_PIN 8
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// ---------------- Display Configuration ----------------
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

// EEPROM addresses
#define HOLFUY_STATION_EEPROM_ADDRESS 0
#define HOLFUY_PASSWORD_EEPROM_ADDRESS 16
#define EEPROM_SIZE 512

// Holfuy configuration
String holfuy_station_id = "214";  // Default station
String holfuy_password = "correcthorsebatterystaple";  // Default password

// Web Server
WebServer server(80);

// Include FreeFonts
#include "Fonts/FreeSansBold24pt7b.h"
#include "Fonts/FreeSansBold18pt7b.h"

// EEPROM Functions
void writeStringToEEPROM(int addrOffset, const String &strToWrite) {
    byte len = strToWrite.length();
    EEPROM.write(addrOffset, len);
    for (int i = 0; i < len; i++) {
        EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
    }
    EEPROM.commit();
    DEBUG_SERIAL.printf("[EEPROM] Wrote string to address %d: %s\n", addrOffset, strToWrite.c_str());
}

String readStringFromEEPROM(int addrOffset) {
    byte len = EEPROM.read(addrOffset);
    if (len == 0 || len == 255) {  // Empty or uninitialized
        return "";
    }
    char data[len + 1];
    for (int i = 0; i < len; i++) {
        data[i] = (char)EEPROM.read(addrOffset + 1 + i);
    }
    data[len] = '\0';
    String result = String(data).substring(0, len);
    DEBUG_SERIAL.printf("[EEPROM] Read string from address %d: %s\n", addrOffset, result.c_str());
    return result;
}

// Web Interface HTML
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WindFlag - Configuration</title>
<style>
body {
    font-family: Arial, sans-serif;
    max-width: 600px;
    margin: 50px auto;
    padding: 20px;
    background-color: #f0f0f0;
}
.container {
    background-color: white;
    padding: 30px;
    border-radius: 10px;
    box-shadow: 0 2px 10px rgba(0,0,0,0.1);
}
h1 {
    color: #333;
    text-align: center;
}
h2 {
    color: #666;
    border-bottom: 2px solid #4CAF50;
    padding-bottom: 10px;
}
.form-group {
    margin-bottom: 20px;
}
label {
    display: block;
    margin-bottom: 5px;
    font-weight: bold;
    color: #555;
}
input[type="text"] {
    width: 100%;
    padding: 10px;
    border: 1px solid #ddd;
    border-radius: 5px;
    box-sizing: border-box;
    font-size: 16px;
}
input[type="submit"] {
    background-color: #4CAF50;
    color: white;
    padding: 12px 30px;
    border: none;
    border-radius: 5px;
    cursor: pointer;
    font-size: 16px;
    width: 100%;
    margin-top: 10px;
}
input[type="submit"]:hover {
    background-color: #45a049;
}
.current-config {
    background-color: #f9f9f9;
    padding: 15px;
    border-radius: 5px;
    margin-bottom: 20px;
    border-left: 4px solid #4CAF50;
}
.current-config p {
    margin: 5px 0;
    color: #666;
}
.success-message {
    background-color: #d4edda;
    color: #155724;
    padding: 15px;
    border-radius: 5px;
    margin-bottom: 20px;
    border: 1px solid #c3e6cb;
}
</style>
</head>
<body>
<div class="container">
<h1>🌪️ WindFlag Configuration</h1>

<div class="current-config">
<h3>Current Configuration</h3>
<p><strong>Station ID:</strong> %STATION_ID%</p>
<p><strong>Password:</strong> %PASSWORD_DISPLAY%</p>
<p><strong>IP Address:</strong> %IP_ADDRESS%</p>
</div>

<h2>Holfuy Station Settings</h2>
<form action="/saveholfuy" method="GET">
<div class="form-group">
<label for="station_id">Station ID:</label>
<input type="text" id="station_id" name="station_id" value="%STATION_ID%" required>
</div>
<div class="form-group">
<label for="password">Password:</label>
<input type="text" id="password" name="password" value="%PASSWORD%" required>
</div>
<input type="submit" value="Save Configuration">
</form>

<div style="margin-top: 30px; text-align: center; color: #999; font-size: 12px;">
<p>WindFlag v2.0 - Station configuration will be saved to EEPROM</p>
<p>Contact info@holfuy.hu for station API credentials</p>
</div>
</div>
</body>
</html>
)rawliteral";

// Success page HTML
const char success_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="refresh" content="3;url=/">
<title>Configuration Saved</title>
<style>
body {
    font-family: Arial, sans-serif;
    max-width: 600px;
    margin: 50px auto;
    padding: 20px;
    background-color: #f0f0f0;
}
.container {
    background-color: white;
    padding: 30px;
    border-radius: 10px;
    box-shadow: 0 2px 10px rgba(0,0,0,0.1);
    text-align: center;
}
.success {
    color: #155724;
    font-size: 24px;
    margin-bottom: 20px;
}
</style>
</head>
<body>
<div class="container">
<div class="success">✅ Configuration Saved Successfully!</div>
<p>Station ID: <strong>%STATION_ID%</strong></p>
<p>Password: <strong>%PASSWORD_DISPLAY%</strong></p>
<p>Redirecting to main page in 3 seconds...</p>
<p><a href="/">Click here if not redirected</a></p>
</div>
</body>
</html>
)rawliteral";

// Handle root page
void handleRoot() {
    DEBUG_SERIAL.println("[WEB] Serving root page");
    String html = String(index_html);
    
    // Replace placeholders
    html.replace("%STATION_ID%", holfuy_station_id);
    html.replace("%PASSWORD%", holfuy_password);
    html.replace("%PASSWORD_DISPLAY%", "********");  // Mask password display
    html.replace("%IP_ADDRESS%", WiFi.localIP().toString());
    
    server.send(200, "text/html", html);
}

// Handle save configuration
void handleSaveHolfuy() {
    DEBUG_SERIAL.println("[WEB] Saving Holfuy configuration");
    
    if (server.hasArg("station_id") && server.hasArg("password")) {
        holfuy_station_id = server.arg("station_id");
        holfuy_password = server.arg("password");
        
        DEBUG_SERIAL.printf("[WEB] New Station ID: %s\n", holfuy_station_id.c_str());
        DEBUG_SERIAL.printf("[WEB] New Password: %s\n", holfuy_password.c_str());
        
        // Save to EEPROM
        writeStringToEEPROM(HOLFUY_STATION_EEPROM_ADDRESS, holfuy_station_id);
        writeStringToEEPROM(HOLFUY_PASSWORD_EEPROM_ADDRESS, holfuy_password);
        
        // Send success page
        String html = String(success_html);
        html.replace("%STATION_ID%", holfuy_station_id);
        html.replace("%PASSWORD_DISPLAY%", "********");
        
        server.send(200, "text/html", html);
        
        DEBUG_SERIAL.println("[WEB] Configuration saved to EEPROM");
    } else {
        server.send(400, "text/plain", "Missing parameters");
        DEBUG_SERIAL.println("[WEB] Error: Missing parameters");
    }
}

// Build API URL with current configuration
String getApiUrl() {
    String url = "https://api.holfuy.com/live/?s=" + holfuy_station_id + 
                 "&pw=" + holfuy_password + 
                 "&m=JSON&tu=C&su=m/s";
    return url;
}

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
    http.setTimeout(10000);
    
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

// Function to display centered wind speed
void displayCenteredWindSpeed(float speed, uint16_t textColor, uint16_t bgColor) {
    DEBUG_SERIAL.printf("[DISPLAY] Updating display - Speed: %.1f m/s\n", speed);
    tft->fillScreen(bgColor);
    
    char speedStr[8];
    dtostrf(speed, 0, 1, speedStr);
    
    tft->setFont(&FreeSansBold24pt7b);
    tft->setTextColor(textColor);
    
    int16_t x1, y1;
    uint16_t w_speed, h_speed;
    tft->getTextBounds(speedStr, 0, 0, &x1, &y1, &w_speed, &h_speed);
    
    int startX = (SCREEN_WIDTH - w_speed) / 2;
    int yPos = (SCREEN_HEIGHT / 2);
    
    tft->setCursor(startX, yPos);
    tft->print(speedStr);
    
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

    windInfo.speed = doc["wind"]["speed"];
    windInfo.direction = doc["wind"]["direction"];
    windInfo.temperature = doc["temperature"];

    DEBUG_SERIAL.println("[DATA] Wind Information:");
    DEBUG_SERIAL.printf("  Speed: %.1f m/s\n", windInfo.speed);
    DEBUG_SERIAL.printf("  Direction: %.1f degrees\n", windInfo.direction);
    DEBUG_SERIAL.printf("  Temperature: %.1f°C\n", windInfo.temperature);

    if (isCalibrated) {
        DEBUG_SERIAL.println("[CONTROL] Motor is calibrated, moving to direction");
        moveStepperToDirection(windInfo.direction);
    } else {
        DEBUG_SERIAL.println("[CONTROL] WARNING: Motor not calibrated, skipping movement");
    }

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

    displayCenteredWindSpeed(windInfo.speed, textColor, backgroundColor);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    DEBUG_SERIAL.println("\n\n=================================");
    DEBUG_SERIAL.println("ESP32 Wind Monitor Starting...");
    DEBUG_SERIAL.println("=================================");
    
    // Initialize EEPROM
    DEBUG_SERIAL.println("[INIT] Initializing EEPROM...");
    EEPROM.begin(EEPROM_SIZE);
    
    // Load saved configuration from EEPROM
    String saved_station = readStringFromEEPROM(HOLFUY_STATION_EEPROM_ADDRESS);
    String saved_password = readStringFromEEPROM(HOLFUY_PASSWORD_EEPROM_ADDRESS);
    
    if (saved_station.length() > 0) {
        holfuy_station_id = saved_station;
        DEBUG_SERIAL.printf("[INIT] Loaded station ID from EEPROM: %s\n", holfuy_station_id.c_str());
    }
    
    if (saved_password.length() > 0) {
        holfuy_password = saved_password;
        DEBUG_SERIAL.printf("[INIT] Loaded password from EEPROM: %s\n", holfuy_password.c_str());
    }
    
    // Initialize LED
    DEBUG_SERIAL.println("[INIT] Initializing LED...");
    FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);
    leds[0] = CRGB::Black;
    FastLED.show();
    DEBUG_SERIAL.println("[INIT] LED initialized");
    
    // Initialize Display
    DEBUG_SERIAL.println("[INIT] Initializing display...");
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    
    tft->begin();
    delay(100);
    
    tft->fillScreen(RED);
    delay(1000);
    tft->fillScreen(GREEN);
    delay(1000);
    tft->fillScreen(BLUE);
    delay(1000);
    
    tft->fillScreen(BLACK);
    tft->setTextColor(WHITE);
    tft->setTextSize(2);
    tft->setCursor(20, 60);
    tft->println("Connecting WiFi...");
    DEBUG_SERIAL.println("[INIT] Display initialized");
    
    // WiFiManager
    DEBUG_SERIAL.println("[WIFI] Starting WiFi connection...");
    WiFiManager wifiManager;
    wifiManager.autoConnect("esp32-wind");
    
    DEBUG_SERIAL.println("[WIFI] WiFi connected successfully!");
    DEBUG_SERIAL.print("[WIFI] IP Address: ");
    DEBUG_SERIAL.println(WiFi.localIP());
    
    tft->fillScreen(BLACK);
    tft->setCursor(20, 60);
    tft->println("WiFi Connected!");
    tft->setCursor(20, 100);
    tft->print("IP: ");
    tft->println(WiFi.localIP());
    delay(2000);
    
    // Initialize Web Server
    DEBUG_SERIAL.println("[WEB] Starting web server...");
    server.on("/", handleRoot);
    server.on("/saveholfuy", handleSaveHolfuy);
    server.begin();
    DEBUG_SERIAL.println("[WEB] Web server started");
    DEBUG_SERIAL.printf("[WEB] Configuration page: http://%s/\n", WiFi.localIP().toString().c_str());
    
    // Initialize Stepper Motor
    #if ENABLE_CALIBRATION
        DEBUG_SERIAL.println("[INIT] Starting motor calibration...");
        tft->fillScreen(BLACK);
        tft->setCursor(20, 60);
        tft->println("Calibrating motor...");
        
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
        DEBUG_SERIAL.println("[INIT] Motor calibration DISABLED");
        isCalibrated = false;
    #endif
    
    tft->fillScreen(BLACK);
    tft->setCursor(20, 60);
    tft->println("Ready!");
    tft->setCursor(20, 100);
    tft->print("Config: ");
    tft->println(WiFi.localIP());
    delay(2000);
    
    DEBUG_SERIAL.println("=================================");
    DEBUG_SERIAL.println("Setup Complete - Entering Loop");
    DEBUG_SERIAL.println("=================================\n");
}

void loop() {
    static unsigned long loopCount = 0;
    static unsigned long lastUpdate = 0;
    unsigned long currentMillis = millis();
    
    // Handle web server requests
    server.handleClient();
    
    // Update weather data every 60 seconds
    if (currentMillis - lastUpdate >= 60000 || lastUpdate == 0) {
        lastUpdate = currentMillis;
        loopCount++;
        
        DEBUG_SERIAL.println("\n---------------------------------");
        DEBUG_SERIAL.printf("Loop iteration: %lu\n", loopCount);
        DEBUG_SERIAL.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
        DEBUG_SERIAL.println("---------------------------------");
        
        if (WiFi.status() == WL_CONNECTED) {
            DEBUG_SERIAL.println("[WIFI] WiFi connected, fetching data...");
            String apiUrl = getApiUrl();
            String jsonResponse = fetchData(apiUrl.c_str());
            if (jsonResponse.length() > 0) {
                processJSON(jsonResponse);
            } else {
                DEBUG_SERIAL.println("[ERROR] Empty response from API");
            }
        } else {
            DEBUG_SERIAL.println("[ERROR] WiFi not connected. Reconnecting...");
            tft->fillScreen(BLACK);
            tft->setCursor(20, 80);
            tft->setTextSize(2);
            tft->println("WiFi Error!");
            WiFi.reconnect();
        }
    }
    
    delay(10); // Small delay to prevent watchdog issues
}
