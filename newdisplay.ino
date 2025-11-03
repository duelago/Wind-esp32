#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// LED Configuration
#define LED_PIN 48
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Display Configuration (adjust pins according to your wiring)
#define TFT_CS     10
#define TFT_RST    9
#define TFT_DC     8

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Function to fetch data from the API
String fetchData(const char* apiUrl) {
    HTTPClient http;
    http.begin(apiUrl);
    int httpCode = http.GET();

    if (httpCode > 0) {
        return http.getString();
    } else {
        Serial.println("Error on HTTP request");
        return "";
    }
    http.end();
}

// Function to process JSON and control LED + Display
void processJSON(String jsonResponse) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonResponse);

    if (error) {
        Serial.print("JSON Deserialization failed: ");
        Serial.println(error.c_str());
        return;
    }

    float windSpeed = doc["wind"]["speed"];
    float windDirection = doc["wind"]["direction"];

    Serial.printf("Wind Speed: %.1f, Wind Direction: %.1f\n", windSpeed, windDirection);

    // Determine color based on conditions
    uint16_t backgroundColor;
    if (windSpeed > 10.0) {
        leds[0] = CRGB::Blue;
        backgroundColor = ST77XX_BLUE;
    } else if (windSpeed >= 4.0 && windSpeed <= 8.0 && windDirection >= 160.0 && windDirection <= 260.0) {
        leds[0] = CRGB::Green;
        backgroundColor = ST77XX_GREEN;
    } else {
        leds[0] = CRGB::Red;
        backgroundColor = ST77XX_RED;
    }
    FastLED.show();

    // Update Display with matching background color
    tft.fillScreen(backgroundColor);
    tft.setCursor(10, 40);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(3);
    tft.println("Wind Speed");
    
    tft.setCursor(10, 90);
    tft.setTextSize(5);
    tft.print(windSpeed, 1);
    tft.setTextSize(3);
    tft.println(" m/s");
}

void setup() {
    Serial.begin(115200);
    
    // Initialize LED
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB::Black;
    FastLED.show();

    // Initialize Display
    tft.init(172, 320);  // Initialize for 320x172 display
    tft.setRotation(1);  // Landscape mode
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 80);
    tft.println("Connecting WiFi...");

    // WiFiManager
    WiFiManager wifiManager;
    wifiManager.autoConnect("esp32-wind");

    Serial.println("WiFi connected");
    
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, 80);
    tft.println("WiFi Connected!");
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
        Serial.println("WiFi not connected. Reconnecting...");
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(10, 80);
        tft.println("WiFi Error!");
    }

    delay(60000); // Wait for 60 seconds before the next API call
}
