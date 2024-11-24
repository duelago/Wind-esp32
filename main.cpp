#include <WiFiManager.h> // WiFi Manager library
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <FastLED.h>

#define LED_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// Function to fetch data from the API
String fetchData(const char* apiUrl) {
    HTTPClient http;
    http.begin(apiUrl); // Specify the API URL
    int httpCode = http.GET(); // Make the HTTP request

    if (httpCode > 0) { // HTTP request was successful
        return http.getString(); // Return response as string
    } else {
        Serial.println("Error on HTTP request");
        return "";
    }
    http.end();
}

// Function to process JSON and control LED
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

    if (windSpeed > 10.0) {
        leds[0] = CRGB::Blue; // Blue for wind speed > 10 m/s
    } else if (windSpeed >= 4.0 && windSpeed <= 8.0 && windDirection >= 160.0 && windDirection <= 260.0) {
        leds[0] = CRGB::Green; // Green for conditions met
    } else {
        leds[0] = CRGB::Red; // Red otherwise
    }

    FastLED.show();
}

void setup() {
    Serial.begin(115200);
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB::Black; // Turn off LED initially
    FastLED.show();

    // WiFiManager
    WiFiManager wifiManager;
    wifiManager.autoConnect("esp32-wind");

    Serial.println("WiFi connected");
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
    }

    delay(60000); // Wait for 10 seconds before the next API call
}
