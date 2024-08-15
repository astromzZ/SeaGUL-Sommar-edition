#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include "page.h"  // Include the header file with HTML content

// Network credentials
const char* ssid = "SEAGUL";
const char* password = "0123456789";

// Custom IP configuration
IPAddress local_IP(192, 168, 4, 1);  // Change this to your desired IP
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Create a WebServer object on port 80
WebServer server(80);

// Global variable to be controlled by the buttons
String currentState = "Idle";

// Handle root path
void handleRoot() {
  String pageWithSignalStrength = String(page_html);
  server.send(200, "text/html", pageWithSignalStrength);
}

// Handle update
void handleUpdate() {
  if (server.hasArg("state")) {
    currentState = server.arg("state");
    Serial.print("Current state updated to: ");
    Serial.println(currentState);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// Function to generate random number
float randomFloat(float minValue, float maxValue) {
  return minValue + (float)rand() / ((float)RAND_MAX / (maxValue - minValue));
}

// Handle signal strength
String getConnectedDevices() {
  String deviceInfo = "[";
  wifi_sta_list_t wifi_sta_list;
  esp_wifi_ap_get_sta_list(&wifi_sta_list);

  for (int i = 0; i < wifi_sta_list.num; i++) {
    if (i > 0) deviceInfo += ",";
    wifi_sta_info_t station = wifi_sta_list.sta[i];
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             station.mac[0], station.mac[1], station.mac[2],
             station.mac[3], station.mac[4], station.mac[5]);
    deviceInfo += "{";
    deviceInfo += "\"mac\":\"" + String(macStr) + "\",";
    deviceInfo += "\"rssi\":" + String(station.rssi);
    deviceInfo += "}";
  }

  deviceInfo += "]";
  return deviceInfo;
}

// Handle data
void handleData() {
  String json = "{";
  json += "\"currentState\":\"" + currentState + "\",";
  json += "\"devices\":" + getConnectedDevices() + ",";
  json += "\"waterTemperature\":" + String(randomFloat(15.0, 25.0)) + ",";
  json += "\"pressure\":" + String(randomFloat(1.0, 5.0)) + ",";
  json += "\"salinity\":" + String(randomFloat(30.0, 50.0)) + ",";
  json += "\"pitch\":" + String(randomFloat(-360.0, 360.0)) + ",";
  json += "\"roll\":" + String(randomFloat(-360.0, 360.0)) + ",";
  json += "\"yaw\":" + String(randomFloat(-360.0, 360.0)) + ",";
  json += "\"batteryVoltage\":" + String(randomFloat(22.0, 38.5)) + ",";
  json += "\"compassCourse\":" + String(randomFloat(0.0, 360.0)) + ",";
  json += "\"gnssCoordinates\":\"" + String(randomFloat(-90.0, 90.0), 6) + ", " + String(randomFloat(-180.0, 180.0), 6) + "\",";
  json += "\"internalTemperature\":" + String(randomFloat(15.0, 25.0)) + ",";
  json += "\"internalPressure\":" + String(randomFloat(0.0, 1.0)) + ",";
  json += "\"internalHumidity\":" + String(randomFloat(10.0, 30.0));
  json += "}";
  server.send(200, "application/json", json);
}

// Setup function
void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/data", handleData);

  server.begin();
  Serial.println("HTTP server started");
}

// Loop function
void loop() {
  server.handleClient();
}
