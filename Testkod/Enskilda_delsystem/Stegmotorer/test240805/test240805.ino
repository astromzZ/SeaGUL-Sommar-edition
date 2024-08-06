

#include <AccelStepper.h>
#include <Arduino.h>
#include <WebServer.h>
#include <pgmspace.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "page.h"               // Include the header file with HTML content


#define TRAN_DIR_PIN            17 //Translation motor direction pin
#define TRAN_STEP_PIN           16 //Translation motor step pin
#define TRAN_SLEEP_PIN          15 //Trasnlation motor sleep pin

AccelStepper translationMotor(AccelStepper::DRIVER, TRAN_STEP_PIN, TRAN_DIR_PIN);

// Enum to keep track of the state of the glider
enum GliderState {
    Idle,
    Diving,
    Calibrating,
    GlidingDown,
    GlidingUp,
    DropWeight,
    Surface
};
GliderState gliderState = Idle; // Initial state of the glider

bool rotatingLeft = false;
bool rotatingRight = false;
bool movingForward = false;
bool movingBackward = false;
bool translationMotorRunning = false;

//NETWORKING

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

// Handle when idle button is pressed
void handleIdle() {
  // isIdle = true;
  // isDiving = false;
  // isCalibrating = false;
  gliderState = Idle;
  Serial.println("Set to Idle");
  server.send(200, "text/plain", "Set to Idle");
}

// Handle when intitiate dive button is pressed
void handleInitiateDive() {
  // isIdle = false;
  // isDiving = true;
  // isCalibrating = false;
  gliderState = Diving;
  Serial.println("Dive Initiated");
  server.send(200, "text/plain", "Dive Initiated");
}

// Handle when calibrate button is pressed
void handleCalibrate() {
  // isIdle = false;
  // isDiving = false;
  // isCalibrating = true;
  gliderState = Calibrating;
  Serial.println("Calibration Started");
  server.send(200, "text/plain", "Calibration Started");
}

// Handle when rotate left button is pressed
void handleRotateLeft() {
  rotatingLeft = true;
//   Serial.println("Rotate Left command received");
//   Serial.print("rotatingLeft: ");
//   Serial.println(rotatingLeft);
  server.send(200, "text/plain", "Rotate Left started");
}

// Handle when rotate right button is pressed
void handleRotateRight() {
  rotatingRight = true;
//   Serial.println("Rotate Right command received");
//   Serial.print("rotatingRight: ");
//   Serial.println(rotatingRight);
  server.send(200, "text/plain", "Rotate Right started");
}

// Handle when move forward button is pressed
void handleMoveForward() {
  movingForward = true;
//   Serial.println("Move Forward command received");
//   Serial.print("movingForward: ");
//   Serial.println(movingForward);
  server.send(200, "text/plain", "Move Forward started");
}

// Handle when move backward button is pressed
void handleMoveBackward() {
  movingBackward = true;
//   Serial.println("Move Backward command received");
//   Serial.print("movingBackward: ");
//   Serial.println(movingBackward);
  server.send(200, "text/plain", "Move Backward started");
}

// Handle when rotate left button is released
void handleStopRotateLeft() {
  rotatingLeft = false;
//   Serial.println("Rotate Left stopped");
//   Serial.print("rotatingLeft: ");
//   Serial.println(rotatingLeft);
  server.send(200, "text/plain", "Rotate Left stopped");
}

// Handle when rotate right button is released
void handleStopRotateRight() {
  rotatingRight = false;
  server.send(200, "text/plain", "Rotate Right stopped");
}

// Handle when move forward button is released
void handleStopMoveForward() {
  movingForward = false;
  server.send(200, "text/plain", "Move Forward stopped");
}

// Handle when move backward button is released
void handleStopMoveBackward() {
  movingBackward = false;
  server.send(200, "text/plain", "Move Backward stopped");
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
  json += "\"waterTemperature\":" + String(randomFloat(0.0, 3.3)) + ",";
  json += "\"pressure\":" + String(randomFloat(0.0, 3.3)) + ",";
  json += "\"salinity\":" + String(randomFloat(0.0, 3.3)) + ",";
  json += "\"pitch\":" + String(randomFloat(0.0, 3.3)) + ",";
  json += "\"roll\":" + String(randomFloat(0.0, 3.3)) + ",";
  json += "\"yaw\":" + String(randomFloat(0.0, 3.3)) + ",";
  json += "\"batteryVoltage\":" + String(randomFloat(0.0, 3.3)) + ",";
  json += "\"compassCourse\":" + String(randomFloat(0.0, 360.0)) + ",";
  json += "\"gnssCoordinates\":\"" + String(randomFloat(-90.0, 90.0), 6) + ", " + String(randomFloat(-180.0, 180.0), 6) + "\",";
  json += "\"internalTemperature\":" + String(randomFloat(15.0, 25.0)) + ",";
  json += "\"internalPressure\":" + String(randomFloat(0.0, 1.0)) + ",";
  json += "\"internalHumidity\":" + String(randomFloat(10.0, 30.0)) + ",";
  json += "\"adcValue\":" + String(randomFloat(0.0, 3.3));
  json += "}";
  server.send(200, "application/json", json);
}


void setup () {
  Serial.begin(115200);
  pinMode(TRAN_SLEEP_PIN, OUTPUT);
//   digitalWrite(TRAN_SLEEP_PIN, HIGH);
  translationMotor.enableOutputs();
  // Set the maximum speed in steps per second:
  translationMotor.setMaxSpeed(1000);
//   translationMotor.setPinsInverted(true, false, false);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/data", handleData);
  server.on("/idle", handleIdle);
  server.on("/initiateDive", handleInitiateDive);
  server.on("/calibrate", handleCalibrate);
  server.on("/rotateLeft", handleRotateLeft);
  server.on("/rotateRight", handleRotateRight);
  server.on("/moveForward", handleMoveForward);
  server.on("/moveBackward", handleMoveBackward);
  server.on("/stopRotateLeft", handleStopRotateLeft);
  server.on("/stopRotateRight", handleStopRotateRight);
  server.on("/stopMoveForward", handleStopMoveForward);
  server.on("/stopMoveBackward", handleStopMoveBackward);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
    server.handleClient();

    if (movingForward) {
        
        if (!translationMotorRunning) {
        digitalWrite(TRAN_SLEEP_PIN, HIGH);
        translationMotorRunning = true;
        }

        translationMotor.setSpeed(800);
        translationMotor.runSpeed();
    } else if (translationMotorRunning) {
        digitalWrite(TRAN_SLEEP_PIN, LOW);
        translationMotorRunning = false;
    }

    if (movingBackward) {
        
        if (!translationMotorRunning) {
        digitalWrite(TRAN_SLEEP_PIN, HIGH);
        translationMotorRunning = true;
        }

        translationMotor.setSpeed(-800);
        translationMotor.runSpeed();

    } else if (translationMotorRunning) {
        digitalWrite(TRAN_SLEEP_PIN, LOW);
        translationMotorRunning = false;
    }
}




