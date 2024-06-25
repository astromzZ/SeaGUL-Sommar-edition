#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include "page.h"  // Include the header file with HTML content
#include <ICM_20948.h>
#include <pgmspace.h>

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

#define SDA_PIN 13
#define SCL_PIN 14
#define AD0_VAL 1

struct SensorData {
    float pitch;
    float roll;
    // float depth;
    // float pressure;
    // float potentiometer;
};

float displaypitch = 0;
float displayroll = 0;

bool isIdle = false;
bool isDiving = false;
bool isCalibrating = false;

ICM_20948_I2C myICM;
QueueHandle_t sensorDataQueue;

void steppermotor(void* pvParameters);
void datagathering(void* pvParameters);

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

void handleIdle() {
  isIdle = true;
  isDiving = false;
  isCalibrating = false;
  Serial.println("Set to Idle");
  server.send(200, "text/plain", "Set to Idle");
}

void handleInitiateDive() {
  isIdle = false;
  isDiving = true;
  isCalibrating = false;
  Serial.println("Dive Initiated");
  server.send(200, "text/plain", "Dive Initiated");
}

void handleCalibrate() {
  isIdle = false;
  isDiving = false;
  isCalibrating = true;
  Serial.println("Calibration Started");
  server.send(200, "text/plain", "Calibration Started");
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
  json += "\"pitch\":" + String(displaypitch) + ",";
  json += "\"roll\":" + String(displayroll) + ",";
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

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  bool initialized = false;
  while (!initialized) {
      myICM.begin(Wire, AD0_VAL);
      Serial.print(F("Initialization of the sensor returned: "));
      Serial.println(myICM.statusString());
      if (myICM.status != ICM_20948_Stat_Ok) {
          Serial.println("Trying again...");
          delay(500);
      } else {
          initialized = true;
      }
  }

  sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
  if (sensorDataQueue == NULL) {
      Serial.println("Failed to create queue");
      return;
  }

  xTaskCreatePinnedToCore(
      datagathering,     // Function to implement the task
      "Measurement and storing of data",   // Name of the task
      8192,      // Stack size in bytes
      NULL,      // Task input parameter
      1,         // Priority of the task
      NULL,      // Task handle.
      0          // Core where the task should run
  );

  xTaskCreatePinnedToCore(
      steppermotor,
      "driving of stepper motor",
      8192,
      NULL,
      1,
      NULL,
      1
  );

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/data", handleData);
  server.on("/idle", handleIdle);
  server.on("/initiateDive", handleInitiateDive);
  server.on("/calibrate", handleCalibrate);

  server.begin();
  Serial.println("HTTP server started");
}

// Loop function
void loop() {
  server.handleClient();
}

void steppermotor(void* pvParameters) {
    while (true) {
        SensorData receivedData;
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
            // Do something with the data
            // Serial.println("Received data from queue");
            Serial.print("Pitch: ");
            Serial.print(receivedData.pitch * 180 / PI);
            Serial.print(" degrees, Roll: ");
            Serial.print(receivedData.roll * 180 / PI);
            Serial.println(" degrees");

        } else {
            Serial.println("Failed to receive data from queue");
        }
    }
}

void datagathering(void* pvParameters) {

    while (true) {
        if (myICM.dataReady()) {
            SensorData data;
            myICM.getAGMT(); // The values are only updated when you call 'getAGMT'

            // Calculate pitch and roll
            data.pitch = atan2(myICM.accY(), sqrt(myICM.accX() * myICM.accX() + myICM.accZ() * myICM.accZ()));
            data.roll = -atan2(-myICM.accX(), myICM.accZ());

            // Calculate display pitch and roll
            displaypitch = data.pitch * 180 / PI;
            displayroll = data.roll * 180 / PI;

            // Psensor.read();

            // data.depth = Psensor.depth();
            // data.pressure = Psensor.pressure();

            // data.potentiometer = analogRead(SOFT_POT_PIN);

            if (xQueueSend(sensorDataQueue, &data, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send data to queue");
            }

            // Tsensor.read();

            // float temperature = Tsensor.temperature(); 

            // String logData = "Pitch: " + String(data.pitch * 180 / PI) + " degrees, " +
            //                  "Roll: " + String(data.roll * 180 / PI) + " degrees, ";
                            //  "Depth: " + String(data.depth) + " m, " +
                            //  "Pressure: " + String(data.pressure) + " mbar, " +
                            //  "Temperature: " + String(temperature) + " Celsius";
                            //  "Current: " + String(current) + " A";

            // writeSD(logData);

        } else {
            Serial.println("Sensor data not ready");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adding a delay to reduce bus congestion and improve stability
    }
}