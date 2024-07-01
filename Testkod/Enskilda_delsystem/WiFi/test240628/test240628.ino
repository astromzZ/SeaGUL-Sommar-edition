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
    float yaw;
    // float depth;
    // float pressure;
    // float potentiometer;
};

float displaypitch = 0;
float displayroll = 0;
float displayyaw = 0;

bool isIdle = false;
bool isDiving = false;
bool isCalibrating = false;

bool rotatingLeft = false;
bool rotatingRight = false;
bool movingForward = false;
bool movingBackward = false;

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
  currentState = "Diving";
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


void handleRotateLeft() {
  rotatingLeft = true;
  Serial.println("Rotate Left command received");
  Serial.print("rotatingLeft: ");
  Serial.println(rotatingLeft);
  server.send(200, "text/plain", "Rotate Left started");
}

void handleRotateRight() {
  rotatingRight = true;
//   Serial.println("Rotate Right command received");
//   Serial.print("rotatingRight: ");
//   Serial.println(rotatingRight);
  server.send(200, "text/plain", "Rotate Right started");
}

void handleMoveForward() {
  movingForward = true;
//   Serial.println("Move Forward command received");
//   Serial.print("movingForward: ");
//   Serial.println(movingForward);
  server.send(200, "text/plain", "Move Forward started");
}

void handleMoveBackward() {
  movingBackward = true;
//   Serial.println("Move Backward command received");
//   Serial.print("movingBackward: ");
//   Serial.println(movingBackward);
  server.send(200, "text/plain", "Move Backward started");
}

void handleStopRotateLeft() {
  rotatingLeft = false;
  Serial.println("Rotate Left stopped");
  Serial.print("rotatingLeft: ");
  Serial.println(rotatingLeft);
  server.send(200, "text/plain", "Rotate Left stopped");
}

void handleStopRotateRight() {
  rotatingRight = false;
  server.send(200, "text/plain", "Rotate Right stopped");
}

void handleStopMoveForward() {
  movingForward = false;
  server.send(200, "text/plain", "Move Forward stopped");
}

void handleStopMoveBackward() {
  movingBackward = false;
//   Serial.println("Move Backward stopped");
//   Serial.print("movingBackward: ");
//   Serial.println(movingBackward);
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
  json += "\"waterTemperature\":" + String(randomFloat(15.0, 25.0)) + ",";
  json += "\"pressure\":" + String(randomFloat(1.0, 5.0)) + ",";
  json += "\"salinity\":" + String(randomFloat(30.0, 50.0)) + ",";
  json += "\"pitch\":" + String(displaypitch) + ",";
  json += "\"roll\":" + String(displayroll) + ",";
  json += "\"yaw\":" + String(displayyaw) + ",";
  json += "\"batteryVoltage\":" + String(randomFloat(22.0, 38.5)) + ",";
  json += "\"compassCourse\":" + String(randomFloat(0.0, 360.0)) + ",";
  json += "\"gnssCoordinates\":\"" + String(randomFloat(-90.0, 90.0), 6) + ", " + String(randomFloat(-180.0, 180.0), 6) + "\",";
  json += "\"internalTemperature\":" + String(randomFloat(15.0, 25.0)) + ",";
  json += "\"internalPressure\":" + String(randomFloat(0.0, 1.0)) + ",";
  json += "\"internalHumidity\":" + String(randomFloat(10.0, 30.0)) + ",";
  json += "\"adcValue\":" + String(randomFloat(0.0, 4095.0));
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

  Serial.println("Device connected!");

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
    // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success) {
      Serial.println("DMP configuration successful");
  } else {
      Serial.println("DMP configuration failed");
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

// Loop function
void loop() {
  server.handleClient();
}

void steppermotor(void* pvParameters) {
    while (true) {
        SensorData receivedData;
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdTRUE) {

            // if(rotatingLeft == true){
            //     Serial.println("Rotate Left");
            // }

            if(rotatingRight == true){
                Serial.println("Rotate Right");
            }

            // if(movingForward == true){
            //     Serial.println("Move Forward");
            // }

            // if(movingBackward == true){
            //     Serial.println("Move Backward");
            // }
            // Serial.print(F("Backward: "));
            // Serial.print(movingBackward);
            // Serial.print(F(" Forward: "));
            // Serial.print(movingForward);
            // Serial.print(F(" Left: "));
            // Serial.print(rotatingLeft);
            // Serial.print(F(" Right: "));
            // Serial.println(rotatingRight);

            // Serial.print(F("Roll:"));
            // Serial.print(receivedData.roll, 1);
            // Serial.print(F(" Pitch:"));
            // Serial.print(receivedData.pitch, 1);
            // Serial.print(F(" Yaw:"));
            // Serial.println(receivedData.yaw, 1);

        } else {
            Serial.println("Failed to receive data from queue");
        }
    }
}

void datagathering(void* pvParameters) {
  while (true) {
      icm_20948_DMP_data_t IMUdata;
      myICM.readDMPdataFromFIFO(&IMUdata);

    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
        SensorData data;
        // myICM.getAGMT(); // The values are only updated when you call 'getAGMT'

        if ((IMUdata.header & DMP_header_bitmap_Quat6) > 0) {
          // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
          // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
          // The quaternion data is scaled by 2^30.

          //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

          // Scale to +/- 1
          double q1 = ((double)IMUdata.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
          double q2 = ((double)IMUdata.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
          double q3 = ((double)IMUdata.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

          double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

          double q2sqr = q2 * q2;

          // roll (x-axis rotation)
          double t0 = +2.0 * (q0 * q1 + q2 * q3);
          double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
          data.roll = atan2(t0, t1) * 180.0 / PI;

          // pitch (y-axis rotation)
          double t2 = +2.0 * (q0 * q2 - q3 * q1);
          t2 = t2 > 1.0 ? 1.0 : t2;
          t2 = t2 < -1.0 ? -1.0 : t2;
          data.pitch = asin(t2) * 180.0 / PI;

          // yaw (z-axis rotation)
          double t3 = +2.0 * (q0 * q3 + q1 * q2);
          double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
          data.yaw = atan2(t3, t4) * 180.0 / PI;


          // Calculate display pitch and roll
          displaypitch = data.pitch;
          displayroll = data.roll;
          displayyaw = data.yaw;

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
        }

    // } else {
    //     Serial.println("Sensor data not ready");
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Adding a delay to reduce bus congestion and improve stability
  }
}