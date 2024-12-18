#include <Arduino.h>
#include <ICM_20948.h>
#include <KellerLD.h>
#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include <TSYS01.h>
#include <pgmspace.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include "page.h"  // Include the header file with HTML content


#define stepsPerRevolution 200
#define STEPS_PER_CM 600
#define STEPS_PER_DEGREE 10 //Ta reda på detta värde

#define TRANSLATION_MIN_POSITION_CM -1
#define TRANSLATION_MAX_POSITION_CM 1
#define TRANSLATIONMOTOR_STEP_SIZE 5

#define ROTATION_MIN_DEGREE -90
#define ROTATION_MAX_DEGREE 90
#define ROTATIONMOTOR_STEP_SIZE 1

#define AD0_VAL 1

#define RPM 186

#define TRAN_DIR_PIN 17 //Tidigare 9
#define TRAN_STEP_PIN 16 //Tidigare 46
#define TRAN_SLEEP_PIN 15 //Tidigare 3

#define ROT_DIR_PIN 9
#define ROT_STEP_PIN 8
#define ROT_SLEEP_PIN 18

#define SDA_PIN 13
#define SCL_PIN 14

#include "DRV8825.h"
#define MODE0 10
#define MODE1 11
#define MODE2 12
DRV8825 transtepper(stepsPerRevolution, TRAN_DIR_PIN, TRAN_STEP_PIN, TRAN_SLEEP_PIN, MODE0, MODE1, MODE2);
DRV8825 rotstepper(stepsPerRevolution, ROT_DIR_PIN, ROT_STEP_PIN, ROT_SLEEP_PIN, MODE0, MODE1, MODE2);

#define SOFT_POT_PIN 2 // Pin connected to softpot wiper
#define PUMP_PIN 5
#define VALVE_PIN 6
#define DROPWEIGHT_PIN 4

const int GRAPH_LENGTH = 40; // Length of line graph

bool glideUp = false;
bool glideDown = false;
bool transmit = true;
float n_dyk = 0;

bool dropweight = false;

bool isIdle = false;
bool isDiving = false;
bool isCalibrating = false;

float displaypitch = 0;
float displayroll = 0;
float displayyaw = 0;
float displaypressure = 0;
float displaytemperature = 0;
float displaypotentiometer = 0;

struct SensorData {
    float pitch;
    float roll;
    float yaw;
    float depth;
    float pressure;
    float potentiometer;
    float temperature;
};

QueueHandle_t sensorDataQueue;

KellerLD Psensor;

TSYS01 Tsensor;

ICM_20948_I2C myICM;

OpenLog myLog;

String fileName = "SD-test10_240612.txt";

void openLogSetup();
void moveRotationMotor(SensorData receivedData, float &currentDegree, float &rollSP, bool &rotationmotorRunning);
void moveTranslationMotor(SensorData receivedData, float &currentPosition, float &pitchSP, bool translationmotorRunning);
void steppermotor(void* pvParameters);
void datagathering(void* pvParameters);

void writeSD(String measurement) {
  myLog.append(fileName); //Bestämmer att vi ska sriva i denna filen 
  myLog.println(measurement); //Skriver till filen
  myLog.syncFile();
}

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
  delay(1000);
  currentState = "Diving";
  Serial.println("Diving");
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
  json += "\"waterTemperature\":" + String(displaytemperature) + ",";
  json += "\"pressure\":" + String(displaypressure) + ",";
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
  json += "\"adcValue\":" + String(displaypotentiometer);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
    Serial.begin(115200);

    transtepper.begin(RPM);
    transtepper.enable();
    rotstepper.begin(RPM);
    rotstepper.enable();

    pinMode(TRAN_SLEEP_PIN, OUTPUT);
    pinMode(TRAN_DIR_PIN, OUTPUT);
    pinMode(ROT_SLEEP_PIN, OUTPUT);
    pinMode(ROT_DIR_PIN, OUTPUT);

    pinMode(SOFT_POT_PIN, INPUT);

    pinMode(PUMP_PIN, OUTPUT);
    pinMode(VALVE_PIN, OUTPUT);

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

    Psensor.init();
    Psensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    if (Psensor.isInitialized()) {
        Serial.println("Pressure sensor connected.");
    } else {
        Serial.println("Pressure sensor not connected.");
    }

    Tsensor.init();

    while (!Tsensor.init()) {
        Serial.println("TSYS01 device failed to initialize!");
        delay(1000);
    }

    openLogSetup();

    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorDataQueue == NULL) {
        Serial.println("Failed to create queue");
        return;
    }

    xTaskCreatePinnedToCore(
        datagathering,     // Function to implement the task
        "Measurement and storing of data",   // Name of the task
        2048,      // Stack size in bytes
        NULL,      // Task input parameter
        1,         // Priority of the task
        NULL,      // Task handle.
        0          // Core where the task should run
    );

    xTaskCreatePinnedToCore(
        steppermotor,
        "driving of stepper motor",
        2048,
        NULL,
        1,
        NULL,
        1
    );
}

void loop() {
    server.handleClient();
}

void openLogSetup() {
  myLog.begin();

//   Serial.begin(115200); //9600bps is used for debug statements
  Serial.println("OpenLog Status Test");

  byte status = myLog.getStatus();

  Serial.print("Status byte: 0x");
  if(status < 0x10) {
    Serial.print("0");
  }
  Serial.println(status, HEX);

  if (status == 0xFF) {
    Serial.println("OpenLog failed to respond. Freezing.");
    while(1); //Freeze!
  }

  if(status & 1<<STATUS_SD_INIT_GOOD) {
    Serial.println("SD card is good");
  }  
  else {
    Serial.println("SD init failure. Is the SD card present? Formatted?");
  }
  if(status & 1<<STATUS_IN_ROOT_DIRECTORY) {    
    Serial.println("Root directory open");
  }
  else {
    Serial.println("Root failed to open. Is SD card present? Formatted?");
  }
  if (status & 1<<STATUS_FILE_OPEN) {
    Serial.println("Log file open and ready for recording");
  }
  else {
    Serial.println("No log file open. Use append command to start a new log.");
  }
  Serial.println("Done!");
  myLog.create(fileName);
  myLog.syncFile();
}

void moveRotationMotor (SensorData receivedData, float &currentDegree, float &rollSP, bool &rotationmotorRunning) {
    float roll_error = rollSP - (receivedData.roll * 180 / PI);
    int rot_direction = (roll_error < 0) ? 1 : -1;
    float nextDegree = currentDegree + rot_direction * (ROTATIONMOTOR_STEP_SIZE / (float)STEPS_PER_DEGREE);

    if (fabs(roll_error) > 5) {
        if (!rotationmotorRunning) {
            digitalWrite(ROT_SLEEP_PIN, HIGH);
            rotationmotorRunning = true;
        }

        if (nextDegree < ROTATION_MAX_DEGREE && nextDegree > ROTATION_MIN_DEGREE) {
            rotstepper.setMicrostep(1);
            rotstepper.move(rot_direction > 0 ? ROTATIONMOTOR_STEP_SIZE : -ROTATIONMOTOR_STEP_SIZE);
            currentDegree = nextDegree;
        } else {
            if (!rotationmotorRunning) {
                digitalWrite(ROT_SLEEP_PIN, HIGH);
                rotationmotorRunning = true;
            }

            if ((rot_direction > 0 && currentDegree >= ROTATION_MAX_DEGREE) || (rot_direction < 0 && currentDegree <= ROTATION_MIN_DEGREE)) {
                rotstepper.setMicrostep(1);
                rotstepper.move(rot_direction > 0 ? ROTATIONMOTOR_STEP_SIZE : -ROTATIONMOTOR_STEP_SIZE);
                currentDegree = nextDegree;
            }
        }
    } else {
        if (rotationmotorRunning) {
            digitalWrite(ROT_SLEEP_PIN, LOW);
            rotationmotorRunning = false;
        }
    }
}

void moveTranslationMotor (SensorData receivedData, float &currentPosition, float &pitchSP, bool translationmotorRunning) {
    //Translation motor
    float pitch_error = pitchSP - (receivedData.pitch * 180 / PI);
    int transl_direction = (pitch_error < 0) ? 1 : -1;
    float nextPosition = currentPosition + transl_direction * (TRANSLATIONMOTOR_STEP_SIZE / (float)STEPS_PER_CM);

   if (fabs(pitch_error) > 5) {
        if (!translationmotorRunning) {
            digitalWrite(TRAN_SLEEP_PIN, HIGH);
            translationmotorRunning = true;
        }

        if (nextPosition < TRANSLATION_MAX_POSITION_CM && nextPosition > TRANSLATION_MIN_POSITION_CM) {
            transtepper.setMicrostep(1);
            transtepper.move(transl_direction > 0 ? TRANSLATIONMOTOR_STEP_SIZE : -TRANSLATIONMOTOR_STEP_SIZE);
            currentPosition = nextPosition;
        } else {
            if (!translationmotorRunning) {
                digitalWrite(TRAN_SLEEP_PIN, HIGH);
                translationmotorRunning = true;
            }

            if ((transl_direction > 0 && currentPosition >= TRANSLATION_MAX_POSITION_CM) || (transl_direction < 0 && currentPosition <= TRANSLATION_MIN_POSITION_CM)) {
                transtepper.setMicrostep(1);
                transtepper.move(transl_direction > 0 ? TRANSLATIONMOTOR_STEP_SIZE : -TRANSLATIONMOTOR_STEP_SIZE);
                currentPosition = nextPosition;
            }
        }
    } else {
        if (translationmotorRunning) {
            digitalWrite(TRAN_SLEEP_PIN, LOW);
            translationmotorRunning = false;
        }
    }
}

void steppermotor(void* pvParameters) {
    bool translationmotorRunning = false;
    bool rotationmotorRunning = false;
    bool glideDownInitialized = false;
    bool glideUpInitialized = false;
    bool dykcount = false;
    float currentPosition = 0;
    float currentDegree = 0;
    float pitchSP = 0;
    float rollSP = 0;
    unsigned long glideDownStartTime = 0;
    unsigned long glideUpStartTime = 0;
    const unsigned long ONE_HOUR = 3600000; // 1 hour in milliseconds
    SensorData receivedData;

    while (true) {
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {

            if (isCalibrating == true) {
                currentPosition = 0;
                currentDegree = 0;

                delay(1000);
                currentState = "Calibration Complete";
                isCalibrating = false;
            }
            
            if (transmit == true) {
                pitchSP = -20;
                rollSP = 0;

                moveTranslationMotor(receivedData, currentPosition, pitchSP, translationmotorRunning);
                moveRotationMotor(receivedData, currentDegree, rollSP, rotationmotorRunning);

                if (isDiving == true) {
                    Serial.println("Initierar dyk");
                    Serial.println("");
                    delay(1000);
                    n_dyk = 0;
                    glideDown = true;
                    transmit = false;
                    Serial.println("Går in i dykläge");
                    Serial.println("");
                }

                // bool klartecken = false;

                // if (!klartecken) { //Simulera väntan på klartecken från land
                //     delay(30000);
                //     klartecken = true;
                //     Serial.println("Klartecken givet, dyker");
                //     n_dyk = 0;
                //     glideDown = true;
                //     transmit = false;
                //     Serial.println("Går in i dykläge");
                //     Serial.println("");
                // }
            }

            if (glideDown == true) {
                pitchSP = -20;
                rollSP = 0;

                if (!glideDownInitialized) {
                    Serial.println("Öppnar magnetventil...");
                    glideDownStartTime = millis();
                    digitalWrite(VALVE_PIN, HIGH);
                    delay(10000);

                    if (receivedData.potentiometer >= 3500) {
                        Serial.println("Bälg fylld, stänger magnetventil...");
                        digitalWrite(VALVE_PIN, LOW);
                        glideDownInitialized = true;
                    }

                    // delay(2000);
                    // Serial.println("Reglerar pitch- och rollvinkel");
                    // Serial.println("");
                    // glideDownInitialized = true;
                }

                moveTranslationMotor(receivedData, currentPosition, pitchSP, translationmotorRunning);
                moveRotationMotor(receivedData, currentDegree, rollSP, rotationmotorRunning);

                if (receivedData.depth > 90) {
                    Serial.println("Maxdjup uppnåt, initierar uppstigning");
                    Serial.println("");
                    glideDown = false;
                    glideUp = true;
                }  

                if (receivedData.depth > 110) {
                    Serial.println("Djupare än 110m, släpper dropweight");
                    dropweight = true;
                }

                if (glideDown && millis() - glideDownStartTime >= ONE_HOUR) {
                    Serial.println("En timme har passerat, släpper dropweight");
                    Serial.println("");
                    dropweight = true;
                }
            }

            if (glideUp == true) {
                pitchSP = 20;
                rollSP = 0;

                if (!glideUpInitialized) {
                    Serial.print("Startar pump...");
                    delay(20000);
                    digitalWrite(PUMP_PIN, HIGH);
                    if (receivedData.potentiometer <= 1800) {
                        Serial.println("Blåsa fylld, stänger av pump.");
                        digitalWrite(PUMP_PIN, LOW);
                        glideUpInitialized = true;
                    }
                    delay(2000);
                    Serial.println("Reglerar pitch- och rollvinkel");
                    Serial.println("");

                    // glideUpInitialized = true;
                }

                moveTranslationMotor(receivedData, currentPosition, pitchSP, translationmotorRunning);
                moveRotationMotor(receivedData, currentDegree, rollSP, rotationmotorRunning);

                if (receivedData.depth < 5) {
                    
                    if (!dykcount) {
                        n_dyk += 1;
                        Serial.println(n_dyk);
                        dykcount = true;
                    }

                    if (n_dyk < 3) {
                        glideDown = true;
                        glideUp = false;
                        Serial.println("Yttryck uppnått, dyker igen");
                        Serial.println("");
                    }

                    if (n_dyk >= 3) {
                        transmit = true;
                        glideUp = false;
                        Serial.println("Alla dyk genomförda, går upp till ytan");
                        Serial.println("");
                    }
                }

                if (glideUp && millis() - glideUpStartTime >= ONE_HOUR) {
                    Serial.println("En timme har passerat, släpper dropweight");
                    Serial.println("");
                    dropweight = true;
                }
            }

            if (dropweight == true) {
                digitalWrite(DROPWEIGHT_PIN, HIGH);
            }

            // moveTranslationMotor(receivedData, currentPosition, pitchSP, translationmotorRunning);
            // moveRotationMotor(receivedData, currentDegree, rollSP, rotationmotorRunning);

            // Serial.println(receivedData.potentiometer);

            // if (receivedData.potentiometer > 3500) {
            //     Serial.println("Bälg full, stänger magnetventil...");
            // }

            // if (receivedData.potentiometer < 2300) {
            //     Serial.println("Bälg tom, öppnar magnetventil...");
            // }
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

            Psensor.read();
            data.depth = Psensor.depth();
            data.pressure = Psensor.pressure();
            displaypressure = data.pressure;

            Tsensor.read();
            data.temperature = Tsensor.temperature();
            displaytemperature = data.temperature;

            data.potentiometer = analogRead(SOFT_POT_PIN);
            displaypotentiometer = data.potentiometer;

            if (xQueueSend(sensorDataQueue, &data, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send data to queue");
            }

            String logData = "Pitch: " + String(data.pitch) + " degrees, " +
                             "Roll: " + String(data.roll) + " degrees, " +
                             "Yaw: " + String(data.yaw) + " degrees, " +
                             "Depth: " + String(data.depth) + " m, " +
                             "Pressure: " + String(data.pressure) + " mbar, " +
                             "Temperature: " + String(data.temperature) + " Celsius";
                            //  "Batterycurrent: " + String(current) + " A";
            writeSD(logData);


            } else {
                Serial.println("Sensor data not ready");
            }
        
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adding a delay to reduce bus congestion and improve stability
    }
}

