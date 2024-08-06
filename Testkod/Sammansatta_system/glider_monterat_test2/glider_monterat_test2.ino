#include <Arduino.h>
#include <Wire.h>
#include <pgmspace.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include "page.h"               // Include the header file with HTML content


//.............................................................................

//Pin definitions for the ESP32 S3 

#define TRAN_DIR_PIN            17 //Translation motor direction pin
#define TRAN_STEP_PIN           16 //Translation motor step pin
#define TRAN_SLEEP_PIN          15 //Trasnlation motor sleep pin
#define ROT_DIR_PIN              9 //Rotation motor direction pin
#define ROT_STEP_PIN             8 //Rotation motor step pin
#define ROT_SLEEP_PIN           18 //Rotation motor sleep pin
#define SDA_PIN                 13 //SDA pin for I2C communication
#define SCL_PIN                 14 //SCL pin for I2C communication
// #define MODE0                10 //Microstepping mode pin 0
// #define MODE1                11 //Microstepping mode pin 1
// #define MODE2                12 //Microstepping mode pin 2
#define SOFT_POT_PIN             4 //Pin connected to softpot wiper
#define PUMP_PIN                 5 //Pin connected to pump
#define VALVE_PIN                6 //Pin connected to valve 
#define DROPWEIGHT_PIN           2 //Pin connected to dropweight
#define BAT_VOLTAGE_PIN         10 //Pin connected to bat voltage
#define BAT_VOLTAGE_MONITOR_PIN 11 //Pin connected to bat voltage monitor
#define RX_PIN                  36 // RX pin configuration
#define TX_PIN                  35 // TX pin configuration
#define POKE_PIN                48 // Pin to check if the device has been poked by the AGT
#define ACTIVATION_PIN          45 // Activation pin for initializing UART communication with AGT

//.............................................................................

//Stepper motor definitions

#include <AccelStepper.h> //Library for the stepper motor

AccelStepper translationMotor(AccelStepper::DRIVER, TRAN_STEP_PIN, TRAN_DIR_PIN);  //Create an instance of the stepper motor driver for the translation motor
AccelStepper rotationMotor(AccelStepper::DRIVER, ROT_STEP_PIN, ROT_DIR_PIN); //Create an instance of the stepper motor driver for the rotation motor

//.............................................................................

//Keep track of the number of dives the glider has done
int n_dyk = 0;
long totalNumberofDives = 0; // Total number of dives the glider has done since the start of the program

//Flag to check if the dropweight has been released
bool dropweight = false;

bool isIdle = false;
bool isDiving = false;
bool isCalibrating = false;

//Flags that are used to control the movement of the glider when in Idle state. 
//Can be set to true via the SeaGUL webpage.
bool rotatingLeft = false;
bool rotatingRight = false;
bool movingForward = false;
bool movingBackward = false;


//Vent and pump states
bool pumpState = false; // false = off, true = on
bool ventState = false; // false = closed, true = open
String errorMessage = "";
bool errorMessageSent = false;

bool batteryWiFiCheck = false;

//Values that are displayed on the SeaGUL webpage
float displaypitch = 0;
float displayroll = 0;
float displayyaw = 0;
float displaypressure = 0;
float displaytemperature = 0;
float displaypotentiometer = 0;
float displayconductivity = 0;
float displaybattery = 0;
float displayRotationSteps = 0;
float displayTranslationSteps = 0;

//Used to keep track on where the weight is positioned
float nextPosition = 0;
float nextDegree = 0;

unsigned long stateStartMillis = 0; // Variable to store the start time of the states GlidingDown and GlidingUp

float Vbat = 30.0; // Variable to store the battery voltage
#define V_LOW 29.0 // Low battery voltage threshold

// Struct to hold sensor data
struct SensorData {
    float pitch;
    float roll;
    float yaw;
    float depth;
    float pressure;
    float potentiometer;
    float temperature;
};
QueueHandle_t sensorDataQueue; // Queue to hold sensor data

float lastconductivityreading = 0; // Variable to store the conductivity
bool readingConductivity = false; // Flag to indicate if the conductivity is being read
uint32_t next_ECread_time = 0;        


// Enum to keep track of the state of the glider
enum GliderState {
    Glider_init,
    Idle,
    Diving,
    Calibrating,
    GlidingDown,
    GlidingUp,
    DropWeight,
    Surface
};
GliderState gliderState = Glider_init; // Initial state of the glider
bool startSetup = false; // Flag to indicate if the setup should run, activated by the SeaGUL webpage

// Pressure sensor
#include <KellerLD.h>
KellerLD Psensor;

// Temperature sensor
#include <TSYS01.h>
TSYS01 Tsensor;

// Orientation sensor
#include <ICM_20948.h>
#define AD0_VAL 1 //The value of the AD0 pin on the ICM-20948 sensor
ICM_20948_I2C myICM;

// SD-card logging
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
OpenLog myLog;
String fileName = "SD-test10_240612.txt"; //Name of the file that the data will be stored in

// Salinity sensor
#include <Ezo_i2c.h>
#include <sequencer2.h>
#include <Ezo_i2c_util.h>

// Ezo_board PH = Ezo_board(99, "PH");       //create a PH circuit object, who's address is 99 and name is "PH"
Ezo_board EC = Ezo_board(100, "EC");      //create an EC circuit object who's address is 100 and name is "EC"

HardwareSerial mySerial(1); // Initialize UART1


//.............................................................................

// Function prototypes

void openLogSetup();
void moveRotationMotor(SensorData receivedData, float &currentDegree, float &rollSP, bool &rotationmotorRunning);
void moveTranslationMotor(SensorData receivedData, float &currentPosition, float &pitchSP, bool translationmotorRunning);
void glidercontrol(void* pvParameters);
void datagathering(void* pvParameters);
void writeSD(String measurement) {
  myLog.append(fileName); //Bestämmer att vi ska sriva i denna filen 
  myLog.println(measurement); //Skriver till filen
  myLog.syncFile();
}
// void sequencerstep1();  //forward declarations of functions to use them in the sequencer before defining them
// void sequencerstep2();
// Sequencer2 Seq(&sequencerstep1, 1000, &sequencerstep2, 0);  //calls the steps in sequence with time in between them
//.............................................................................

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

void handleSetup() {
  startSetup = true;
  server.send(200, "text/plain", "Setup started");
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
  rotatingRight = false;
//   Serial.println("Rotate Left command received");
//   Serial.print("rotatingLeft: ");
//   Serial.println(rotatingLeft);
  server.send(200, "text/plain", "Rotate Left started");
}

// Handle when rotate right button is pressed
void handleRotateRight() {
  rotatingRight = true;
  rotatingLeft = false;
//   Serial.println("Rotate Right command received");
//   Serial.print("rotatingRight: ");
//   Serial.println(rotatingRight);
  server.send(200, "text/plain", "Rotate Right started");
}

// Handle when move forward button is pressed
void handleMoveForward() {
  movingForward = true;
  movingBackward = false;
//   Serial.println("Move Forward command received");
//   Serial.print("movingForward: ");
//   Serial.println(movingForward);
  server.send(200, "text/plain", "Move Forward started");
}

// Handle when move backward button is pressed
void handleMoveBackward() {
  movingBackward = true;
  movingForward = false;
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

void handleTogglePump() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      pumpState = true;
    } else if (state == "off") {
      pumpState = false;
    }
    server.send(200, "text/plain", "Pump state: " + state);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleToggleVent() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "open") {
      ventState = true;
    } else if (state == "closed") {
      ventState = false;
    }
    server.send(200, "text/plain", "Pump state: " + state);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleErrorMessage() {
    server.send(200, "text/plain", errorMessage); // Serve the captured error message
    errorMessage = ""; // Clear the error message after serving once
}

void handleCheckBattery() {
    batteryWiFiCheck = true;
    server.send(200, "text/plain", "Battery voltage: " + String(displaybattery) + "V");
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
  json += "\"salinity\":" + String(displayconductivity) + ",";
  json += "\"pitch\":" + String(displaypitch) + ",";
  json += "\"roll\":" + String(displayroll) + ",";
  json += "\"yaw\":" + String(displayyaw) + ",";
  json += "\"batteryVoltage\":" + String(displaybattery) + ",";
  json += "\"compassCourse\":" + String(randomFloat(0.0, 360.0)) + ",";
  json += "\"gnssCoordinates\":\"" + String(randomFloat(-90.0, 90.0), 6) + ", " + String(randomFloat(-180.0, 180.0), 6) + "\",";
  json += "\"internalTemperature\":" + String(randomFloat(15.0, 25.0)) + ",";
  json += "\"internalPressure\":" + String(randomFloat(0.0, 1.0)) + ",";
  json += "\"internalHumidity\":" + String(randomFloat(10.0, 30.0)) + ",";
  json += "\"rotationAngle\":" + String(displayRotationSteps) + ",";
  json += "\"translationPosition\":" + String(displayTranslationSteps) + ",";
  json += "\"adcValue\":" + String(displaypotentiometer);
  json += "}";
  server.send(200, "application/json", json);
}

void getVBat() {
  digitalWrite(BAT_VOLTAGE_MONITOR_PIN, HIGH); // Enable the voltage monitor

  delay(500); // Wait for the voltage to stabilize
  // Read the battery voltage
  Vbat = ((float)analogRead(BAT_VOLTAGE_PIN)) * 12 * 3.3 / 4095; // Convert the analog reading to voltage 10/120
  displaybattery = Vbat;

  digitalWrite(BAT_VOLTAGE_MONITOR_PIN, LOW); // Disable the voltage monitor
}

void setup() {
    Serial.begin(115200);
    mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    pinMode(TRAN_SLEEP_PIN, OUTPUT);
    digitalWrite(TRAN_SLEEP_PIN, LOW);
    pinMode(TRAN_DIR_PIN, OUTPUT);
    pinMode(ROT_SLEEP_PIN, OUTPUT);
    digitalWrite(ROT_SLEEP_PIN, LOW);
    pinMode(ROT_DIR_PIN, OUTPUT);
    pinMode(SOFT_POT_PIN, INPUT);
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    pinMode(VALVE_PIN, OUTPUT);
    digitalWrite(VALVE_PIN, LOW);
    pinMode(DROPWEIGHT_PIN, OUTPUT); 
    digitalWrite(DROPWEIGHT_PIN, LOW);
    pinMode(POKE_PIN, INPUT);
    pinMode(ACTIVATION_PIN, OUTPUT);
    digitalWrite(ACTIVATION_PIN, LOW);
    pinMode(BAT_VOLTAGE_PIN, INPUT);
    pinMode(BAT_VOLTAGE_MONITOR_PIN, OUTPUT);
    digitalWrite(BAT_VOLTAGE_MONITOR_PIN, LOW);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    while (!Serial) // Wait for the user to open the serial monitor
      ;
    delay(100);
    Serial.println();
    Serial.println();
    Serial.println(F("REVERE Glider SeaGUL"));
    Serial.println(F("Monterat test"));
    Serial.println();

    //empty the serial buffer
    while(Serial.available() > 0)
      Serial.read();

    //wait for the user to press any key before beginning
    Serial.println(F("Please check that the Serial Monitor is set to 115200 Baud"));
    Serial.println(F("Then click Send to start the test."));
    Serial.println();
    while(Serial.available() == 0)
      ;

    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorDataQueue == NULL) {
        Serial.println("Failed to create queue");
        return;
    }

    // Seq.reset(); //reset the sequencer

    xTaskCreatePinnedToCore(
        datagathering,                       // Function to implement the task
        "Measurement and storing of data",   // Name of the task
        4096,                                // Stack size in bytes
        NULL,                                // Task input parameter
        1,                                   // Priority of the task
        NULL,                                // Task handle.
        0                                    // Core where the task should run
    );

    xTaskCreatePinnedToCore(
        glidercontrol,                      // Function to implement the task
        "Descision making for glider",      // Name of the task
        4096,                               // Stack size in bytes
        NULL,                               // Task input parameter
        1,                                  // Priority of the task
        NULL,                               // Task handle.
        1                                   // Core where the task should run
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
    server.on("/togglePump", handleTogglePump);
    server.on("/toggleVent", handleToggleVent);
    server.on("/errorMessage", handleErrorMessage);
    server.on("/checkBattery", handleCheckBattery);
    
    server.begin();
    Serial.println("HTTP server started");

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
    // float roll_error = rollSP - (receivedData.roll * 180 / PI);
    // int rot_direction = (roll_error < 0) ? 1 : -1;
    // nextDegree = currentDegree + rot_direction * (ROTATIONMOTOR_STEP_SIZE / (float)STEPS_PER_DEGREE);

    // if (fabs(roll_error) > 5) {
    //     if (!rotationmotorRunning) {
    //         digitalWrite(ROT_SLEEP_PIN, HIGH);
    //         rotationmotorRunning = true;
    //     }

    //     if (nextDegree < ROTATION_MAX_DEGREE && nextDegree > ROTATION_MIN_DEGREE) {
    //         rotstepper.setMicrostep(1);
    //         rotstepper.move(rot_direction > 0 ? ROTATIONMOTOR_STEP_SIZE : -ROTATIONMOTOR_STEP_SIZE);
    //         currentDegree = nextDegree;
    //     } else {
    //         if (!rotationmotorRunning) {
    //             digitalWrite(ROT_SLEEP_PIN, HIGH);
    //             rotationmotorRunning = true;
    //         }

    //         if ((rot_direction > 0 && currentDegree >= ROTATION_MAX_DEGREE) || (rot_direction < 0 && currentDegree <= ROTATION_MIN_DEGREE)) {
    //             rotstepper.setMicrostep(1);
    //             rotstepper.move(rot_direction > 0 ? ROTATIONMOTOR_STEP_SIZE : -ROTATIONMOTOR_STEP_SIZE);
    //             currentDegree = nextDegree;
    //         }
    //     }
    // } else {
    //     if (rotationmotorRunning) {
    //         digitalWrite(ROT_SLEEP_PIN, LOW);
    //         rotationmotorRunning = false;
    //     }
    // }
}

void moveTranslationMotor (SensorData receivedData, float &currentPosition, float &pitchSP, bool translationmotorRunning) {
    //Translation motor
//     float pitch_error = pitchSP - (receivedData.pitch * 180 / PI);
//     int transl_direction = (pitch_error < 0) ? 1 : -1;
//     nextPosition = currentPosition + transl_direction * (TRANSLATIONMOTOR_STEP_SIZE / (float)STEPS_PER_CM);

//    if (fabs(pitch_error) > 5) {
//         if (!translationmotorRunning) {
//             digitalWrite(TRAN_SLEEP_PIN, HIGH);
//             translationmotorRunning = true;
//         }

//         if (nextPosition < TRANSLATION_MAX_POSITION_CM && nextPosition > TRANSLATION_MIN_POSITION_CM) {
//             transtepper.setMicrostep(1);
//             transtepper.move(transl_direction > 0 ? TRANSLATIONMOTOR_STEP_SIZE : -TRANSLATIONMOTOR_STEP_SIZE);
//             currentPosition = nextPosition;
//         } else {
//             if (!translationmotorRunning) {
//                 digitalWrite(TRAN_SLEEP_PIN, HIGH);
//                 translationmotorRunning = true;
//             }

//             if ((transl_direction > 0 && currentPosition >= TRANSLATION_MAX_POSITION_CM) || (transl_direction < 0 && currentPosition <= TRANSLATION_MIN_POSITION_CM)) {
//                 transtepper.setMicrostep(1);
//                 transtepper.move(transl_direction > 0 ? TRANSLATIONMOTOR_STEP_SIZE : -TRANSLATIONMOTOR_STEP_SIZE);
//                 currentPosition = nextPosition;
//             }
//         }
//     } else {
//         if (translationmotorRunning) {
//             digitalWrite(TRAN_SLEEP_PIN, LOW);
//             translationmotorRunning = false;
//         }
//     }
}

void controlRotationMotor (float &rotation_direction, bool rotationmotorRunning) {
    rotationMotor.setSpeed(100 * rotation_direction);

    if (rotationMotor.currentPosition() >= -250 && rotationMotor.currentPosition() <= 250) {
        rotationMotor.runSpeed();
    } else if (rotationMotor.currentPosition() <= -250 && rotation_direction == 1) {
        rotationMotor.runSpeed();
    } else if (rotationMotor.currentPosition() >= 250 && rotation_direction == -1) {
        rotationMotor.runSpeed();
    }

    displayRotationSteps = rotationMotor.currentPosition();

}

void controlTranslationMotor (float &translation_direction, bool translationmotorRunning) {
    translationMotor.setSpeed(400 * translation_direction);

    if (translationMotor.currentPosition() >= 0 && translationMotor.currentPosition() <= 1000) {
        translationMotor.runSpeed();
    } else if (translationMotor.currentPosition() <= 0 && translation_direction == 1) {
        translationMotor.runSpeed();
    } else if (translationMotor.currentPosition() >= 1000 && translation_direction == -1) {
        translationMotor.runSpeed();
    }

    displayTranslationSteps = translationMotor.currentPosition();
}

void receive_response(Ezo_board &ECsensor)  // Function to decode the response after a command was issued
{
  char sensordata_buffer[32];            // Buffer to hold the incoming data
  Ezo_board::errors error = ECsensor.receive_cmd(sensordata_buffer, 32); // Receive the response

//   Serial.print(Sensor.get_name());       // Print the name of the circuit getting the reading
//   Serial.print(": ");

  if (error == Ezo_board::SUCCESS) {
    // Serial.println(sensordata_buffer);   // Print the received data
    lastconductivityreading = atof(sensordata_buffer);
    displayconductivity = lastconductivityreading;
  } else {
    switch (error) {
      case Ezo_board::FAIL:
        Serial.println("Failed");        // The command has failed
        break;
      case Ezo_board::NOT_READY:
        Serial.println("Pending");       // The command has not yet been finished calculating
        break;
      case Ezo_board::NO_DATA:
        Serial.println("No Data");       // The sensor has no data to send
        break;
      case Ezo_board::NOT_READ_CMD:
        Serial.println("Not Read Command"); // The command was not a read command
        break;
      default:
        Serial.println("Unknown error"); // An unknown error occurred
        break;
    }
  }
}

void glidercontrol(void* pvParameters) {
  bool translationmotorRunning = false;
  bool rotationmotorRunning = false;
  bool glideDownReservoirFull = false;
  bool glideUpReservoirEmpty = false;
  bool dropweightReleased = false;
  bool dykcount = false;
  float currentPosition = 0;
  float currentDegree = 0;
  float pitchSP = 0;
  float rollSP = 0;
  unsigned long glideDownTime = 0;
  unsigned long glideUpTime = 0;
  int valePinState = digitalRead(VALVE_PIN);
  int pumpPinState = digitalRead(PUMP_PIN);

  float previousDepth = 0; // Variable to store the previous depth
  unsigned volatile long previousTime = 0; // Variable to store the previous time
  unsigned volatile long previousPrintTime = 0; // Variable to store the previous time the state was printed
  unsigned volatile long previousBatteryCheck = 0; // Variable to store the previous time the battery level was checked
  const unsigned long BAT_CHECK_INTERVAL = 60000 * 5; // Interval to check the battery level (in milliseconds) (5 minutes)
  const unsigned long ONE_HOUR = 3600000; // 1 hour in milliseconds, used to check if the glider has been going down/up for too long
  const unsigned long sampleInterval = 10000; //Interval to sample the depth (in milliseconds)
  const unsigned long statePrintInterval = 5000; //Interval to check if the glider is moving upwards instead of downwards (in milliseconds)
  unsigned int warningCount = 0; // Variable to keep track of the number of warnings
  bool warningFlag = false; // Flag to indicate if a warning has been issued
  bool previousTimeSet = false; // Flag to indicate if the previous time has been set when Down/Up states shift

  SensorData receivedData;

  while (true) {
    unsigned long currentMillis = millis(); //Holds the current time the program has been running in milliseconds


    //ToDo:
    // 1.
    // Add checks when transitioning between states to ensure that the motor/valve/pump are not continuing to run.
    // If they are, stop them before transitioning to the next state. 
    //
    // 2. 
    // Add checks to ensure that the glider is not moving in the wrong direction. So that when it is supposed to move up, it is not moving down.
    // 
    // 3. 
    // Add communication with AGT 
    //
    // 4. 
    // 
    

    switch (gliderState) { 

      case Glider_init:

            while (!startSetup) {
                delay(100);
                errorMessage = "Gliderstate: Init, waiting for setup to start";
                if (currentMillis - previousPrintTime >= statePrintInterval) {
                Serial.println("Gliderstate: Init, waiting for setup to start");
                previousPrintTime = currentMillis;
                }
            }

            errorMessage = "Gliderstate: Init, starting setup";

            bool initialized = false;
            while (!initialized) {
                myICM.begin(Wire, AD0_VAL);
                Serial.print(F("Initialization of the sensor returned: "));
                Serial.println(myICM.statusString());
                errorMessage = myICM.statusString();
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

            // Set the FIFO mode to snapshot mode
            success &= (myICM.setFIFOmode(true) == ICM_20948_Stat_Ok);

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
                errorMessage = "DMP configuration failed";
            }

            Psensor.init();
            Psensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
            if (Psensor.isInitialized()) {
                Serial.println("Pressure sensor connected.");
            } else {
                Serial.println("Pressure sensor not connected.");
                errorMessage = "Pressure sensor not connected.";
            }

            Tsensor.init();
            while (!Tsensor.init()) {
                Serial.println("TSYS01 device failed to initialize!");
                errorMessage = "Temperature sensor not connected.";
                delay(1000);
            }

            openLogSetup();

            errorMessage = "Glider initialized. Entering Idle state";

            gliderState = Idle;
          
          break;

      case Idle: //This case is for when the glider is not doing anything. Activated via button on SeaGUL webpage.

          if (currentMillis - previousPrintTime >= statePrintInterval) {
            Serial.println("Gliderstate: Idle");
            previousPrintTime = currentMillis;
          }
          //Check if the pump or valve pins are high. If they are, set them to low.
          if (pumpPinState == HIGH) {
            digitalWrite(PUMP_PIN, LOW);
          }
          if (valePinState == HIGH) {
            digitalWrite(VALVE_PIN, LOW);
          }

          // Check the battery voltage
          if (currentMillis - previousBatteryCheck >= BAT_CHECK_INTERVAL || batteryWiFiCheck) {
            getVBat();
            previousBatteryCheck = currentMillis;
            batteryWiFiCheck = false;

            if (Vbat < V_LOW) {
              errorMessage = "Low battery voltage!";
            }
          }

          //It is now possible to move the stepper motors manually through the SeaGUL webpage.
          if (movingForward) {
              float translation_direction = 1; // 1 = forward, -1 = backward

              if (!translationmotorRunning) { //Start the motor if it is not already running
                  digitalWrite(TRAN_SLEEP_PIN, HIGH);
                  translationmotorRunning = true;
              }

              controlTranslationMotor(translation_direction, translationmotorRunning); //Move the motor

            } else if (movingBackward) {
                float translation_direction = -1; // 1 = forward, -1 = backward

                if (!translationmotorRunning) { //Start the motor if it is not already running
                    digitalWrite(TRAN_SLEEP_PIN, HIGH);
                    translationmotorRunning = true;
                }

                controlTranslationMotor(translation_direction, translationmotorRunning); //Move the motor
            
            } else if (translationmotorRunning) { //Stop the motor if it is running
                digitalWrite(TRAN_SLEEP_PIN, LOW);
                translationmotorRunning = false;
            }

            if (rotatingLeft) {
                float rotation_direction = 1; // 1 = left, -1 = right

                if (!rotationmotorRunning) { //Start the motor if it is not already running
                    digitalWrite(ROT_SLEEP_PIN, HIGH);
                    rotationmotorRunning = true;
                }

                controlRotationMotor(rotation_direction, rotationmotorRunning); //Move the motor

            } else if (rotatingRight) {
                float rotation_direction = -1; // 1 = left, -1 = right

                if (!rotationmotorRunning) { //Start the motor if it is not already running
                    digitalWrite(ROT_SLEEP_PIN, HIGH);
                    rotationmotorRunning = true;
                }

                controlRotationMotor(rotation_direction, rotationmotorRunning); //Move the motor

            } else if (rotationmotorRunning) { //Stop the motor if it is running
                digitalWrite(ROT_SLEEP_PIN, LOW);
                rotationmotorRunning = false;
            }

          // It is also possible to control the pump and vent of the glider manually through the SeaGUL webpage.
          if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
            if (receivedData.potentiometer < 3000) {
              if (ventState) {
                digitalWrite(VALVE_PIN, HIGH);
              } else {
                digitalWrite(VALVE_PIN, LOW);
              }
            } else {
              errorMessage = "The reservoir is full.";
            }

            if (receivedData.potentiometer > 200) {
              if (pumpState) {
                digitalWrite(PUMP_PIN, HIGH);
              } else {
                digitalWrite(PUMP_PIN, LOW);
              }
            } else {
              errorMessage = "The reservoir is empty.";
            }
          }


          if (Serial.available() > 0) {
            char incomingChar = Serial.read();

            if (incomingChar == 'q') {
              Serial.println("Dropweight command received, moving to DropWeight state.");
              gliderState = DropWeight;
            }
          }

          break;

      case Diving: //This case activates when the Initiate Dive button is pressed on the SeaGUL webpage.

          if (currentMillis - previousPrintTime >= statePrintInterval) {
            Serial.println("Gliderstate: Diving");
            previousPrintTime = currentMillis;
          }

          //Check if the pump or valve pins are high. If they are, set them to low.
          if (pumpPinState == HIGH) {
            digitalWrite(PUMP_PIN, LOW);
          }
          if (valePinState == HIGH) {
            digitalWrite(VALVE_PIN, LOW);
          }
          Serial.println("Initiating dive...");

          n_dyk = 0; //Reset the number of dives

          delay(1000);

          Serial.println("Dive initiated. Moving to GlidingDown state.");
          
          gliderState = GlidingDown;
          stateStartMillis = currentMillis; //Store the time the GlidingDown state started
          break;

      case Calibrating:
          //Set the displacement of rotation and translation to 0. 
          //This should be done when the pitch and roll are indicated as 0 on the SeaGul webpage,
          //before being sent on a mission. 
          Serial.println("Calibrating...");

          currentPosition = 0;
          currentDegree = 0;
          //Future addition: Calibration of compass. 

          delay(2000);

          Serial.println("Calibration done. Returning to Idle state");
          gliderState = Idle;

          break;

      case GlidingDown:

          if (currentMillis - previousPrintTime >= statePrintInterval) {
            Serial.println("Gliderstate: GlidingDown");
            previousPrintTime = currentMillis;
          }

          //Reset flag, warning count and timer
          glideUpReservoirEmpty = false;
          warningCount = 0;

          pitchSP = -20; // Setpoint for pitch
          rollSP = 0; // Setpoint for roll

          //Future addition: Turning motion

          //First fill the reservoir with oil.
          if (!glideDownReservoirFull) {
              Serial.println("Opening valve");
              digitalWrite(VALVE_PIN, HIGH);

              if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
                  if (receivedData.potentiometer >= 3500) { //When the reservoir is full we close the valve. Value is not determined yet.
                      Serial.println("Reservoir full, closing valve.");
                      digitalWrite(VALVE_PIN, LOW);
                      glideDownReservoirFull = true; 
                  }
              }
          }

          //After the reservoir is full we can regulate the pitch and roll of the glider.
          //We can't do these simultaneously because the electrical system will be overloaded. 
          if (glideDownReservoirFull) {

              if (!previousTimeSet) { // Set the previous time to the current time after the reservoir is full.
                  previousTime = currentMillis;
                  previousTimeSet = true;
              }

              if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
                //ToDo: If the error is greater than 5 degrees, let the motor adjust the pitch for e.g. 10 seconds. 
                moveTranslationMotor(receivedData, currentPosition, pitchSP, translationmotorRunning);
                moveRotationMotor(receivedData, currentDegree, rollSP, rotationmotorRunning);
              }

              if (Serial.available() > 0) {
                char incomingChar = Serial.read();
                if (incomingChar == 's') {
                  Serial.println("Depth reached, moving to GlidingUp state.");

                  //Reset boolean from the dive down.
                  glideDownReservoirFull = false;

                  gliderState = GlidingUp;
                  stateStartMillis = currentMillis; //Store the time the GlidingUp state started
                  previousTimeSet = false;
                }

                if (incomingChar == 'q') {
                  Serial.println("Dropweight command received, moving to DropWeight state.");
                  gliderState = DropWeight;
                }

              }

              //UNCOMMENT THIS IF YOU WANT TO USE THE WARNING SYSTEM! NOT TESTED YET!
              // if (currentMillis - previousTime >= sampleInterval) {
              //   warningFlag = false;
              //   float depthChangeRate = (receivedData.depth - previousDepth) / ((glideDownTime - previousTime)/1000);
              //   previousDepth = receivedData.depth;
              //   previousTime = currentMillis;

              //   if (depthChangeRate > 0) {
                  
              //     Serial.println("WARNING: Glider is moving upwards instead of down!");

              //     if (!warningFlag) {
              //       warningCount += 1;
              //       warningFlag = true;
              //     }

              //     if (warningCount >= 3) {
              //       Serial.println("WARNING: Glider is moving upwards instead of down! Releasing weight by moving to DropWeight state.");
              //       gliderState = DropWeight;
              //     }                            
              //   }
              // }
              //UNCOMMMENT THIS IF YOU WANT TO USE THE WARNING SYSTEM! NOT TESTED YET!
              
              if (currentMillis - stateStartMillis >= ONE_HOUR) {
                  Serial.println("To much time has passed, releasing weight by moving to DropWeight state.");
                  gliderState = DropWeight;
              }
          }

          //Check if too much time has passed for the glider to reach its target depth.
          if (currentMillis - stateStartMillis >= ONE_HOUR) {
              Serial.println("To much time has passed, releasing weight by moving to DropWeight state.");
              gliderState = DropWeight;
          }

          delay(100);

          break;
      
      case GlidingUp:

          if (currentMillis - previousPrintTime >= statePrintInterval) {
            Serial.println("Gliderstate: GlidingUp");
            previousPrintTime = currentMillis;
          }

          warningCount = 0;
          previousTime = 0;

          pitchSP = 20; // Setpoint for pitch
          rollSP = 0; // Setpoint for roll

          //First start the pump to empty the reservoir.
          if (!glideUpReservoirEmpty) {
              Serial.println("Starting pump");
              digitalWrite(PUMP_PIN, HIGH);

              if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
                if (receivedData.potentiometer <= 1000) { //When the reservoir is empty we stop the pump. Value is not determined yet.
                    Serial.println("Reservoir empty, stopping pump.");
                    digitalWrite(PUMP_PIN, LOW);
                    glideUpReservoirEmpty = true;
                }
              }
          }

          //After the reservoir is empty we can regulate the pitch and roll of the glider.
          if (glideUpReservoirEmpty) {

              if (!previousTimeSet) { // Set the previous time to the current time after the reservoir is empty.
                  previousTime = currentMillis;
                  previousTimeSet = true;
              }


              if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
                moveTranslationMotor(receivedData, currentPosition, pitchSP, translationmotorRunning);
                moveRotationMotor(receivedData, currentDegree, rollSP, rotationmotorRunning);
              }

              if (Serial.available() > 0) {
                char incomingChar = Serial.read();
                if (incomingChar == 'd') {
                  Serial.println("Surface reached, moving to surface or dive state. Dive number: " + String(n_dyk));

                  //Since a dive is complete we add 1 to the number of dives.
                  if (!dykcount) {
                      n_dyk += 1;
                      totalNumberofDives += 1;
                      dykcount = true; 
                  }

                  //If the number of dives is less than 3 we will continue diving by returning to the GlidingDown state.
                  if (n_dyk < 3) {
                      gliderState = GlidingDown;
                      dykcount = false;
                      stateStartMillis = currentMillis; //Store the time the GlidingDown state started
                      previousTimeSet = false;
                  }

                  //If the number of dives is 3 or more we will move to the Surface state and reset the reservoir flag. 
                  if (n_dyk >= 3) {
                      gliderState = Surface;
                      glideUpReservoirEmpty = false;
                      dykcount = false;
                  }

                }

                if (incomingChar == 'q') {
                  Serial.println("Dropweight command received, moving to DropWeight state.");
                  gliderState = DropWeight;
                }

              }

              //UNCOMENT THIS IF YOU WANT TO USE THE WARNING SYSTEM! NOT TESTED YET!
              // if (currentMillis - previousTime >= sampleInterval) {
              //   warningFlag = false;
              //   float depthChangeRate = (receivedData.depth - previousDepth) / ((millis() - previousTime)/1000);
              //   previousDepth = receivedData.depth;
              //   previousTime = currentMillis;

              //   if (depthChangeRate < 0) {

              //     Serial.println("WARNING: Glider is moving downwards instead of up!");

              //     if(!warningFlag) {
              //       warningCount += 1;
              //       warningFlag = true;
              //     }
                  
              //     if (warningCount >= 3) {
              //       Serial.println("WARNING: Glider is moving downwards instead of up! Releasing weight by moving to DropWeight state.");
              //       gliderState = DropWeight;
              //     }
              //   }
              // }
              //UNCOCMENT THIS IF YOU WANT TO USE THE WARNING SYSTEM! NOT TESTED YET!

              //Check if too much time has passed for the glider to rise to the surface. 
              if (currentMillis - stateStartMillis >= ONE_HOUR) {
                  Serial.println("To much time has passed, releasing weight by moving to DropWeight state.");
                  gliderState = DropWeight;
              }
          }

          break;

      case Surface:
    
          if (currentMillis - previousPrintTime >= statePrintInterval) {
            Serial.println("Gliderstate: Surface");
            previousPrintTime = currentMillis;
          }

          pitchSP = -20; // Setpoint for pitch, we want the antenna to be above the water.
          rollSP = 0; // Setpoint for roll

          //Check if the dropweight has been released. If not, buissness as usual.
          // if (dropweightReleased) {
          //     //If the dropweight has been released, we want to transmit that information to land
          //     //so that the glider can be picked up.
          //     Serial.println("Dropweight released, pick me up!");
          //     digitalWrite(ACTIVATION_PIN, HIGH);

          //     while (digitalRead(POKE_PIN) == LOW) { // Might need a timeout counter here if the AGT is dead for some reason.
          //       delay(100);
          //     }

          //     if (digitalRead(POKE_PIN) == HIGH) {
          //       Serial.println("AGT is ready to receive message");
          //       delay(1000);
          //       mySerial.print("d]");
          //       Serial.println("Message sent: d");
          //       delay(100);
          //       digitalWrite(ACTIVATION_PIN, LOW);
          //     }

          // } else {
          //   Serial.println("Dropweight not released, send position via AGT and wait for command to dive again.");

          //   digitalWrite(ACTIVATION_PIN, HIGH);

          //   while (digitalRead(POKE_PIN) == LOW) { // Might need a timeout counter here if the AGT is dead for some reason.
          //     delay(100);
          //   }

          //   if (digitalRead(POKE_PIN) == HIGH) {
          //     Serial.println("AGT is ready to receive message");
          //     delay(1000);
          //     mySerial.print("g]");
          //     Serial.println("Message sent: g");
          //     delay(100);
          //     digitalWrite(ACTIVATION_PIN, LOW);
          //   }
          // }

          //Battery check. If the battery is low, we want to transmit that information to land.

          //Send GPS position to land along with the state of the glider.


          //Await command to dive again. 

          if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
            moveRotationMotor(receivedData, currentDegree, rollSP, rotationmotorRunning);
            moveTranslationMotor(receivedData, currentPosition, pitchSP, translationmotorRunning);
          }

          //When command is given enter the Diving state and reset the number of dives
          if (Serial.available() > 0) {
              char incomingChar = Serial.read();
              if (incomingChar == 'u') {
                  Serial.println("Dive command received, moving to Diving state.");

                  //Reset the number of dives
                  n_dyk = 0;

                  gliderState = Diving;
                  stateStartMillis = currentMillis; //Store the time the Diving state started
              }
          }

          break;

      case DropWeight:

          Serial.println("Gliderstate: DropWeight"); 

          Serial.println("Releasing weight...");

          //Check if the pump or valve pins are high. If they are, set them to low.
          if (pumpPinState == HIGH) {
            digitalWrite(PUMP_PIN, LOW);
          }
          if (valePinState == HIGH) {
            digitalWrite(VALVE_PIN, LOW);
          }
          
          digitalWrite(DROPWEIGHT_PIN, HIGH);

          dropweightReleased = true;

          delay(1000);

          digitalWrite(DROPWEIGHT_PIN, LOW);

          //After the release of the dropweight, enter Surface state.
          gliderState = Surface;
          break;
    }
  }
}


void datagathering(void* pvParameters) {
  while (true) {
      SensorData data;
      icm_20948_DMP_data_t IMUdata;
      myICM.readDMPdataFromFIFO(&IMUdata);
      if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {

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

          // Calculate display pitch and roll, the values that are displayed on the SeaGUL webpage.
          displaypitch = data.pitch;
          displayroll = data.roll;
          displayyaw = data.yaw;

          } else {
              Serial.println("Quat6 data not ready");
          }

      } else {
          Serial.println("Failed to read DMP data");
      }

      delay(35);
      // Read pressure 
      Psensor.read();
      data.depth = Psensor.depth();
      data.pressure = Psensor.pressure();
      displaypressure = data.pressure;

      delay(35);
      // Read temperature
      Tsensor.read();
      data.temperature = Tsensor.temperature();
      displaytemperature = data.temperature;

      // Read conductivity
      if (!readingConductivity) {
        EC.send_cmd("R"); //Request a reading
        
        next_ECread_time = millis() + 1000; // Set when the response will arrive
        readingConductivity = true; // Switch to the receiving phase
      }
  
    if (readingConductivity && millis() >= next_ECread_time) {
        receive_response(EC); // Get the response from the EC circuit
        readingConductivity = false; // Switch back to asking for readings
    }

      // Read potentiometer
      data.potentiometer = analogRead(SOFT_POT_PIN);
      displaypotentiometer = data.potentiometer;

      // Send data to the second core via a queue
      if (xQueueSend(sensorDataQueue, &data, (TickType_t)10) != pdPASS) {
          // Serial.println("Failed to send data to queue");
      }

      // Log the data to the SD card
      String logData = "Pitch: " + String(data.pitch) + " degrees, " +
                        "Roll: " + String(data.roll) + " degrees, " +
                        "Yaw: " + String(data.yaw) + " degrees, " +
                        "Depth: " + String(data.depth) + " m, " +
                        "Pressure: " + String(data.pressure) + " mbar, " +
                        "Conductivity" + String(lastconductivityreading) + " mS/cm, " +
                        "Temperature: " + String(data.temperature) + " Celsius";
                      //  "Batterycurrent: " + String(current) + " A";
      // writeSD(logData);
      Serial.println(logData);

      // delay(100); // Adding a delay to reduce bus congestion and improve stability
    }

  // vTaskDelay(pdMS_TO_TICKS(1000)); // Adding a delay to reduce bus congestion and improve stability
}


