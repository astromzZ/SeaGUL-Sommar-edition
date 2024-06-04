#include <Wire.h>
//#include "SoftwareSerial.h"
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Stepper.h>
#include <KellerLD.h>
#include "MS5837.h"
#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include <ICM_20948.h>
#include <Adafruit_INA228.h>
#include <TSYS01.h>
#include "ICM_20948.h"

#define STATUS_SD_INIT_GOOD 0
#define STATUS_LAST_COMMAND_SUCCESS 1
#define STATUS_LAST_COMMAND_KNOWN 2
#define STATUS_FILE_OPEN 3
#define STATUS_IN_ROOT_DIRECTORY 4


//Pins
const int SDA_PIN = 17;
const int SCL_PIN = 16;
const int dropweight_pin = ...;
const int dirPin_tr = 5;
const int stepPin_tr = 6;
const int sleepPin_tr = 7;
const int leakpin = ...;
const int potPin = ;
const int vent = ;
const int pumpPin = ;
const int stepsPerRevolution = 200;


//Värde för orienteringssensorn
#define AD0_VAL 1

// Filter coefficients
const float alpha = 0.98;
const float dt = 0.01; // Time interval in seconds

// Potentiometer
const float potlength = 40;

// Riktvärden för dykningen
const float real_pitch = 20;
const float pressure_max = ...; //högsta tillåtna trycket
const float pressure_min = ...; //yttrycket
const float maxtime = ...; // maximala tiden den får vara under vatten innan drop weight släpps
String fileName = "gliderSDTest6.txt";

ICM_20948_I2C myICM;
OpenLog myLog; //Create instance

// Initierar variabler för mätvärden mm.
float pitch, yaw, roll;
unsigned long lastPressureTime = 0;
bool maxtime_exceeded = false; //tid under vattnet för länge
bool empty = true; // är yttre blåsan (nästan) tom?
bool full = false;
bool pumping;
bool dropweight = false;

//Initierar övriga värden
const int len_pres_arr = 100;

TSYS01 Tsensor;
KellerLD Psensor;

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Serial.begin(115200);
  
  while (!Serial) {
    delay(10);
  }

  //Initialize SD-card reader
  openLogSetup();
  //............................
  
  //Initialize current sensor
  INA228_setup();
  //............................



  //Initialize Orientation Sensor
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again...");
      delay(500);
    }

    else {
      initialized = true;
    }

    pinMode(potpin, INPUT);

    pinMode(dropweight_pin, OUTPUT);
  }

  //..............................................................

  //Initialize Pressure Sensor

  Psensor.init();
  Psensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  float pressure_array[len_pres_arr];
  int n = 0;

  //..............................................................

  //Setup for pump and ventil
  pinMode(vent, OUTPUT); //Ventil
  digitalWrite(vent, LOW); //Closed
  pinMode(pumpPin, OUTPUT); //Pump
  digitalWrite(pumpPin, LOW); //Closed

  //..............................................................

  //Initialize Temperatur sensor

  Tsensor.init();
}


void loop() {

  //Orientation sensor:

  if (myICM.dataReady()) {
    myICM.getAGMT();
    updateOrientation();
    pitch_reg();
    //rollreglering
    //yawreglering
  }

  else {
    Serial.println("Waiting for data");
    delay(500);
  }
  // .........................................

  //Pressure sensor

  Psensor.read();

  float pressure = Psensor.pressure() //tryck från trycksensor
  if ((pressure < pressure_max) && (pressure > pressure_min)) {
    if (!maxtime_exceeded) {
      if (millis() - lastPressureTime >= maxtime) {
        dropweight = true;
      }
    }
  }

  else {
    lastPressureTime = millis();
    maxtime_exceeded = false;
  }

  pressure_array[1] = pressure;
  if (n > 0) {
    for (int i = 0; i < len_pres_arr - 2; i++) {
      pressure_array[i] = pressure_array[i+1];
    }
    pressure_array[len_pres_arr-1] = pressure;
  }
  n += 1;

  // .........................................


  //Potentiometer, pump och ventil

  int potValue = analogRead(potPin);
  int percent = map(potValue, 0, 1023, 0, 100); //percent of max resistance from potentiometer
  float dep_to_pres = 997*9.82;

  empty = (percent == 100); // är yttre blåsan tom? (ändra till == 0 möjligtvis), kan sätta lite marginal på detta
  if (pressure > pressure_min + 1*dep_to_pres) {
    if (!empty) {
      digitalWrite(ventilPin, HIGH);
    }
    else {
      digitalWrite(ventilPin, LOW);
    }
  }

  full = (percent == 0); // (ändra till == 100 möjligtvis) kan sätta lite marginal på detta
  if ((pressure > pressure_max - 10*dep_to_pres) || pumping) {
    if (!full) {
      digitalWrite(pumpPin, HIGH);
      pumping = true;
    }
    else {
      digitalWrite(pumpPin, LOW);
      pumping = false;
    }
  }
  

  //SOS-sensor
  leak = digitalRead(leakpin);
  if (leak == 1) {
    dropweight = true;
  }

  // .........................................

  //SD-kort
  writeSD(INA228_measure(1));
  writeSD(INA228_measure(2));

  // .........................................

  //Temperaturesensor

  Tsensor.read();

  writeSD(Tsensor.temperature()); //Write temperature value to SD-card


  // Släpp dropweight om något allvarligt problem uppstått 
  if (dropweight) {
      digitalWrite(dropweight_pin, HIGH);
  }

  // .........................................
  
}


void updateOrientation() {
  // Pitch och roll från accelerometerdata
  float accRoll = atan2(myICM.accY(), sqrt(myICM.accX()*myICM.accX() + myICM.accZ()*myICM.accZ())) * RAD_TO_DEG;
  float accPitch = atan2(-myICM.accX(), myICM.accZ()) * RAD_TO_DEG;
  
  // Yaw från magnetfältsdata
  float magYaw = atan2(myICM.magY(), myICM.magX()) * RAD_TO_DEG;
  if (magYaw < 0) {
    magYaw += 360.0;
  }

  //Kalmanfilter för mer stabil dataavläsning
  pitch = alpha * (pitch + myICM.gyrX() * dt) + (1 - alpha) * accPitch;
  roll = alpha * (roll - myICM.gyrY() * dt) + (1 - alpha) * accRoll;
  yaw = alpha * (yaw - myICM.gyrZ() * dt) + (1 - alpha) * magYaw;
  delay(dt*1000);
}

void pitch_reg() {
  bool down = pressure_array[0] < pressure_array[-1]; // på väg ner?
  bool up = pressure_array[0] > pressure_array[-1]; // på väg upp?

  float eps = 2; // tillåtet fel för vinkeln
  if (down) {
    if (pitch > - real_pitch + eps) {
      digitalWrite(sleepPin_tr, HIGH);
      delay(50);
      digitalWrite(dirPin_tr, HIGH); // eller LOW
      digitalWrite(stepPin_tr, HIGH);
      delay(50);
      digitalWrite(stepPin_tr, LOW);
      delay(50);
    }
    else if (pitch < - real_pitch - eps) {
      digitalWrite(sleepPin_tr, HIGH);
      delay(50);
      digitalWrite(dirPin_tr, LOW); // eller HIGH
      digitalWrite(stepPin_tr, HIGH);
      delay(50);
      digitalWrite(stepPin_tr, LOW);
      delay(50);
    }
    else {
      digitalWrite(sleepPin_tr, LOW);
    }
  }

  if (up) {
    if (pitch > real_pitch + eps) {
      digitalWrite(sleepPin_tr, HIGH);
      delay(50);
      digitalWrite(dirPin_tr, HIGH); // eller LOW
      digitalWrite(stepPin_tr, HIGH);
      delay(50);
      digitalWrite(stepPin_tr, LOW);
      delay(50);
    }
    else if (pitch < real_pitch - eps) {
      digitalWrite(sleepPin_tr, HIGH);
      delay(50);
      digitalWrite(dirPin_tr, LOW); // eller HIGH
      digitalWrite(stepPin_tr, HIGH);
      delay(50);
      digitalWrite(stepPin_tr, LOW);
      delay(50);
    }
    else {
      digitalWrite(sleepPin_tr, LOW);
    }
  }
}


//SD-card setup
void openLogSetup() {
  myLog.begin();

  Serial.begin(115200);
  Serial.println("OpenLog Status Test");

  byte status = myLog.getStatus();

  Serial.print("Status byte: 0x");
  if(status < 0x10) Serial.print("0"); {
    Serial.println(status, HEX);
  }

  if (status == 0xFF) {
    Serial.println("OpenLog failed to respond. Freezing.");
    while(1); //Freeze!
  }

  if (status & 1<<STATUS_SD_INIT_GOOD) {    
    Serial.println("SD card is good");
  }  
  else {
    Serial.println("SD init failure. Is the SD card present? Formatted?");
  }
  if (status & 1<<STATUS_IN_ROOT_DIRECTORY) {
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

void INA228_setup() {
  Serial.println("Adafruit INA228 Test");

  if (!ina228.begin()) {
    Serial.println("Couldn't find INA228 chip");
    while (1) {
    ;
    }
  }

  Serial.println("Found INA228 chip");
  // we need to set the resistance (default 0.1 ohm) and our max expected
  // current (no greater than 3.2A)
  ina228.setShunt(0.1, 1.0);

  ina228.setAveragingCount(INA228_COUNT_16);
  uint16_t counts[] = {1, 4, 16, 64, 128, 256, 512, 1024};
  Serial.print("Averaging counts: ");
  Serial.println(counts[ina228.getAveragingCount()]);

  // set the time over which to measure the current and bus voltage
  ina228.setVoltageConversionTime(INA228_TIME_150_us);
  Serial.print("Voltage conversion time: ");
  switch (ina228.getVoltageConversionTime()) {
    case INA228_TIME_50_us:
      Serial.print("50");
      break;
    case INA228_TIME_84_us:
      Serial.print("84");
      break;
    case INA228_TIME_150_us:
      Serial.print("150");
      break;
    case INA228_TIME_280_us:
      Serial.print("280");
      break;
    case INA228_TIME_540_us:
      Serial.print("540");
      break;
    case INA228_TIME_1052_us:
      Serial.print("1052");
      break;
    case INA228_TIME_2074_us:
      Serial.print("2074");
      break;
    case INA228_TIME_4120_us:
      Serial.print("4120");
      break;
  }
  Serial.println(" uS");

  ina228.setCurrentConversionTime(INA228_TIME_280_us);
  Serial.print("Current conversion time: ");
  switch (ina228.getCurrentConversionTime()) {
    case INA228_TIME_50_us:
      Serial.print("50");
      break;
    case INA228_TIME_84_us:
      Serial.print("84");
      break;
    case INA228_TIME_150_us:
      Serial.print("150");
      break;
    case INA228_TIME_280_us:
      Serial.print("280");
      break;
    case INA228_TIME_540_us:
      Serial.print("540");
      break;
    case INA228_TIME_1052_us:
      Serial.print("1052");
      break;
    case INA228_TIME_2074_us:
      Serial.print("2074");
      break;
    case INA228_TIME_4120_us:
      Serial.print("4120");
      break;
  }
  Serial.println(" uS");
}

String INA228_measure(int(value)) {
  // Value väljer vad som ska mätas
  if (value == 1) {
    Serial.print("Current: ");
    Serial.print(ina228.readCurrent());
    Serial.println(" mA");
    return "Current: "+String(ina228.readCurrent())+" mA";
  }
  if (value == 2) {
    Serial.print("Bus Voltage: ");
    Serial.print(ina228.readBusVoltage());
    Serial.println(" mV");
    return "Bus Voltage: "+String(ina228.readBusVoltage())+" mV";
  }
  if (value == 3) {
    Serial.print("Shunt Voltage: ");
    Serial.print(ina228.readShuntVoltage());
    Serial.println(" mV");
    return "Shunt Voltage: "+String(ina228.readShuntVoltage())+" mV";
  }
  if (value == 3) {
    Serial.print("Power: ");
    Serial.print(ina228.readPower());
    Serial.println(" mW");
    return "Power: "+String(ina228.readPower())+" mW";
  }
  if (value == 4) {
    Serial.print("Energy: ");
    Serial.print(ina228.readEnergy());
    Serial.println(" J");
    return "Energy: "+String(ina228.readEnergy())+" J";
  }
  if (value == 5) {
    Serial.print("Temperature: ");
    Serial.print(ina228.readDieTemp());
    Serial.println(" *C");
    return "Temperature: "+String(ina228.readDieTemp())+" *C";
  }

  Serial.println();
  delay(1000);
}

void writeSD(String(measurement)) {
  myLog.append(fileName); //Bestämmer att vi ska sriva i denna filen 
  myLog.println(measurement); //Skriver till filen
  myLog.syncFile();
}