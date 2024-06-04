#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
OpenLog myLog; //Create instance

#include <Adafruit_INA228.h>
Adafruit_INA228 ina228 = Adafruit_INA228();

String fileName = "gliderSDTest6.txt";

#define STATUS_SD_INIT_GOOD 0
#define STATUS_LAST_COMMAND_SUCCESS 1
#define STATUS_LAST_COMMAND_KNOWN 2
#define STATUS_FILE_OPEN 3
#define STATUS_IN_ROOT_DIRECTORY 4

#define SDA_PIN 2
#define SCL_PIN 1

void openLogSetup() {
  myLog.begin();

  Serial.begin(115200); //9600bps is used for debug statements
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

void INA228_setup() {
  Serial.println("Adafruit INA228 Test");

  if (!ina228.begin()) {
    Serial.println("Couldn't find INA228 chip");
    while (1)
      ;
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
  if (value == 4) {
    Serial.print("Power: ");
    Serial.print(ina228.readPower());
    Serial.println(" mW");
    return "Power: "+String(ina228.readPower())+" mW";
  }
  if (value == 5) {
    Serial.print("Energy: ");
    Serial.print(ina228.readEnergy());
    Serial.println(" J");
    return "Energy: "+String(ina228.readEnergy())+" J";
  }
  if (value == 6) {
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

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) {
    delay(10);
  }
  openLogSetup();
  INA228_setup();
}

void loop() {
  writeSD(INA228_measure(1));
  writeSD(INA228_measure(2));
  delay(1000);
}