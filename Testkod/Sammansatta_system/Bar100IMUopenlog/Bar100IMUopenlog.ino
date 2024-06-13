
#include <Arduino.h>
#include <ICM_20948.h>
#include <KellerLD.h>
#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"

OpenLog myLog;

String fileName = "SD-test240612.txt";

KellerLD Psensor;

ICM_20948_I2C myICM;

#define SDA_PIN 13
#define SCL_PIN 14

#define STATUS_SD_INIT_GOOD 0
#define STATUS_LAST_COMMAND_SUCCESS 1
#define STATUS_LAST_COMMAND_KNOWN 2
#define STATUS_FILE_OPEN 3
#define STATUS_IN_ROOT_DIRECTORY 4

#define AD0_VAL 1


void openLogSetup();

void PsensorSetup();

void OrientationSetup();

String Measurement(int(value)) {

    if (value == 1) {
        Serial.print(" Depth: ");
        Serial.print(Psensor.depth());
        Serial.print(" m"); 

        Serial.print(" Pressure: ");
        Serial.print(Psensor.pressure());
        Serial.print(" mbar"); 

        return "Depth: "+String(Psensor.depth())+" m, "+"Pressure: "+String(Psensor.pressure())+" mbar";
    }

    if (value == 2) {

        if (myICM.dataReady()){

            myICM.getAGMT();

            float pitch = atan2(myICM.accY(), sqrt(myICM.accX() * myICM.accX() + myICM.accZ() * myICM.accZ()));

            Serial.print(" Pitch: ");
            Serial.print(pitch);
            Serial.print(" ");
            
            return " Pitch: "+String(pitch)+" degrees";
        }
    }
}

void writeSD(String(measurement)) {
  myLog.append(fileName); //Bestämmer att vi ska sriva i denna filen 
  myLog.println(measurement); //Skriver till filen
  myLog.syncFile();
}

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    openLogSetup();
    PsensorSetup();
    OrientationSetup();

}


void loop() {
    writeSD(Measurement(1));
    writeSD(Measurement(2));

    delay(1000);
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

void PsensorSetup() {

    Psensor.init();
    Psensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    if (Psensor.isInitialized()) {
        Serial.println("Pressure sensor connected.");
    } else {
        Serial.println("Pressure sensor not connected.");
    }

}

void OrientationSetup() {
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
}