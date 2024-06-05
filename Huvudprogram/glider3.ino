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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

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
const int ventilPin = ;
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

void datacollection(void *pvParameters) {

    while (true) {
        //Samla in data från sensorer

        //Spara data på SD-kort

        //Skicka tryck- och orienteringsdata till kärna 1

    }


}



void vehiclecontrol(void *pvParameters) {
    
    while (true) {
        //Ta emot data från kärna 0

        //Reglera utefter inhämtade värden

    }

}


void Setup() {
    Serial.begin(115200);


    xTaskCreatePinnedToCore(datacollection, "Sensordata", 10000, NULL, 1, NULL, 0); //Sensordata samlas in på kärna 0

    xTaskCreatePinnedToCore(vehiclecontrol, "Reglering och styrning", 10000, NULL, 1, NULL, 1) //Reglering och styrning hanteras av kärna 1


}