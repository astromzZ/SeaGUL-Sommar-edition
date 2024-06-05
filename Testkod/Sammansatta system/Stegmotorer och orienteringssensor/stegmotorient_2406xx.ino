#include <Wire.h>
#include "ICM_20948.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>


#define SDA_PIN 17
#define SCL_PIN 16

#define AD0_VAL 1


// Define stepper motor connections and steps per revolution:
#define dirPin 5
#define stepPin 6
#define sleepPin 7
#define stepsPerRevolution 200

// ICM_20948_AGMT_t agmt;

ICM_20948_I2C myICM;

const float real_pitch = -20 * PI / 180;
// const float real_roll = 0;
// const float real_yaw = ...;


void Orientation(void *pvParameters) {
    while(true) {

    }

}



void Vehiclecontrol(void *pvParameters) {


}


void setup() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(sleepPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.begin(115200);

    xTaskCreatePinnedToCore(Orientation, "Orienteringsdata", 10000, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(Vehiclecontrol, "Reglering av tyngdpaket", 10000, NULL, 1, NULL, 1)

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
    }  
}
