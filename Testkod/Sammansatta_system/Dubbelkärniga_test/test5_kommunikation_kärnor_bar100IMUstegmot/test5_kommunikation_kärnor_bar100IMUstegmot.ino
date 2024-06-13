#include "ICM_20948.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <KellerLD.h>
#include <BasicStepperDriver.h>
#include <DRV8825.h>
#include <MultiDriver.h>
#include <SyncDriver.h>
#include <Arduino.h>

// #include <MS5837.h>


#define SDA_PIN 13
#define SCL_PIN 14

#define AD0_VAL 1

#define DIR_PIN 9
#define STEP_PIN 46
#define SLEEP_PIN 3

#define PITCHTHRESHOLD 20

#define stepsPerRevolution 200

#define RPM 186

#include "DRV8825.h"
#define MODE0 10
#define MODE1 11
#define MODE2 12
DRV8825 stepper(stepsPerRevolution, DIR_PIN, STEP_PIN, SLEEP_PIN, MODE0, MODE1, MODE2);

ICM_20948_I2C myICM;

const float real_pitch = -20 * PI / 180;
// const float real_roll = 0;
// const float real_yaw = ...;

KellerLD Psensor;

struct SensorData{
    float sensor1Data;
    float sensor2Data;
    float sensor3Data;
    float sensor4Data;
};


QueueHandle_t sensorDataQueue;

SemaphoreHandle_t printMutex;

// SemaphoreHandle_t i2cMutex;

void steppermotor(void* pvParameters);

void orientation(void* pvParameters0);


void setup() {
    // Declare pins as output:
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);

    stepper.begin(RPM);

    stepper.enable();
    //put your setup code here, to run once:
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.begin(115200);

    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorDataQueue == NULL) {
        Serial.println("Failed to create queue");
        return;
    }

    printMutex = xSemaphoreCreateMutex();
    if (printMutex == NULL) {
        Serial.println("Failed to create print mutex");
        return;
    }

    // i2cMutex = xSemaphoreCreateMutex();
    // if (i2cMutex == NULL) {
    //     Serial.print("Failed to create i2c mutex");
    //     return;
    // }

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

    // while (!Psensor.init()) {
    //     Serial.println("Init failed!");
    //     Serial.println("Are SDA/SCL connected correctly?");
    //     Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    //     Serial.println("\n\n\n");
    //     delay(5000);
    // }

    Psensor.init();
    Psensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    if (Psensor.isInitialized()) {
        Serial.println("Pressure sensor connected.");
    } else {
        Serial.println("Pressure sensor not connected.");
    }
    

    xTaskCreatePinnedToCore (
        orientation,     // Function to implement the task
        "orientation measurement",   // Name of the task
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
    // while(true) {
    //     Serial.print ("loop() running in core ");
    //     Serial.println (xPortGetCoreID());
    // }
}

void steppermotor(void* pvParameters) {
    SensorData receivedData;
    bool motorRunning = false;

    while (true) {
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
            // if (xSemaphoreTake(printMutex, portMAX_DELAY)){
            //     Serial.print("Pitch:");
            //     Serial.print(receivedData.sensor1Data * 180 / PI);
            //     Serial.print("°\t");

            //     Serial.print("Roll:");
            //     Serial.print(receivedData.sensor2Data * 180 / PI);
            //     Serial.print("°\t");

            //     Serial.print("Pressure: ");
            //     Serial.print(receivedData.sensor3Data);
            //     Serial.print(" mbar\t");

            //     Serial.print("Depth: ");
            //     Serial.print(receivedData.sensor4Data);
            //     Serial.print(" m\t");                

            //     Serial.print("Core: ");
            //     Serial.print(xPortGetCoreID());

            //     Serial.println("");

            //     xSemaphoreGive(printMutex);
            // }
            // else {
            //     Serial.print("Printing: Mutex not aquired");
            // }

            if (receivedData.sensor1Data * 180 / PI >= PITCHTHRESHOLD) {
                if (!motorRunning) {
                    digitalWrite(DIR_PIN, HIGH);
                    digitalWrite(SLEEP_PIN, HIGH);
                    motorRunning = true;
                }
            } else {
                if (motorRunning) {
                    digitalWrite(SLEEP_PIN, LOW);
                    motorRunning = false;
                }
            }
        }

        if (motorRunning) {
            stepper.setMicrostep(1);
            stepper.move(-10);
            // delayMicroseconds(1000);
            }

        // vTaskDelay(1);
    }
}


void orientation(void* pvParameters) {
    while(true) {
        if (myICM.dataReady()) {
            SensorData data;
            myICM.getAGMT();         // The values are only updated when you call 'getAGMT'

            // Calculate pitch and roll
            data.sensor1Data = (atan2(myICM.accY(), sqrt(myICM.accX() * myICM.accX() + myICM.accZ() * myICM.accZ())));
            data.sensor2Data = -atan2(-myICM.accX(), myICM.accZ());

            //Pressure and depth
            Psensor.read();
            data.sensor3Data = Psensor.pressure();
            data.sensor4Data = Psensor.depth();

            xQueueSend(sensorDataQueue, &data, portMAX_DELAY);
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            
            // Calculate yaw
            // float magX = myICM.magX() * cos(pitch) + myICM.magY() * sin(roll) * sin(pitch) + myICM.magZ() * cos(roll) * sin(pitch);
            // float magY = myICM.magY() * cos(roll) - myICM.magZ() * sin(roll);
            // float yaw = atan2(-magY, magX);
            // -160 * PI / 180;

            if (xSemaphoreTake(printMutex, portMAX_DELAY)){
                Serial.print("Pitch: ");
                Serial.print(data.sensor1Data * 180 / PI); // Convert to degrees
                Serial.print("°\t");

                Serial.print("Roll: ");
                Serial.print(data.sensor2Data * 180 / PI); // Convert to degrees
                Serial.print("°\t");

                Serial.print("Pressure: ");
                Serial.print(data.sensor3Data);
                Serial.print(" mbar\t");

                Serial.print("Depth: ");
                Serial.print(data.sensor4Data);
                Serial.print(" m\t");

                Serial.print("Core: ");
                Serial.print(xPortGetCoreID());

                Serial.println("");

                xSemaphoreGive(printMutex);
            }
            else{
                Serial.println("Orientation: Mutex not acquired");
            }
        }
    }
}

