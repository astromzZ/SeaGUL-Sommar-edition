#include "ICM_20948.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>


#define SDA_PIN 17
#define SCL_PIN 16

#define AD0_VAL 1

ICM_20948_I2C myICM;

const float real_pitch = -20 * PI / 180;
// const float real_roll = 0;
// const float real_yaw = ...;

struct SensorData{
    float sensor1Data;
    float sensor2Data;
};


QueueHandle_t sensorDataQueue;

SemaphoreHandle_t printMutex;

void printing(void* pvParameters);

void orientation(void* pvParameters0);


void setup() {
    // Declare pins as output:
    pinMode(LED_BUILTIN, OUTPUT);
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
        Serial.println("Failed to create mutex");
        return;
    }
  
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
    digitalWrite(LED_BUILTIN, LOW);

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
        printing,
        "printing of orientation data",
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

void printing(void* pvParameters) {
    SensorData receivedData;
    while (true) {
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
            if (xSemaphoreTake(printMutex, portMAX_DELAY)){
                Serial.print("Pitch:");
                Serial.print(receivedData.sensor1Data * 180 / PI);
                Serial.print("°\t");

                Serial.print("Roll:");
                Serial.print(receivedData.sensor2Data * 180 / PI);
                Serial.print("°\t");

                Serial.print("Core: ");
                Serial.print(xPortGetCoreID());

                Serial.println("");

                xSemaphoreGive(printMutex);
            }
            else {
                Serial.print("Printing: Mutex not aquired");
            }
            // Serial.print ("Hello");
            // Serial.print (" World from printing() at ");
            // Serial.println (xPortGetCoreID());
        }
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

            xQueueSend(sensorDataQueue, &data, portMAX_DELAY);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            
            // Calculate yaw
            // float magX = myICM.magX() * cos(pitch) + myICM.magY() * sin(roll) * sin(pitch) + myICM.magZ() * cos(roll) * sin(pitch);
            // float magY = myICM.magY() * cos(roll) - myICM.magZ() * sin(roll);
            // float yaw = atan2(-magY, magX);
    // -160 * PI / 180;
            if (xSemaphoreTake(printMutex, portMAX_DELAY)){
                Serial.print("Pitch: ");
                Serial.print(data.sensor1Data * 180 / PI); // Convert to degrees
                Serial.print("°\t");

                Serial.print(" Roll: ");
                Serial.print(data.sensor2Data * 180 / PI); // Convert to degrees
                Serial.print("°\t");

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

