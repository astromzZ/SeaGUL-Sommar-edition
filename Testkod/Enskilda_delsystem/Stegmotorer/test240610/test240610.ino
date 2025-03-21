#include <Arduino.h>
#include <ICM_20948.h>

#define stepsPerRevolution 200
#define STEPS_PER_CM 600

#define MIN_POSITION_CM -1
#define MAX_POSITION_CM 1
#define MOTOR_STEP_SIZE 5

#define AD0_VAL 1

#define RPM 186

#define DIR_PIN 8
#define STEP_PIN 18
#define SLEEP_PIN 17

#define SDA_PIN 13
#define SCL_PIN 14

#include "DRV8825.h"
#define MODE0 10
#define MODE1 11
#define MODE2 12
DRV8825 stepper(stepsPerRevolution, DIR_PIN, STEP_PIN, SLEEP_PIN, MODE0, MODE1, MODE2);

ICM_20948_I2C myICM;

struct SensorData {
    float sensor1Data;
};

QueueHandle_t sensorDataQueue;

void steppermotor(void* pvParameters);
void orientation(void* pvParameters);

void setup() {
    Serial.begin(115200);
    stepper.begin(RPM);
    stepper.enable();
    pinMode(SLEEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

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

    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorDataQueue == NULL) {
        Serial.println("Failed to create queue");
        return;
    }

    xTaskCreatePinnedToCore(
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
    // The loop function is intentionally left empty as we use tasks.
}

void steppermotor(void* pvParameters) {
    bool motorRunning = false;
    float currentPosition = 0;
    // bool direction = true;  // true for forward, false for backward
    SensorData receivedData;
    float error = 0;

    while (true) {
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
            Serial.println("Received data from queue");
            error = 0 - (receivedData.sensor1Data * 180 / PI);
            int direction = (error < 0) ? 1 : -1;
            float nextPosition = currentPosition + direction * (MOTOR_STEP_SIZE / (float)STEPS_PER_CM);

            if (fabs(error) > 5) {
                Serial.println("Error exceeds tolerance, starting motor");
                if (!motorRunning) {
                    // digitalWrite(DIR_PIN, direction ? HIGH : LOW);
                    digitalWrite(SLEEP_PIN, HIGH);
                    motorRunning = true;
                }

                if (nextPosition < MAX_POSITION_CM && nextPosition > MIN_POSITION_CM) {
                    stepper.setMicrostep(1);
                    stepper.move(direction > 0? MOTOR_STEP_SIZE : -MOTOR_STEP_SIZE);

                    currentPosition = nextPosition;

                    Serial.print("Moving to: ");
                    Serial.print(currentPosition);
                } else {
                    // Serial.print("Next position out of bounds, changing direction.");
                    // direction = !direction;
                    // digitalWrite(DIR_PIN, direction ? HIGH : LOW);

                    if (!motorRunning) {
                        // digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
                        digitalWrite(SLEEP_PIN, HIGH);
                        motorRunning = true;
                    }

                    if ((direction > 0 && currentPosition>= MAX_POSITION_CM) || (direction < 0 && currentPosition<=MIN_POSITION_CM)){
                        stepper.setMicrostep(1);
                        stepper.move(direction > 0 ? MOTOR_STEP_SIZE : -MOTOR_STEP_SIZE);

                        currentPosition = nextPosition;

                        Serial.print("Moving to: ");
                        Serial.print(currentPosition);
                    }
                }
            } else {
                Serial.print("Error within tolerance, stopping motor.");
                if (motorRunning) {
                    digitalWrite(SLEEP_PIN, LOW);
                    motorRunning = false;
                }
            }

            Serial.print("Current Pitch: ");
            Serial.print(receivedData.sensor1Data * 180 / PI);
            Serial.print(", Error: ");
            Serial.print(error);
            Serial.print(", Current Position: ");
            Serial.print(currentPosition);
        }
    }
}

void orientation(void* pvParameters) {
    while (true) {
        if (myICM.dataReady()) {
            SensorData data;
            myICM.getAGMT(); // The values are only updated when you call 'getAGMT'

            // Calculate pitch
            data.sensor1Data = atan2(myICM.accY(), sqrt(myICM.accX() * myICM.accX() + myICM.accZ() * myICM.accZ()));

            if (xQueueSend(sensorDataQueue, &data, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send data to queue");
            }
        } else {
            Serial.println("Sensor data not ready");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adding a delay to reduce bus congestion and improve stability
    }
}
