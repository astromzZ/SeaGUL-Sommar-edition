#include <Arduino.h>
#include <ICM_20948.h>

#define stepsPerRevolution 200
#define STEPS_PER_CM 600
#define STEPS_PER_DEGREE 10 //Ta reda på detta värde

#define TRANSLATION_MIN_POSITION_CM -1
#define TRANSLATION_MAX_POSITION_CM 1
#define TRANSLATIONMOTOR_STEP_SIZE 100

#define ROTATION_MIN_DEGREE -90
#define ROTATION_MAX_DEGREE 90
#define ROTATIONMOTOR_STEP_SIZE 100

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
#define MODE0 5
#define MODE1 6
#define MODE2 7
DRV8825 transtepper(stepsPerRevolution, TRAN_DIR_PIN, TRAN_STEP_PIN, TRAN_SLEEP_PIN, MODE0, MODE1, MODE2);
DRV8825 rotstepper(stepsPerRevolution, ROT_DIR_PIN, ROT_STEP_PIN, ROT_SLEEP_PIN, MODE0, MODE1, MODE2);

// PID Variables
// double pitchSetpoint = 0, pitchInput, pitchOutput;
// double rollSetpoint = 0, rollInput, rollOutput;
// PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 2, 5, 1, DIRECT);
// PID rollPID(&rollInput, &rollOutput, &rollSetpoint, 2, 5, 1, DIRECT);

ICM_20948_I2C myICM;

struct SensorData {
    float pitch;
    float roll;
};

QueueHandle_t sensorDataQueue;

void steppermotor(void* pvParameters);
void orientation(void* pvParameters);

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

    // pitchPID.SetMode(AUTOMATIC);
    // rollPID.SetMode(AUTOMATIC);
    // pitchPID.SetOutputLimits(-TRANSLATIONMOTOR_STEP_SIZE, TRANSLATIONMOTOR_STEP_SIZE);
    // rollPID.SetOutputLimits(-ROTATIONMOTOR_STEP_SIZE, ROTATIONMOTOR_STEP_SIZE);

}

void loop() {
    // The loop function is intentionally left empty as we use tasks.
}

void steppermotor(void* pvParameters) {
    bool translationmotorRunning = false;
    bool rotationmotorRunning = false;
    float currentPosition = 0;
    float currentDegree = 0;
    SensorData receivedData;
    float pitch_error = 0;
    float roll_error = 0;

    while (true) {
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY)) {
            // Serial.println("Received data from queue");

            //Translation motor
            pitch_error = 0 - (receivedData.pitch * 180 / PI);
            int transl_direction = (pitch_error < 0) ? 1 : -1;
            float nextPosition = currentPosition + transl_direction * (TRANSLATIONMOTOR_STEP_SIZE / (float)STEPS_PER_CM);

            //Rotation motor
            roll_error = 0 - (receivedData.roll * 180/PI);
            int rot_direction = (roll_error < 0) ? 1 : -1;
            float nextDegree = currentDegree + rot_direction * (ROTATIONMOTOR_STEP_SIZE / (float)STEPS_PER_DEGREE);

            if (fabs(pitch_error) > 5) {
                // Serial.print("Pitch error exceeds tolerance, starting motor.");
                if (!translationmotorRunning) {
                    digitalWrite(TRAN_SLEEP_PIN, HIGH);
                    translationmotorRunning = true;
                }

                if (nextPosition < TRANSLATION_MAX_POSITION_CM && nextPosition > TRANSLATION_MIN_POSITION_CM) {
                    transtepper.setMicrostep(2);
                    transtepper.move(transl_direction > 0 ? 2 * TRANSLATIONMOTOR_STEP_SIZE : -2 * TRANSLATIONMOTOR_STEP_SIZE);

                    currentPosition = nextPosition;

                    // Serial.print("Moving to: ");
                    // Serial.print(currentPosition);
                } else {
                    // Serial.print("Next position out of bounds, changing transl_direction.");
                    // transl_direction = !transl_direction;
                    // digitalWrite(TRAN_DIR_PIN, transl_direction ? HIGH : LOW);

                    if (!translationmotorRunning) {
                        digitalWrite(TRAN_SLEEP_PIN, HIGH);
                        translationmotorRunning = true;
                    }

                    if ((transl_direction > 0 && currentPosition>= TRANSLATION_MAX_POSITION_CM) || (transl_direction < 0 && currentPosition<=TRANSLATION_MIN_POSITION_CM)){
                        transtepper.setMicrostep(2);
                        transtepper.move(transl_direction > 0 ? 2 * TRANSLATIONMOTOR_STEP_SIZE : -2 * TRANSLATIONMOTOR_STEP_SIZE);

                        currentPosition = nextPosition;

                        // Serial.print("Moving to: ");
                        // Serial.print(currentPosition);
                    }
                }
            } else {
                // Serial.print("Pitch error within tolerance, stopping motor.");
                if (translationmotorRunning) {
                    digitalWrite(TRAN_SLEEP_PIN, LOW);
                    translationmotorRunning = false;
                }
            }


            if (fabs(roll_error) > 5) {
                Serial.println("Roll error exceeds tolerance, starting motor.");
                if (!rotationmotorRunning) {
                    digitalWrite(ROT_SLEEP_PIN, HIGH);
                    rotationmotorRunning = true;
                }

                if (nextDegree < ROTATION_MAX_DEGREE && nextDegree > ROTATION_MIN_DEGREE) {
                    rotstepper.setMicrostep(1);
                    rotstepper.move(rot_direction > 0 ? ROTATIONMOTOR_STEP_SIZE : -ROTATIONMOTOR_STEP_SIZE);

                    currentDegree = nextDegree;

                    Serial.print("Moving to ");
                    Serial.print(currentDegree);
                }
                else {

                    if (!rotationmotorRunning) {
                        digitalWrite(ROT_SLEEP_PIN, HIGH);
                        rotationmotorRunning = true;
                    }

                    if ((rot_direction > 0 && currentDegree >= ROTATION_MAX_DEGREE) || (rot_direction < 0 && currentDegree <= ROTATION_MIN_DEGREE)) {
                        rotstepper.setMicrostep(1);
                        rotstepper.move(rot_direction > 0 ? ROTATIONMOTOR_STEP_SIZE : -ROTATIONMOTOR_STEP_SIZE);

                        currentDegree = nextDegree;

                        Serial.print("Moving to ");
                        Serial.print(currentDegree);
                    }
                }
            } else {
                Serial.println("Roll error within tolerance, stopping motor.");
                if (rotationmotorRunning) {
                    digitalWrite(ROT_SLEEP_PIN, LOW);
                    rotationmotorRunning = false;
                }
            }

            // Serial.print("Current Pitch: ");
            // Serial.print(receivedData.pitch * 180 / PI);
            // Serial.print(", Pitch error: ");
            // Serial.print(pitch_error);
            // Serial.print(", Current Position: ");
            // Serial.print(currentPosition);
            Serial.print(", Current Roll: ");
            Serial.print(receivedData.roll * 180/PI);
            Serial.print(", Roll error: ");
            Serial.print(roll_error);
            Serial.print(", Current degree: ");
            Serial.print(currentDegree);
        }
    }
}

void orientation(void* pvParameters) {
    while (true) {
        if (myICM.dataReady()) {
            SensorData data;
            myICM.getAGMT(); // The values are only updated when you call 'getAGMT'

            // Calculate pitch and roll
            data.pitch = atan2(myICM.accY(), sqrt(myICM.accX() * myICM.accX() + myICM.accZ() * myICM.accZ()));
            data.roll = -atan2(-myICM.accX(), myICM.accZ());

            if (xQueueSend(sensorDataQueue, &data, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send data to queue");
            }
        } else {
            Serial.println("Sensor data not ready");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adding a delay to reduce bus congestion and improve stability
    }
}

