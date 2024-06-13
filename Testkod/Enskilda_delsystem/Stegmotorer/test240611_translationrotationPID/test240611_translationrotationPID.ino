#include <Arduino.h>
#include <ICM_20948.h>
#include <PID_v1.h> // Include the PID library

#define stepsPerRevolution 200
#define STEPS_PER_CM 600
#define STEPS_PER_DEGREE 10

#define TRANSLATION_MIN_POSITION_CM -1
#define TRANSLATION_MAX_POSITION_CM 1
#define TRANSLATIONMOTOR_STEP_SIZE 5

#define ROTATION_MIN_DEGREE -90
#define ROTATION_MAX_DEGREE 90
#define ROTATIONMOTOR_STEP_SIZE 1

#define AD0_VAL 1

#define RPM 186

#define TRAN_DIR_PIN 17
#define TRAN_STEP_PIN 16
#define TRAN_SLEEP_PIN 15

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

ICM_20948_I2C myICM;

struct SensorData {
    float pitch;
    float roll;
};

QueueHandle_t sensorDataQueue;

// Define PID variables
double pitchSetpoint = 0, pitchInput, pitchOutput;
double rollSetpoint = 0, rollInput, rollOutput;
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, 1, 0, 0, DIRECT); // Proportional tuning values
PID rollPD(&rollInput, &rollOutput, &rollSetpoint, 1, 0.1, 0, DIRECT);     // Proportional-Derivative tuning values

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
        orientation,
        "orientation measurement",
        2048,
        NULL,
        1,
        NULL,
        0
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

    pitchPID.SetMode(AUTOMATIC);
    rollPD.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(-TRANSLATIONMOTOR_STEP_SIZE, TRANSLATIONMOTOR_STEP_SIZE);
    rollPD.SetOutputLimits(-ROTATIONMOTOR_STEP_SIZE, ROTATIONMOTOR_STEP_SIZE);
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
            pitch_error = 0 - (receivedData.pitch * 180 / PI);
            int transl_direction = (pitch_error < 0) ? 1 : -1;
            float nextPosition = currentPosition + transl_direction * (TRANSLATIONMOTOR_STEP_SIZE / (float)STEPS_PER_CM);

            roll_error = 0 - (receivedData.roll * 180 / PI);
            int rot_direction = (roll_error < 0) ? 1 : -1;
            float nextDegree = currentDegree + rot_direction * (ROTATIONMOTOR_STEP_SIZE / (float)STEPS_PER_DEGREE);

            pitchInput = pitch_error; // Set PID input
            rollInput = roll_error;   // Set PD input

            pitchPID.Compute(); // Calculate PID output
            rollPD.Compute();   // Calculate PD output

            if (fabs(pitch_error) > 5) {
                if (!translationmotorRunning) {
                    digitalWrite(TRAN_SLEEP_PIN, HIGH);
                    translationmotorRunning = true;
                }

                if (nextPosition < TRANSLATION_MAX_POSITION_CM && nextPosition > TRANSLATION_MIN_POSITION_CM) {
                    transtepper.setMicrostep(8);
                    transtepper.move(transl_direction > 0 ? TRANSLATIONMOTOR_STEP_SIZE : -TRANSLATIONMOTOR_STEP_SIZE);
                    currentPosition = nextPosition;
                }
            } else {
                if (translationmotorRunning) {
                    digitalWrite(TRAN_SLEEP_PIN, LOW);
                    translationmotorRunning = false;
                }
            }

            if (fabs(roll_error) > 5) {
                if (!rotationmotorRunning) {
                    digitalWrite(ROT_SLEEP_PIN, HIGH);
                    rotationmotorRunning = true;
                }

                if (nextDegree < ROTATION_MAX_DEGREE && nextDegree > ROTATION_MIN_DEGREE) {
                    rotstepper.setMicrostep(1);
                    rotstepper.move(rot_direction > 0 ? ROTATIONMOTOR_STEP_SIZE : -ROTATIONMOTOR_STEP_SIZE);
                    currentDegree = nextDegree;
                }
            } else {
                if (rotationmotorRunning) {
                    digitalWrite(ROT_SLEEP_PIN, LOW);
                    rotationmotorRunning = false;
                }
            }
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
