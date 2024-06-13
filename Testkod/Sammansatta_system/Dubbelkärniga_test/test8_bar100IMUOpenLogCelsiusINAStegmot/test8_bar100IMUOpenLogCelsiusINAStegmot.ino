#include <Arduino.h>
#include <ICM_20948.h>
#include <KellerLD.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include <TSYS01.h>


#define stepsPerRevolution 200
#define STEPS_PER_CM 600
#define STEPS_PER_DEGREE 10 //Ta reda på detta värde

#define TRANSLATION_MIN_POSITION_CM -1
#define TRANSLATION_MAX_POSITION_CM 1
#define TRANSLATIONMOTOR_STEP_SIZE 5

#define ROTATION_MIN_DEGREE -90
#define ROTATION_MAX_DEGREE 90
#define ROTATIONMOTOR_STEP_SIZE 1

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
#define MODE0 10
#define MODE1 11
#define MODE2 12
DRV8825 transtepper(stepsPerRevolution, TRAN_DIR_PIN, TRAN_STEP_PIN, TRAN_SLEEP_PIN, MODE0, MODE1, MODE2);
DRV8825 rotstepper(stepsPerRevolution, ROT_DIR_PIN, ROT_STEP_PIN, ROT_SLEEP_PIN, MODE0, MODE1, MODE2);

ICM_20948_I2C myICM;

struct SensorData {
    float pitch;
    float roll;
    float depth;
    float pressure;
};

QueueHandle_t sensorDataQueue;

KellerLD Psensor;

TSYS01 Tsensor;

OpenLog myLog;

String fileName = "SD-test8_240612.txt";


void openLogSetup();
void steppermotor(void* pvParameters);
void datagathering(void* pvParameters);

void writeSD(String measurement) {
  myLog.append(fileName); //Bestämmer att vi ska sriva i denna filen 
  myLog.println(measurement); //Skriver till filen
  myLog.syncFile();
}

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

    Psensor.init();
    Psensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    if (Psensor.isInitialized()) {
        Serial.println("Pressure sensor connected.");
    } else {
        Serial.println("Pressure sensor not connected.");
    }

    Tsensor.init();

    while (!Tsensor.init()) {
        Serial.println("TSYS01 device failed to initialize!");
        delay(1000);
    }


    openLogSetup();

    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorDataQueue == NULL) {
        Serial.println("Failed to create queue");
        return;
    }

    xTaskCreatePinnedToCore(
        datagathering,     // Function to implement the task
        "Measurement and storing of data",   // Name of the task
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
                    // digitalWrite(TRAN_DIR_PIN, transl_direction ? HIGH : LOW);
                    digitalWrite(TRAN_SLEEP_PIN, HIGH);
                    translationmotorRunning = true;
                }

                if (nextPosition < TRANSLATION_MAX_POSITION_CM && nextPosition > TRANSLATION_MIN_POSITION_CM) {
                    transtepper.setMicrostep(1);
                    transtepper.move(transl_direction > 0 ? TRANSLATIONMOTOR_STEP_SIZE : -TRANSLATIONMOTOR_STEP_SIZE);

                    currentPosition = nextPosition;

                    // Serial.print("Moving to: ");
                    // Serial.print(currentPosition);
                } else {
                    // Serial.print("Next position out of bounds, changing transl_direction.");
                    // transl_direction = !transl_direction;
                    // digitalWrite(TRAN_DIR_PIN, transl_direction ? HIGH : LOW);

                    if (!translationmotorRunning) {
                        // digitalWrite(TRAN_DIR_PIN, transl_direction > 0 ? HIGH : LOW);
                        digitalWrite(TRAN_SLEEP_PIN, HIGH);
                        translationmotorRunning = true;
                    }

                    if ((transl_direction > 0 && currentPosition>= TRANSLATION_MAX_POSITION_CM) || (transl_direction < 0 && currentPosition<=TRANSLATION_MIN_POSITION_CM)){
                        transtepper.setMicrostep(1);
                        transtepper.move(transl_direction > 0 ? TRANSLATIONMOTOR_STEP_SIZE : -TRANSLATIONMOTOR_STEP_SIZE);

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
                // Serial.println("Roll error exceeds tolerance, starting motor.");
                if (!rotationmotorRunning) {
                    digitalWrite(ROT_SLEEP_PIN, HIGH);
                    rotationmotorRunning = true;
                }

                if (nextDegree < ROTATION_MAX_DEGREE && nextDegree > ROTATION_MIN_DEGREE) {
                    rotstepper.setMicrostep(1);
                    rotstepper.move(rot_direction > 0 ? ROTATIONMOTOR_STEP_SIZE : -ROTATIONMOTOR_STEP_SIZE);

                    currentDegree = nextDegree;

                    // Serial.print("Moving to ");
                    // Serial.print(currentDegree);
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

                        // Serial.print("Moving to ");
                        // Serial.print(currentDegree);
                    }
                }
            } else {
                // Serial.println("Roll error within tolerance, stopping motor.");
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
            // Serial.print(", Current Roll: ");
            // Serial.print(receivedData.roll * 180/PI);
            // Serial.print(", Roll error: ");
            // Serial.print(roll_error);
            // Serial.print(", Current degree: ");
            // Serial.print(currentDegree);
        }
    }
}

void datagathering(void* pvParameters) {

    while (true) {
        if (myICM.dataReady()) {
            SensorData data;
            myICM.getAGMT(); // The values are only updated when you call 'getAGMT'

            // Calculate pitch and roll
            data.pitch = atan2(myICM.accY(), sqrt(myICM.accX() * myICM.accX() + myICM.accZ() * myICM.accZ()));
            data.roll = -atan2(-myICM.accX(), myICM.accZ());

            Psensor.read();

            data.depth = Psensor.depth();
            data.pressure = Psensor.pressure();

            if (xQueueSend(sensorDataQueue, &data, portMAX_DELAY) != pdPASS) {
                Serial.println("Failed to send data to queue");
            }

            Tsensor.read();

            float temperature = Tsensor.temperature(); 

            String logData = "Pitch: " + String(data.pitch * 180 / PI) + " degrees, " +
                             "Roll: " + String(data.roll * 180 / PI) + " degrees, " +
                             "Depth: " + String(data.depth) + " m, " +
                             "Pressure: " + String(data.pressure) + " mbar, " +
                             "Temperature: " + String(temperature) + " Celsius";
                            //  "Current: " + String(current) + " A";

            writeSD(logData);

        } else {
            Serial.println("Sensor data not ready");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adding a delay to reduce bus congestion and improve stability
    }
}

