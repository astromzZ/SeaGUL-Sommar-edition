#include "ICM_20948.h"


#define SDA_PIN 17
#define SCL_PIN 16

#define AD0_VAL 1


// ICM_20948_AGMT_t agmt;

ICM_20948_I2C myICM;

const float real_pitch = -20 * PI / 180;
// const float real_roll = 0;
// const float real_yaw = ...;



void loop2(void* pvParameters);

void orientation(void* pvParameters0);


void setup() {

    // Declare pins as output:
    pinMode(LED_BUILTIN, OUTPUT);
    //put your setup code here, to run once:
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.begin(115200);
  
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
        loop2,
        "loop2",
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

void loop2(void* pvParameters) {
    while (true) {
        Serial.print ("Hello");
        Serial.print (" World from loop2() at ");
        Serial.println (xPortGetCoreID());
    }
}

void orientation(void* pvParameters) {
    while(true) {
        if (myICM.dataReady()) {
            myICM.getAGMT();         // The values are only updated when you call 'getAGMT'

            // Calculate pitch and roll
            float roll = (atan2(myICM.accY(), sqrt(myICM.accX() * myICM.accX() + myICM.accZ() * myICM.accZ())));
            float pitch = -atan2(-myICM.accX(), myICM.accZ());
    
            // Calculate yaw
            float magX = myICM.magX() * cos(pitch) + myICM.magY() * sin(roll) * sin(pitch) + myICM.magZ() * cos(roll) * sin(pitch);
            float magY = myICM.magY() * cos(roll) - myICM.magZ() * sin(roll);
            float yaw = atan2(-magY, magX);
    // -160 * PI / 180;

            Serial.print("Pitch: ");
            Serial.print(pitch * 180 / PI); // Convert to degrees
            Serial.print(" Roll: ");
            Serial.print(roll * 180 / PI); // Convert to degrees
            Serial.print(" Yaw: ");
            Serial.print(yaw * 180 / PI); // Convert to degrees
            Serial.println();

        }
    }
}

