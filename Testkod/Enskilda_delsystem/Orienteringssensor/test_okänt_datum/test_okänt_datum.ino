#include "ICM_20948.h"


#define SDA_PIN 17
#define SCL_PIN 16

#define AD0_VAL 1


// ICM_20948_AGMT_t agmt;

ICM_20948_I2C myICM;

const float real_pitch = -20 * PI / 180;
// const float real_roll = 0;
// const float real_yaw = ...;

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

}



void loop() {
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

    // pitch_reg();
    // roll_reg();
    // yaw_reg();
  }
  
  else {
    Serial.println("Waiting for data");
    delay(500);
  }
}

//void pitch_reg() {
//   bool (pressure > max_pressure) = up;
//   float sign = 2*up-1;
//   float real_pitch = sign*real_pitch;
//   // if (up) {
//   float eps = PI / 180;
//   if (pitch > real_pitch + eps) {
//     digitalWrite(dirPin, HIGH);
//     digitalW
//   }
//   // }
// }