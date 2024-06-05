#include <Wire.h>
#include "TSYS01.h"

TSYS01 Tsensor;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting");

    Wire.begin();
  
    while (!Tsensor.init()) {
        Serial.println("TSYS01 device failed to initialize!");
        delay(2000);
    }
}

void loop() {
    Tsensor.read();
    Serial.print("Temperature: ");
    Serial.print(Tsensor.temperature()); 
    Serial.println(" deg C");
    delay(1000);
}