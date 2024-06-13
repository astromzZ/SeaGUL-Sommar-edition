#include <Arduino.h>

#define PUMP_PIN 5

void setup() {
    Serial.begin(115200);
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
}

void loop() {
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println("Pump pin high");

    delay(10000);

    digitalWrite(PUMP_PIN, LOW);
    Serial.println("Pump pin low");

    delay(20000);
}