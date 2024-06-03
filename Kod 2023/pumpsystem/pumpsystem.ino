#include <Wire.h>
#include "MS5837.h"
#include <SoftwareSerial.h>
#include <Servo.h>

float value = 0;
float voltage = 0;

byte servoPin = 9;
Servo servo;

int maxDepth = 100;  //Change
int minDepth = 20;   //Change
//Initiate pump and ventil
int pumpPin = 4;
int ventilPin = 3;
//Iniciate potensiometer
int potensPin = A2;
int readVal;
float V2;
float potensValue;
int minVolume = 12.4;
int maxVolume = 2.5;
int delayPoten = 100;
int delayVolt = 100;
int depth = 0;
int nrRound = 2;      //Change
int minVoltage = 26;  //Change, not under 24V

int leakPin = 6;  // Leak Signal Pin
int leak = 0;     //Start value of leak indicator¨

//Initiate pressure sensor
MS5837 sensor;

int state = 0;
SoftwareSerial bt(11, 12);

void setup() {
  bt.begin(9600);
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.writeMicroseconds(1500);  // send "stop" signal to ESC.
  delay(7000);

  //Setup for pump and ventil
  pinMode(ventilPin, OUTPUT);    //Ventil
  digitalWrite(ventilPin, LOW);  //Closed
  pinMode(pumpPin, OUTPUT);      //Pump
  digitalWrite(pumpPin, LOW);    //Closed
                                 //Initiate the SOS-sensor
  pinMode(leakPin, INPUT);
  /*
  Serial.println("Starting");
  //Setup for pressure sensor
  Wire.begin();
  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);  // kg/m^3 (freshwater, 1029 for seawater)
}*/
}


void loop() {


  if (bt.available())  //If data is available on serial port
  {

    // Reads the data from the serial port
    state = bt.read();
  }

  /*
  Ventil:
  E-Close
  F-Open

  Pump:
  G-Shutoff
  H-Start
*/

  if (state == 'E') {
    digitalWrite(ventilPin, LOW);
    state = 0;
  } else if (state == 'F') {
    digitalWrite(ventilPin, HIGH);
    state = 0;
  } else if (state == 'G') {
    digitalWrite(pumpPin, LOW);
    state = 0;
  } else if (state == 'H') {
    digitalWrite(pumpPin, HIGH);
    state = 0;
  } else if (state == 'K') {
    servo.writeMicroseconds(1364);  // Send signal to ESC.
  } else if (state == 'Y') {
    servo.writeMicroseconds(1500);
  } else if (state == 'O') {

    readVal = analogRead(potensPin);
    V2 = (5 / 1023.) * readVal;
    potensValue = 20 - (20 / 1023.) * readVal;
    Serial.print("Potensiometer= ");
    Serial.println(potensValue);
    delay(delayPoten);

    value = analogRead(A0);
    voltage = 10 * ((value * 5.0 / 1024) - 0.55);
    Serial.print("Voltage = ");
    Serial.println(voltage);
    delay(delayVolt);

    leak = digitalRead(leakPin);

    sensor.read();
    Serial.print("Djup= ");
    Serial.println(sensor.depth());

    if (sensor.depth() >= maxDepth) {
      depth = depth + 1;
      if (depth >= nrRound) {
        digitalWrite(pumpPin, HIGH);
        digitalWrite(ventilPin, LOW);
        if (potensValue == minVolume) {
          digitalWrite(pumpPin, LOW);
        }
      }
    }

    if (voltage <= minVoltage) {
      Serial.println("Leak");
      //digitalWrite(pumpPin,HIGH);
      digitalWrite(ventilPin, LOW);
      if (potensValue <= minVolume) {
        digitalWrite(pumpPin, LOW);
      }
    } else if (sensor.depth() < maxDepth && potensValue >= maxVolume) {
      digitalWrite(pumpPin, LOW);
      digitalWrite(ventilPin, HIGH);
    }

    else if (sensor.depth() > minDepth && potensValue <= minVolume) {
      digitalWrite(pumpPin, HIGH);
      digitalWrite(ventilPin, LOW);
    }

    else {
      digitalWrite(pumpPin, LOW);
      digitalWrite(ventilPin, LOW);
    }
  }
}
