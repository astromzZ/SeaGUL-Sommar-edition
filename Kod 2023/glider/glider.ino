#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Stepper.h>
#include "MS5837.h"
#include <SoftwareSerial.h>



/* Orientation sensor
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GND to GND

   SOS sensor 
   Connecitons: (Arduino 1)
   Connect Leak to digital 6 (Arduino 1)
   Connect VDD to 5V DC (Arduino 1)
   Connect GND to GND (Arduino 1)

   Levelconverter
   Connections:
   VDD to 5V  (Arduino 2)
   GND to GND (Arduino 2)
   SCL to SCL (Arduino 2)
   SDA to SDA (Arduino 2)

   VDD to 5V  (Arduino 1)
   GND to GND (Arduino 1)
   SCL to analog 4 (Arduino 1)
   SDA to analog 5 (Arduino 1)

   Pressure sensor
   Connections:
   Connect SDA to Levelconverter SDL
   Connect SCL to Levelconverter SCL
   Connect VDD to Levelconverter VDD
   Connect GND to Levelconverter GND

   Stepper controller rotation
   Connections:
   Motor A + to AIN1 to digital 2 (Arduino 1)
   Motor A - to AIN2 to digital 3 (Arduino 1)
   Motor B + to BIN1 to digital 4 (Arduino 1)
   Motor B - to BIN2 to digital 5 (Arduino 1)

   Stepper controller translation
   Connections:
   Motor A + to AIN1 to digital 9 (Arduino 1)
   Motor A - to AIN2 to digital 8 (Arduino 1)
   Motor B + to BIN1 to digital 7 (Arduino 1)
   Motor B - to BIN2 to digital 6 (Arduino 1)

   SSR Ventil
   Connections: 
   GND to GND (Arduino 2)
   Signal cable to digital 3 (Arduino 2)

   SSR Pump
   Connections:
   GND to GND (Arduino 2)
   Signal cable to digital 4 (Arduino 2)
*/

/* Delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

int errorRotation = 10;
int errorRising = 10;  //Change
int pitchAngle = 0;    //Change
int rotationSpeed = 10;
int translationSpeed = 60;
int nrOfSteps = 10;
int nrRound = 1;  //Change



// Number of steps per revolution for our motor
const int stepsPerRevolution = 200;

int depth = 0;       //Variable that checks if desired depth is reached
int maxDepth = 100;  //Change
int minDepth = 10;   //Change

float value = 0;
float voltage = 0;
int minVoltage = 26;  //Change, not under 24V
int delayVolt = 100;
//Orientiation sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

// Initiate stepper motor for rotation and tanslation
Stepper stepperRotation(stepsPerRevolution, 2, 3, 4, 5);
Stepper stepperTranslation(stepsPerRevolution, 9, 8, 7, 6);

//Pressure sensor
MS5837 sensor;

int state = 0;
SoftwareSerial bt(11, 12);
//Initiate all the sensors
void setup() {
  bt.begin(9600);
  Serial.begin(9600);
  
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  //Initialise the orientation sensor
  if (!bno.begin()) {
    //Orientation sensor not detected
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  
  
  //Initiate pressure sensor
  Serial.println("Starting pressure sensor");

  Wire.begin();

  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);  // kg/m^3 (freshwater, 1029 for seawater)
}

}

//Part of the program that needs to update
void loop() {

  if (bt.available()) /* If data is available on serial port */
  {

    // Reads the data from the serial port
    state = bt.read();
  }
  /*
  Motor rotation:
  A-clockwise
  B-counterclockwise

  Motor translation:
  C-forward
  D-backwards
  */

  if (state == 'A') {
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(nrOfSteps);
    state = 0;
  } else if (state == 'B') {
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps);
    state = 0;
  } else if (state == 'C') {
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(nrOfSteps);
    state = 0;
  } else if (state == 'D') {
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(-nrOfSteps);
    state = 0;
  } else if (state == 'P') {
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    //Prints status
    Serial.println();
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);
    Serial.println("--");
    delay(BNO055_SAMPLERATE_DELAY_MS);  //Updates every 100ms
    if (mag == 3 && system == 3 && gyro == 3 && accel == 3) {
      state = 0;
    }
  }  else if (state == 'Z') {
  //Data that should be gathered by the orientation data
  sensors_event_t orientationData, linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);  //Data for the degress (divaing, resurfing, rotation, ?heading?)
  //Data gathered from pressure sensor to check depth
  sensor.read();
  Serial.print("Djup= ");
  Serial.println(sensor.depth());

  value = analogRead(A0);
  voltage = 9.2 * ((value * 5.0 / 1024) - 0.55);
  Serial.print("Voltage = ");
  Serial.println(voltage);
  delay(delayVolt);


  //compass(&orientationData);
  //rotationEvent(&orientationData);
  //pitchEvent(&orientationData);


  if (sensor.depth() < maxDepth) {
    pitchAngle = -45;
    rotationEvent(&orientationData);
    pitchEvent(&orientationData);
  } else if (sensor.depth() >= maxDepth) {
    pitchAngle = 45;
    rotationEvent(&orientationData);
    pitchEvent(&orientationData);
  }

  if (sensor.depth() >= maxDepth) {
    depth = depth + 1;
    if (depth >= nrRound) {
      pitchAngle = 45;
      pitchEvent(&orientationData);
      rotationEvent(&orientationData);
    }
    
  }*/
  }


//Function to cheack the rotation if the glider
void rotationEvent(sensors_event_t* event) {
  //dumb values, easy to spot problem
  double rotation = -1000000;

  //Collects data of the rotation for the glider in degr
  rotation = event->orientation.z + 180;
  Serial.print("Rotation= ");
  Serial.println(rotation);

  //if the glider rotates to much clockwise, call the motors to compansate by rotatin the batteries
  if (rotation >= errorRotation && rotation <= 180) {
    //Sets the speed, number of steps and directiom for the stepper motor
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(nrOfSteps);
    delay(500);
  }
  //if the glider rotates to much counterclockwise, call the motors to compansate by rotatin the batteries
  else if (rotation <= 360 - errorRotation && rotation > 180) {
    //Sets the speed, number of steps and directiom for the stepper moto  r
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps);
    delay(500);
  }
  //Otherwise the glider rotation is optimal
  else
    Serial.print(" Rotation Glidaren ligger bra");
  Serial.println("------------------");
}
//Function for controlling the glider when resurfaces
void pitchEvent(sensors_event_t* event) {
  //dumb values, easy to spot problem
  double pitchRising = -1000000;
  //Collecting data for angel when resurface in degrees
  pitchRising = event->orientation.y;
  Serial.print("Pitch= ");
  Serial.println(pitchRising);

  //Cheacks if the glider angel is to much, compensate by turning on motor for translation of the batteries
  if (pitchRising <= pitchAngle - errorRising && pitchRising > -90) {
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(-nrOfSteps);
    delay(500);
  }
  //Cheacks if the glider angel is to little, compensate by turning on motor for translation of the batteries
  else if (pitchRising > pitchAngle + errorRising && pitchRising <= 90) {
    //Pushes batteries backwards
    //Sets the speed, number of steps and directiom for the stepper motor
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(nrOfSteps);
    delay(500);
  }
  //Otherwise the glider angel is optimal
  else
    Serial.print("Pitch Glidaren ligger bra");
  Serial.println("------------------");
}
//Function for controlling the glider when diving


/*void compass(sensors_event_t* event) {
  double holding = -1000000;

  //Collects data of the rotation for the glider in degr
  Serial.print("Holding: ");
  holding = event->orientation.x;
  Serial.print(holding);
  Serial.println("holding: ");
}*/
