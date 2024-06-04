  #include <Wire.h>
  #include <Stepper.h>
  #include <SoftwareSerial.h>
  #include <Servo.h>

byte servoPin = A3;
Servo servo;

//Initiate pump and ventil
int pumpPin = 4;
int ventilPin = 3;
int minVolume = 12.4;
int maxVolume = 2.5;
//Change  //Change, not under 24V
//Start value of leak indicator¨
/* Delay between fresh samples */
//Change
int rotationSpeed = 60;
int translationSpeed = 60;
int nrOfSteps = 50;

// Number of steps per revolution for our motor
const int stepsPerRevolution = 200;

// Initiate stepper motor for rotation and tanslation
Stepper stepperRotation(stepsPerRevolution, 5, 6, 7, 8);
Stepper stepperTranslation(stepsPerRevolution, 2, 9, 10, 13);

int state = 0;
SoftwareSerial bt(12, 11);

void setup() {
  bt.begin(9600);
  Serial.begin(9600);
  analogRead(servoPin);
  servo.attach(servoPin);
  servo.writeMicroseconds(1500);  // send "stop" signal to ESC.
  delay(1000);
  
  //Setup for pump and ventil
  pinMode(ventilPin, OUTPUT);    //Ventil
  digitalWrite(ventilPin, LOW);  //Closed
  pinMode(pumpPin, OUTPUT);      //Pump
  digitalWrite(pumpPin, LOW);    //Closed                            //Initiate the SOS-sensor
}
void loop() {
    //Closed
       //Pump
   /* stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(nrOfSteps);
    delay(500);
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps);
    delay(500);
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(nrOfSteps);
    delay(500);
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(-nrOfSteps);
    delay(500);
    */
  if (bt.available()) /* If data is available on serial port */
  {
    // Reads the data from the serial port
    state = bt.read();
  }
  if (state == 'A') {//Motsols
  Serial.println("A");
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(nrOfSteps);
    state = 0;
  } else if (state == 'B') {//Medsols
  Serial.println("B");
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps);
    state = 0;
  } else if (state == 'C') {//Framåt
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(nrOfSteps);
    state = 0;
  } else if (state == 'D') {//Bakåt 
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(-nrOfSteps);
    state = 0;
  } else if (state == 'E') {
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
  } else if (state == 'K') {//Thruster bakåt
    servo.writeMicroseconds(1364);
    state = 0;
  } else if (state == 'T') {//Thruster framåt 
    servo.writeMicroseconds(1636);
    delay(15000);
    servo.writeMicroseconds(1500);
    state=0;
  } else if (state == 'Y') {//Stoppa thruster 
    servo.writeMicroseconds(1500);
    state = 0;
  }
}