#include <SoftwareSerial.h>
#include <Stepper.h>
#include <Servo.h>

const int stepsPerRevolution = 200;
Stepper stepperRotation(stepsPerRevolution, 6, 7, 8, 9);
Stepper stepperTranslation(stepsPerRevolution, 2, 3, 4, 5);

int pumpPin=13;
int ventilPin=10;
byte servoPin = A3;
Servo servo;

int rotationSpeed = 20;
int translationSpeed = 20;
int nrOfSteps = 100;

int state = 0;
SoftwareSerial bt(12, 11);
void setup() {
  bt.begin(9600);
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(1000);  

  pinMode(ventilPin, OUTPUT);//Ventil
  digitalWrite(ventilPin,LOW);//Closed
  pinMode(pumpPin,OUTPUT);//Pump
  digitalWrite(pumpPin,LOW);//Closed
}
 

void loop() {
    delay(500);
  

  if (bt.available()) /* If data is available on serial port */
  {

    // Reads the data from the serial port
    state = bt.read();
    //bt.write(Serial.println("TEST"));
  }
  /*
  Motor rotation:
  A-clockwise
  B-counterclockwise

  Motor translation:
  C-forward
  D-backwards

  Ventil:
  c-Close
  D-Open

  Pump:
  C-Shutoff
  E-Start
*/
 
  if (state == 'A') {
    bt.write(Serial.println("TEST"));
    state = 0;
  } else if (state == 'B') {
    bt.write(Serial.println("TEST2"));
    state = 0;
  } else if (state == 'C') {
      pinMode(ventilPin,OUTPUT);//Ventil
      pinMode(pumpPin,OUTPUT);//Pump
      digitalWrite(pumpPin,LOW);
      digitalWrite(ventilPin,LOW);//
    state = 0;
  } else if (state == 'D') {
    pinMode(ventilPin,OUTPUT);//Ventil
    pinMode(pumpPin,OUTPUT);//Pump
    digitalWrite(pumpPin,LOW);
    digitalWrite(ventilPin,HIGH);
    state = 0;
  }
   else if (state == 'E') {
      pinMode(ventilPin,OUTPUT);//Ventil
      pinMode(pumpPin,OUTPUT);//Pump
      digitalWrite(ventilPin,LOW);
      digitalWrite(pumpPin,HIGH);
    state = 0;
  }
   else if (state == 'F') {
    servo.writeMicroseconds(1500);
    state = 0;
  } 
   else if (state == 'G') {
    servo.writeMicroseconds(1664);
    state = 0;
  }
  else if (state == 'H') {
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(nrOfSteps);
    state = 0;
  }
  else if (state == 'I') {
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps);
    state = 0;
  }
  else if (state == 'J') {
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(nrOfSteps);
    state = 0;
  }
  else if (state == 'K') {
    stepperTranslation.setSpeed(translationSpeed);
    stepperTranslation.step(-nrOfSteps);
    state = 0;
  }
  else if (state == 'L') {
    servo.writeMicroseconds(1664);
    delay(2000);
    servo.writeMicroseconds(1500);
    delay(1000);
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps*2);
    delay(2000);
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(nrOfSteps*4);
    delay(2000);
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps*2);
    delay(2000);
    pinMode(ventilPin,OUTPUT);//Ventil
    pinMode(pumpPin,OUTPUT);//Pump
    digitalWrite(pumpPin,LOW);
    digitalWrite(ventilPin,LOW);
    delay(1000);
    digitalWrite(pumpPin,HIGH);
    delay(3000);
    digitalWrite(pumpPin,LOW);
    delay(1000);
    state=0;
  }
  else if (state == 'M') {
    servo.writeMicroseconds(1664);
    delay(2000);
    servo.writeMicroseconds(1500);
    delay(120000);
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps*2);
    delay(10000);
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(nrOfSteps*4);
    delay(10000);
    stepperRotation.setSpeed(rotationSpeed);
    stepperRotation.step(-nrOfSteps*2);
    delay(10000);
    pinMode(ventilPin,OUTPUT);//Ventil
    pinMode(pumpPin,OUTPUT);//Pump
     digitalWrite(pumpPin,LOW);
    digitalWrite(ventilPin,HIGH);  
    delay(110000);
    digitalWrite(ventilPin,LOW);
    delay(10000);
    digitalWrite(pumpPin,HIGH);
    delay(30000);
    digitalWrite(pumpPin,LOW);
    state=0;
  }
}
