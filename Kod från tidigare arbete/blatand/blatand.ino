#include <SoftwareSerial.h>
#include <Stepper.h>


/*const int stepsPerRevolution = 200;
Stepper stepperRotation(stepsPerRevolution, 2, 3, 4, 5);
Stepper stepperTranslation(stepsPerRevolution, 9, 8, 7, 6);

int pumpPin=4;
int ventilPin=3;

float value=0;
float voltage=0;

*/

int state = 0;
SoftwareSerial bt(12, 11);
void setup() {
  bt.begin(9600);
  Serial.begin(9600);
  

  /*pinMode(ventilPin, OUTPUT);//Ventil
  digitalWrite(ventilPin,LOW);//Closed
  pinMode(pumpPin,OUTPUT);//Pump
  digitalWrite(pumpPin,LOW);//Closed
  */
}
 

void loop() {
  
  // put your main code here, to run repeatedly:
  /*value = analogRead(A0);
  voltage = (value * 5.0/1023)-0.55;
  //Serial.println(voltage,3);
   */
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
  E-Close
  F-Open

  Pump:
  G-Shutoff
  H-Start
*/
 
  if (state == 'A') {
    bt.write(Serial.println("COOLT"));
    //bt.write(Serial.read());
    state = 0;
  } else if (state == 'B') {
    Serial.println("A2 fungerar");
    state = 0;
  } else if (state == 'C') {
    state = 0;
  } else if (state == 'D') {
    state = 0;
  }
   //bt.write(Serial.println("COOLT4"));
  }
