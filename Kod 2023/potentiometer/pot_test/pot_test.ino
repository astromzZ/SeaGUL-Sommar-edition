int pumpPin=4;
int ventilPin=3;
int potensPin=A2;
int readVal;
float V2;
float length;

int delayT=250; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ventilPin, OUTPUT);//Ventil
  digitalWrite(ventilPin,LOW);//Closed
  pinMode(pumpPin,OUTPUT);//Pump
  digitalWrite(pumpPin,LOW);//Closed
}

void loop() {
  // put your main code here, to run repeatedly:

readVal=analogRead(potensPin);
V2 = (5/1023.)*readVal;
length = 20 - (20/1023.)*readVal;

Serial.println(length);
delay(delayT);
}
