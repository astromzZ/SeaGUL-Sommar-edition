void setup() {
  // put your setup code here, to run once:
  pinMode(10,OUTPUT);//Ventil
  pinMode(13,OUTPUT);//Pump
  digitalWrite(13,LOW);
}

void loop() {
  digitalWrite(10,HIGH);

  // put your main code here, to run repeatedly:
}
