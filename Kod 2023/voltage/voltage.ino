float value=0;
float voltage=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  value = analogRead(A0);
  voltage = (value * 5.0/1023)-0.55;
  Serial.print("Voltage = ");
  Serial.println(voltage);
  delay(500);

}
