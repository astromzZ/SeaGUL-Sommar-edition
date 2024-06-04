#include <Wire.h>
#include "KellerLD.h"

KellerLD Psensor;

void setup() {
  Serial.begin(115200);
  
  Serial.println("Starting");
  
  Wire.begin();

  Psensor.init();
  Psensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  if (Psensor.isInitialized()) {
    Serial.println("Sensor connected.");
  } else {
    Serial.println("Sensor not connected.");
  }
}

void loop() {
  Psensor.read();

  Serial.print("Pressure: "); 
  Serial.print(Psensor.pressure()); 
  Serial.println(" mbar");
  
  Serial.print("Temperature: "); 
  Serial.print(Psensor.temperature()); 
  Serial.println(" deg C");
  
  Serial.print("Depth: "); 
  Serial.print(Psensor.depth()); 
  Serial.println(" m");
  
  Serial.print("Altitude: "); 
  Serial.print(Psensor.altitude()); 
  Serial.println(" m above mean sea level");

  delay(1000);
}