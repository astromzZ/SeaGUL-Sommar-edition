#include <AccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
// #include "page.h"  // Include the header file with HTML content
#include <ICM_20948.h>
#include <pgmspace.h>

#define TRAN_DIR_PIN            17 //Translation motor direction pin
#define TRAN_STEP_PIN           16 //Translation motor step pin
#define TRAN_SLEEP_PIN          15 //Trasnlation motor sleep pin
#define ROT_DIR_PIN              9 //Rotation motor direction pin
#define ROT_STEP_PIN             8 //Rotation motor step pin
#define ROT_SLEEP_PIN           18 //Rotation motor sleep pin


AccelStepper translationMotor(AccelStepper::DRIVER, TRAN_STEP_PIN, TRAN_DIR_PIN);

void setup() {
  Serial.begin(115200);
  pinMode(TRAN_SLEEP_PIN, OUTPUT);
  digitalWrite(TRAN_SLEEP_PIN, HIGH);
  translationMotor.enableOutputs();
  // Set the maximum speed in steps per second:
  translationMotor.setMaxSpeed(1000);
  translationMotor.setPinsInverted(true, false, false);
}

void loop() { 
  // Set the current position to 0:
  translationMotor.setCurrentPosition(0);

  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  while(translationMotor.currentPosition() != 800)
  {
    translationMotor.setSpeed(800);
    translationMotor.runSpeed();
  }

  delay(1000);

  // Reset the position to 0:
  translationMotor.setCurrentPosition(0);

  // Run the motor backwards at 600 steps/second until the motor reaches -200 steps (1 revolution):
  while(translationMotor.currentPosition() != -800) 
  {
    translationMotor.setSpeed(-800);
    translationMotor.runSpeed();
  }

  delay(1000);

//   // Reset the position to 0:
//   translationMotor.setCurrentPosition(0);

//   // Run the motor forward at 400 steps/second until the motor reaches 600 steps (3 revolutions):
//   while(translationMotor.currentPosition() != 100)
//   {
//     translationMotor.setSpeed(400);
//     translationMotor.runSpeed();
//   }

//   delay(3000);
}