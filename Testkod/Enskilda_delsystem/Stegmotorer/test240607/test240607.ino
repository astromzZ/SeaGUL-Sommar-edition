/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP, DIR as indicated
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

// Since microSTEP_PINg is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR_PIN 9
#define STEP_PIN 46
//Uncomment line to use enable/disable functionality
#define SLEEP_PIN 3

#define stepsPerRevolution 200

// 2-wire basic config, microSTEP_PINg is hardwired on the driver
// BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

//Uncomment line to use enable/disable functionality
// BasicStepperDriver stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN, SLEEP_PIN);

void setup() {
    // stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);



}

void loop() {
  digitalWrite(SLEEP_PIN, HIGH);
  digitalWrite(DIR_PIN, HIGH);

  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(2000);
  }

  delay(1000);

  // Set the spinning direction counterclockwise:
  digitalWrite(DIR_PIN, LOW);

  // Spin the stepper motor 1 revolution quickly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }

  delay(1000);

  // Set the spinning direction clockwise:
  digitalWrite(DIR_PIN, HIGH);

  // Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }

  delay(1000);

  // Set the spinning direction counterclockwise:
  digitalWrite(DIR_PIN, LOW);

  //Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }

  delay(1000);
}

