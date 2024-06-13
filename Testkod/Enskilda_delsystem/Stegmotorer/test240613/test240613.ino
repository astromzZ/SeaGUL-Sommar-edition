#include <Arduino.h>
#include <ICM_20948.h>
#include <KellerLD.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include <TSYS01.h>


#define stepsPerRevolution 200
#define STEPS_PER_CM 600
#define STEPS_PER_DEGREE 10 //Ta reda på detta värde

#define TRANSLATION_MIN_POSITION_CM -1
#define TRANSLATION_MAX_POSITION_CM 1
#define TRANSLATIONMOTOR_STEP_SIZE 5

#define ROTATION_MIN_DEGREE -90
#define ROTATION_MAX_DEGREE 90
#define ROTATIONMOTOR_STEP_SIZE 1

#define AD0_VAL 1

#define RPM 186

#define TRAN_DIR_PIN 17 //Tidigare 9
#define TRAN_STEP_PIN 16 //Tidigare 46
#define TRAN_SLEEP_PIN 15 //Tidigare 3

#define ROT_DIR_PIN 9
#define ROT_STEP_PIN 8
#define ROT_SLEEP_PIN 18

#define SDA_PIN 13
#define SCL_PIN 14

#include "DRV8825.h"
#define MODE0 5 //5
#define MODE1 6 //6
#define MODE2 7 //7 
DRV8825 transtepper(stepsPerRevolution, TRAN_DIR_PIN, TRAN_STEP_PIN, TRAN_SLEEP_PIN, MODE0, MODE1, MODE2);
DRV8825 rottranstepper(stepsPerRevolution, ROT_DIR_PIN, ROT_STEP_PIN, ROT_SLEEP_PIN, MODE0, MODE1, MODE2);

#define SOFT_POT_PIN 4 // Pin connected to softpot wiper

const int GRAPH_LENGTH = 40; // Length of line graph

bool glideUp = false;
bool glideDown = false;
bool transmit = true;
float n_dyk = 0;

void setup() {

    transtepper.begin(RPM);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // transtepper.setEnableActiveState(LOW);
    transtepper.enable();

}

void loop() {
    transtepper.setMicrostep(1);  // Set microstep mode to 1:1

    // One complete revolution is 360°
    transtepper.rotate(360);     // forward revolution
    transtepper.rotate(-360);    // reverse revolution

    // One complete revolution is also stepsPerRevolution steps in full step mode
    transtepper.move(stepsPerRevolution);    // forward revolution
    transtepper.move(-stepsPerRevolution);   // reverse revolution

    /*
     * Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
     * Mode 1 is full speed.
     * Mode 32 is 32 microsteps per step.
     * The motor should rotate just as fast (at the set RPM),
     * but movement precision is increased, which may become visually apparent at lower RPMs.
     */
    transtepper.setMicrostep(8);   // Set microstep mode to 1:8

    // In 1:8 microstepping mode, one revolution takes 8 times as many microsteps
    transtepper.move(8 * stepsPerRevolution);    // forward revolution
    transtepper.move(-8 * stepsPerRevolution);   // reverse revolution
    
    // One complete revolution is still 360° regardless of microstepping mode
    // rotate() is easier to use than move() when no need to land on precise microstep position
    transtepper.rotate(360);
    transtepper.rotate(-360);

    delay(5000);

}