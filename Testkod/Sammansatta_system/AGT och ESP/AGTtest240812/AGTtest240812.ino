/*
  Artemis Global Tracker
  Example: Simple Tracker
  
  Written by Paul Clark (PaulZC)
  September 7th 2021

  ** Updated for v2.1.0 of the Apollo3 core / Artemis board package **
  ** (At the time of writing, v2.1.1 of the core conatins a feature which makes communication with the u-blox GNSS problematic. Be sure to use v2.1.0) **

  ** Set the Board to "RedBoard Artemis ATP" **
  ** (The Artemis Module does not have a Wire port defined, which prevents the GNSS library from compiling) **

  This example configures the Artemis Iridium Tracker as a simple
  GNSS + Iridium tracker. A 3D fix is taken from the ZOE-M8Q.
  PHT readings are taken from the MS8607. The bus voltage
  is measured. The data is transmitted every INTERVAL minutes
  via Iridium SBD. The Artemis goes into low power mode between transmissions.

  The message is transmitted in text and has the format:
  
  DateTime,Latitude,Longitude,Altitude,Speed,Course,PDOP,Satellites,Pressure,Temperature,Battery,Count
  
  You will need to install version 3.0.5 of the Iridium SBD I2C library
  before this example will run successfully:
  https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library
  (Available through the Arduino Library Manager: search for IridiumSBDi2c)
  
  You will also need to install the Qwiic_PHT_MS8607_Library:
  https://github.com/sparkfun/SparkFun_PHT_MS8607_Arduino_Library
  (Available through the Arduino Library Manager: search for SparkFun MS8607)
  
  You will need to install the SparkFun u-blox library before this example will run successfully:
  https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library
  (Available through the Arduino Library Manager: search for SparkFun u-blox GNSS)
  
  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun!

  Version history:
  August 25th 2021
    Added a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
  August 7th 2021
    Updated for v2.1 of the Apollo3 core
  December 15th, 2020:
    Adding the deep sleep code from OpenLog Artemis.
    Keep RAM powered up during sleep to prevent corruption above 64K.
    Restore busVoltagePin (Analog in) after deep sleep.

*/


// Define how often messages are sent in MINUTES (max 1440)
// This is the _quickest_ messages will be sent. Could be much slower than this depending on:
// capacitor charge time; gnss fix time; Iridium timeout; etc.
// The default value will be overwritten with the one stored in Flash EEPROM - if one exists
// The value can be changed via a Mobile Terminated message: [INTERVAL=nnn]
#define DEF_TXINT 5 // DEFault TX INTerval (Minutes)

// Both the source and destination RockBLOCK serial numbers are stored in flash.
// These can be changed via MT message too: [RBSOURCE=nnnnn] and [RBDESTINATION=nnnnn]
// Set RBDESTINATION to zero to disable message forwarding
// The default values are:
#define DEF_SOURCE    0 // DEFault SOURCE RockBLOCK serial number (the serial number of the 9603N on this tracker)
#define DEF_DEST      0 // DEFault DESTination RockBLOCK (the serial number of the desination RockBLOCK)


// Artemis Tracker pin definitions
#define spiCS1              4  // D4 can be used as an SPI chip select or as a general purpose IO pin
#define geofencePin         10 // Input for the ZOE-M8Q's PIO14 (geofence) pin
#define busVoltagePin       13 // Bus voltage divided by 3 (Analog in)
#define iridiumSleep        17 // Iridium 9603N ON/OFF (sleep) pin: pull high to enable the 9603N
#define iridiumNA           18 // Input for the Iridium 9603N Network Available
#define LED                 19 // White LED
#define iridiumPwrEN        22 // ADM4210 ON: pull high to enable power for the Iridium 9603N
#define gnssEN              26 // GNSS Enable: pull low to enable power for the GNSS (via Q2)
#define gnssBckpBatChgEN    44 // GNSS backup battery charge enable; when set as INPUT = disabled, when OUTPUT+LOW = charging.
#define superCapChgEN       27 // LTC3225 super capacitor charger: pull high to enable the super capacitor charger
#define superCapPGOOD       28 // Input for the LTC3225 super capacitor charger PGOOD signal
#define busVoltageMonEN     34 // Bus voltage monitor enable: pull high to enable bus voltage monitoring (via Q4 and Q3)
#define spiCS2              35 // D35 can be used as an SPI chip select or as a general purpose IO pin
#define iridiumRI           41 // Input for the Iridium 9603N Ring Indicator
#define dropweightPin       7 // Output to release the dropweight
// Make sure you do not have gnssEN and iridiumPwrEN enabled at the same time!
// If you do, bad things might happen to the AS179 RF switch!

//#define noLED // Uncomment this line to disable the White LED
//#define noTX // Uncomment this line to disable the Iridium SBD transmit if you want to test the code without using message credits

// We use Serial1 to communicate with the Iridium modem. Serial1 on the ATP uses pin 24 for TX and 25 for RX. AGT uses the same pins.

#include <EEPROM.h> // Needed for EEPROM

#include "Tracker_EEPROM_Storage.h"


trackerSettings myTrackerSettings; // Define storage for the tracker settings


#include <IridiumSBD.h> //http://librarymanager/All#IridiumSBDI2C
#define DIAGNOSTICS false // Change this to true to see IridiumSBD diagnostics
// Declare the IridiumSBD object (including the sleep (ON/OFF) and Ring Indicator pins)
IridiumSBD modem(Serial1, iridiumSleep, iridiumRI);

#include <Wire.h> // Needed for I2C
const byte PIN_AGTWIRE_SCL = 8;
const byte PIN_AGTWIRE_SDA = 9;
TwoWire agtWire(PIN_AGTWIRE_SDA, PIN_AGTWIRE_SCL); //Create an I2C port using pads 8 (SCL) and 9 (SDA)

#include <SoftwareSerial.h> 
SoftwareSerial mySerial(D5, D6); // RX, TX (SCK, CIPO)

#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <SparkFun_PHT_MS8607_Arduino_Library.h> //http://librarymanager/All#SparkFun_MS8607
MS8607 barometricSensor; //Create an instance of the MS8607 object

#include "RTC.h" //Include RTC library included with the Arduino_Apollo3 core

// Define how often messages are sent in SECONDS
// This is the _quickest_ messages will be sent. Could be much slower than this depending on:
// capacitor charge time; gnss fix time; Iridium timeout; etc.
unsigned long INTERVAL = 15*60; // 5 minutes

// Use this to keep a count of the second alarms from the rtc
volatile unsigned long secondsCount = 0;

// This flag indicates an interval alarm has occurred
volatile bool intervalAlarm = false;

// This flag indicates a wake-up from deep sleep
volatile bool uartWakeUpFlag = false;

// This flag is used to poke the ESP32 to see if it still runs
volatile bool poke = false;

// This flag is used to release the dropweight
volatile bool dropweight = false;

// This flag is used to send a message to land
volatile bool sendmessage = false;

// This flag is used to send the position to land
volatile bool sendposition = false;

// This flag is used in pickup state
volatile bool pickupFlag = false;

// This flag is used to check if GNSS started
volatile bool gnssStarted = false;

// This flag is used to check if GNSS read
volatile bool gnssRead = false;

// This flag is used to check if the super capacitors started
volatile bool superCapStarted = false;

// This flag is used to check if the super capacitors charged
volatile bool superCapCharged = false;

// This flag is used to check if message was sent correctly
volatile bool messageSent = false;

// This flag is used to check if the message was received correctly
volatile bool messageReceived = false;

int signalQuality = -1; // Initiate signal quality to -1

// iterationCounter is incremented each time a transmission is attempted.
// It helps keep track of whether messages are being sent successfully.
// It also indicates if the tracker has been reset (the count will go back to zero).
long iterationCounter = 0;

// More global variables
float  agtVbat = 5.0; // Battery voltage
float  agtLatitude = 0.0; // Latitude in degrees
float  agtLongitude = 0.0; // Longitude in degrees
long   agtAltitude = 0; // Altitude above Median Seal Level in m
float  agtSpeed = 0.0; // Ground speed in m/s
byte   agtSatellites = 0; // Number of satellites (SVs) used in the solution
long   agtCourse = 0; // Course (heading) in degrees
int    agtPDOP = 0;  // Positional Dilution of Precision in m
int    agtYear = 1970; // GNSS Year
byte   agtMonth = 1; // GNSS month
byte   agtDay = 1; // GNSS day
byte   agtHour = 0; // GNSS hours
byte   agtMinute = 0; // GNSS minutes
byte   agtSecond = 0; // GNSS seconds
int    agtMilliseconds = 0; // GNSS milliseconds
float  agtPascals = 0.0; // Atmospheric pressure in Pascals
float  agtTempC = 0.0; // Temperature in Celcius
byte   agtFixType = 0; // GNSS fix type: 0=No fix, 1=Dead reckoning, 2=2D, 3=3D, 4=GNSS+Dead reckoning
bool   agtPGOOD = false; // Flag to indicate if LTC3225 PGOOD is HIGH
int    agtErr; // Error value returned by IridiumSBD.begin
String message; // The message from the ESP32
String IridiumMessage; // The message to be sent via Iridium
String receivedMessageString; // The message received from Iridium

#define VBAT_LOW 2.8 // Minimum voltage for LTC3225

// Timeout after this many _minutes_ when waiting for a 3D GNSS fix
// (UL = unsigned long)
volatile long GNSS_timeout = 5UL;

// Timeout after this many _minutes_ when waiting for the super capacitors to charge
// 1 min should be OK for 1F capacitors at 150mA.
// Charging 10F capacitors at 60mA can take a long time! Could be as much as 10 mins.
#define CHG_timeout 2UL

// Top up the super capacitors for this many _seconds_.
// 10 seconds should be OK for 1F capacitors at 150mA.
// Increase the value for 10F capacitors.
#define TOPUP_timeout 10UL

// Loop Steps - these are used by the switch/case in the main loop
// This structure makes it easy to go from any of the steps directly to zzz when (e.g.) the batteries are low
typedef enum {
  loop_init = 0, // Send the welcome message, check the battery voltage
  pickup,        // Wait for the dropweight to be released and update the position to land every hour
  read_UART,     // Read the pressure and temperature from the MS8607
  zzz,           // Turn everything off and put the processor into deep sleep
  wakeUp         // Wake from deep sleep, restore the processor clock speed
} loop_steps;
loop_steps loop_step = loop_init; // Make sure loop_step is set to loop_init

// RTC alarm Interrupt Service Routine
// Clear the interrupt flag and increment seconds_count
// If INTERVAL has been reached, set the interval_alarm flag and reset seconds_count
// (Always keep ISRs as short as possible, don't do anything clever in them,
//  and always use volatile variables if the main loop needs to access them too.)
extern "C" void am_rtc_isr(void)
{
  // Clear the RTC alarm interrupt
  rtc.clearInterrupt();

  // Increment seconds_count
  secondsCount = secondsCount + 1;

  // Check if intervalAlarm should be set
  if (secondsCount >= INTERVAL)
  {
    intervalAlarm = true;
    secondsCount = 0;
  }
}

// UART interrupt service routine
void uartISR() {
  uartWakeUpFlag = true;
}

// Get the battery (bus) voltage
// Enable the bus voltage monitor
// Read the bus voltage and store it in agtVbat
// Disable the bus voltage monitor to save power
// Converts the analogread into Volts, compensating for
// the voltage divider (/3) and the Apollo voltage reference (2.0V)
// Include a correction factor of 1.09 to correct for the divider impedance
void getVbat()
{
  digitalWrite(busVoltageMonEN, HIGH); // Enable the bus voltage monitor
  //analogReadResolution(14); //Set resolution to 14 bit
  delay(1); // Let the voltage settle
  agtVbat = ((float)analogRead(busVoltagePin)) * 3.0 * 1.09 * 2.0 / 16384.0;
  digitalWrite(busVoltageMonEN, LOW); // Disable the bus voltage monitor
}

// IridiumSBD Callback - this code is called while the 9603N is trying to transmit
bool ISBDCallback()
{
#ifndef noLED
  // Flash the LED at 4 Hz
  if ((millis() / 250) % 2 == 1) {
    digitalWrite(LED, HIGH);
  }
  else {
    digitalWrite(LED, LOW);
  }
#endif

  // Check the battery voltage now we are drawing current for the 9603
  // If voltage is low, stop Iridium send
  getVbat(); // Read the battery (bus) voltage
  if (agtVbat < VBAT_LOW)
  {
    Serial.print(F("*** LOW VOLTAGE (ISBDCallback) "));
    Serial.print(agtVbat,2);
    Serial.println(F("V ***"));
    return false; // Returning false causes IridiumSBD to terminate
  }
  else
  {     
    return true;
  }
}

void gnssON(void) // Enable power for the GNSS
{
  am_hal_gpio_pincfg_t pinCfg = g_AM_HAL_GPIO_OUTPUT; // Begin by making the gnssEN pin an open-drain output
  pinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN;
  pin_config(PinName(gnssEN), pinCfg);
  delay(1);
  
  digitalWrite(gnssEN, LOW); // Enable GNSS power (HIGH = disable; LOW = enable)
}

void gnssOFF(void) // Disable power for the GNSS
{
  am_hal_gpio_pincfg_t pinCfg = g_AM_HAL_GPIO_OUTPUT; // Begin by making the gnssEN pin an open-drain output
  pinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN;
  pin_config(PinName(gnssEN), pinCfg);
  delay(1);
  
  digitalWrite(gnssEN, HIGH); // Disable GNSS power (HIGH = disable; LOW = enable)
}

// Overwrite the IridiumSBD beginSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::beginSerialPort() // Start the serial port connected to the satellite modem
{
  diagprint(F("custom IridiumSBD::beginSerialPort\r\n"));
  
  // Configure the standard ATP pins for UART1 TX and RX - endSerialPort may have disabled the RX pin
  
  am_hal_gpio_pincfg_t pinConfigTx = g_AM_BSP_GPIO_COM_UART_TX;
  pinConfigTx.uFuncSel = AM_HAL_PIN_24_UART1TX;
  pin_config(D24, pinConfigTx);
  
  am_hal_gpio_pincfg_t pinConfigRx = g_AM_BSP_GPIO_COM_UART_RX;
  pinConfigRx.uFuncSel = AM_HAL_PIN_25_UART1RX;
  pinConfigRx.ePullup = AM_HAL_GPIO_PIN_PULLUP_WEAK; // Put a weak pull-up on the Rx pin
  pin_config(D25, pinConfigRx);
  
  Serial1.begin(19200);
}

// Overwrite the IridiumSBD endSerialPort function - a fix for https://github.com/sparkfun/Arduino_Apollo3/issues/423
void IridiumSBD::endSerialPort()
{
  diagprint(F("custom IridiumSBD::endSerialPort\r\n"));
  
  // Disable the Serial1 RX pin to avoid the code hang
  am_hal_gpio_pinconfig(PinName(D25), g_AM_HAL_GPIO_DISABLE);
}

void clearSerialBuffer(SoftwareSerial &serial) {
  while (serial.available() > 0) {
    serial.read();
  }
}

void setAGTWirePullups(uint32_t i2cBusPullUps)
{
  //Change SCL and SDA pull-ups manually using pin_config
  am_hal_gpio_pincfg_t sclPinCfg = g_AM_BSP_GPIO_IOM1_SCL;
  am_hal_gpio_pincfg_t sdaPinCfg = g_AM_BSP_GPIO_IOM1_SDA;

  if (i2cBusPullUps == 0)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE; // No pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE;
  }
  else if (i2cBusPullUps == 1)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K; // Use 1K5 pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  }
  else if (i2cBusPullUps == 6)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K; // Use 6K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
  }
  else if (i2cBusPullUps == 12)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K; // Use 12K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K;
  }
  else
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K; // Use 24K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K;
  }

  pin_config(PinName(PIN_AGTWIRE_SCL), sclPinCfg);
  pin_config(PinName(PIN_AGTWIRE_SDA), sdaPinCfg);
}

void start_GNSS () {

  Serial.println(F("Powering up the GNSS..."));
  
  gnssON(); // Enable power for the GNSS

  pinMode(gnssBckpBatChgEN, OUTPUT); // GNSS backup batttery charge control; output + low = enable charging
  digitalWrite(gnssBckpBatChgEN, LOW);
  //pinMode(gnssBckpBatChgEN, INPUT); // GNSS backup batttery charge control; input = disable charging

  delay(2000); // Give it time to power up

  agtWire.begin(); // Set up the I2C pins
  agtWire.setClock(100000); // Use 100kHz for best performance
  setAGTWirePullups(0); // Remove the pull-ups from the I2C pins (internal to the Artemis) for best performance
  
  
    if (myGNSS.begin(agtWire) == false) //Connect to the u-blox module using agtWire port
    {
        // If we were unable to connect to the ZOE-M8Q:
        
        // Send a warning message
        Serial.println(F("*** Ublox GNSS not detected at default I2C address ***"));
        
        // Set the lat, long etc. to default values
        agtLatitude = 0.0; // Latitude in degrees
        agtLongitude = 0.0; // Longitude in degrees
        agtAltitude = 0; // Altitude above Median Seal Level in m
        agtSpeed = 0.0; // Ground speed in m/s
        agtSatellites = 0; // Number of satellites (SVs) used in the solution
        agtCourse = 0; // Course (heading) in degrees
        agtPDOP = 0;  // Positional Dilution of Precision in m
        agtYear = 1970; // GNSS Year
        agtMonth = 1; // GNSS month
        agtDay = 1; // GNSS day
        agtHour = 0; // GNSS hours
        agtMinute = 0; // GNSS minutes
        agtSecond = 0; // GNSS seconds
        agtMilliseconds = 0; // GNSS milliseconds

        // Power down the GNSS
        gnssOFF(); // Disable power for the GNSS

    }

    else { // If the GNSS started up OK
        
        //myGNSS.enableDebugging(); // Enable debug messages
        myGNSS.setI2COutput(COM_TYPE_UBX); // Limit I2C output to UBX (disable the NMEA noise)

        // If we are going to change the dynamic platform model, let's do it here.
        // Possible values are:
        // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
        if (myGNSS.setDynamicModel(DYN_MODEL_PORTABLE) == false)
        {
        Serial.println(F("*** Warning: setDynamicModel may have failed ***"));
        }
        else
        {
        Serial.println(F("Dynamic Model updated"));
        }
        gnssStarted = true;
    }
}

// ************************************************************************************************
// Read a fix from the ZOE-M8Q
void read_GNSS() {

  Serial.println(F("Waiting for a 3D GNSS fix..."));

  agtFixType = 0; // Clear the fix type
  
  // Look for GNSS signal for up to GNSS_timeout minutes
  for (unsigned long tnow = millis(); (agtFixType != 3) && ((millis() - tnow) < (GNSS_timeout * 60UL * 1000UL));)
  {
  
    agtFixType = myGNSS.getFixType(); // Get the GNSS fix type
    
    // Check battery voltage now we are drawing current for the GNSS
    // If voltage is low, stop looking for GNSS and go to sleep

  }
  if (agtFixType == 3) // Check if we got a valid 3D fix
  {
    // Get the time and position etc.
    // Get the time first to hopefully avoid second roll-over problems
    agtMilliseconds = myGNSS.getMillisecond();
    agtSecond = myGNSS.getSecond();
    agtMinute = myGNSS.getMinute();
    agtHour = myGNSS.getHour();
    agtDay = myGNSS.getDay();
    agtMonth = myGNSS.getMonth();
    agtYear = myGNSS.getYear(); // Get the year
    agtLatitude = (float)myGNSS.getLatitude() / 10000000.0; // Get the latitude in degrees
    agtLongitude = (float)myGNSS.getLongitude() / 10000000.0; // Get the longitude in degrees
    agtAltitude = myGNSS.getAltitudeMSL() / 1000; // Get the altitude above Mean Sea Level in m
    agtSpeed = (float)myGNSS.getGroundSpeed() / 1000.0; // Get the ground speed in m/s
    agtSatellites = myGNSS.getSIV(); // Get the number of satellites used in the fix
    agtCourse = myGNSS.getHeading() / 10000000; // Get the heading in degrees
    agtPDOP = myGNSS.getPDOP() / 100; // Get the PDOP in m

    Serial.println(F("A 3D fix was found!"));
    Serial.print(F("Latitude (degrees): ")); Serial.println(agtLatitude, 6);
    Serial.print(F("Longitude (degrees): ")); Serial.println(agtLongitude, 6);
    Serial.print(F("Altitude (m): ")); Serial.println(agtAltitude);
    gnssRead = true;

  }
  
  else
  {
    // We didn't get a 3D fix so
    // set the lat, long etc. to default values
    agtLatitude = 0.0; // Latitude in degrees
    agtLongitude = 0.0; // Longitude in degrees
    agtAltitude = 0; // Altitude above Median Seal Level in m
    agtSpeed = 0.0; // Ground speed in m/s
    agtSatellites = 0; // Number of satellites (SVs) used in the solution
    agtCourse = 0; // Course (heading) in degrees
    agtPDOP = 0;  // Positional Dilution of Precision in m
    agtYear = 1970; // GNSS Year
    agtMonth = 1; // GNSS month
    agtDay = 1; // GNSS day
    agtHour = 0; // GNSS hours
    agtMinute = 0; // GNSS minutes
    agtSecond = 0; // GNSS seconds
    agtMilliseconds = 0; // GNSS milliseconds

    Serial.println(F("A 3D fix was NOT found!"));
    Serial.println(F("Using default values..."));
    }
  // Power down the GNSS
  Serial.println(F("Powering down the GNSS..."));
  gnssOFF(); // Disable power for the GNSS
}

// Start the LTC3225 supercapacitor charger
void start_LTC3225() {

  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  digitalWrite(superCapChgEN, HIGH); // Enable the super capacitor charger

  Serial.println(F("Waiting for supercapacitors to charge..."));
  delay(2000);

  agtPGOOD = false; // Flag to show if PGOOD is HIGH
  
  // Wait for PGOOD to go HIGH for up to CHG_timeout minutes
  for (unsigned long tnow = millis(); (!agtPGOOD) && (millis() - tnow < CHG_timeout * 60UL * 1000UL);)
  {
  
    agtPGOOD = digitalRead(superCapPGOOD); // Read the PGOOD pin
    delay(100); 
  }


    if (agtPGOOD)
    {
        // If the capacitors charged OK
        Serial.println(F("Supercapacitors charged!"));
        superCapStarted = true;
    }

    else
    {
        // The super capacitors did not charge so power down and go to sleep
        Serial.println(F("*** Supercapacitors failed to charge ***"));

    }
  
}

    // ************************************************************************************************
    // Give the super capacitors some extra time to charge
void wait_LTC3225() {

  Serial.println(F("Giving the supercapacitors extra time to charge..."));

  // Wait for TOPUP_timeout seconds, keep checking PGOOD and the battery voltage
  for (unsigned long tnow = millis(); (millis() - tnow) < (TOPUP_timeout * 1000UL); )
  {
  
    // Check battery voltage now we are drawing current for the LTC3225
    // If voltage is low, stop charging and go to sleep


    delay(100); // Let's not pound the bus voltage monitor too hard!

  }

  // If voltage is low then go straight to sleep


  if (agtPGOOD)
  {
    // If the capacitors are still charged OK
    Serial.println(F("Supercapacitors charged!"));
    superCapCharged = true; 
    
  }

  else
  {
    // The super capacitors did not charge so power down and go to sleep
    Serial.println(F("*** Supercapacitors failed to hold charge in wait_LTC3225 ***"));

  }
}
    // ************************************************************************************************
    // Enable the 9603N and attempt to send a message
void send_9603() {

  // Enable power for the 9603N
  Serial.println(F("Enabling 9603N power..."));
  digitalWrite(iridiumPwrEN, HIGH); // Enable Iridium Power
  delay(1000);

  // Relax timing constraints waiting for the supercap to recharge.
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation
  // Also begin the serial port connected to the satellite modem via IridiumSBD::beginSerialPort
  Serial.println(F("Starting modem..."));
  agtErr = modem.begin();

  // Check if the modem started correctly
  if (agtErr != ISBD_SUCCESS)
  {
    // If the modem failed to start, disable it and go to sleep
    Serial.print(F("*** modem.begin failed with error "));
    Serial.print(agtErr);
    Serial.println(F(" ***"));
  }

  else
  {
    // The modem started OK so let's try to send the message
    char outBuffer[120]; // Use outBuffer to store the message. Always try to keep message short to avoid wasting credits
    char iridiumMessageBuffer[120]; // Use iridiumMessageBuffer to store the message. Always try to keep message short to avoid wasting credits
    IridiumMessage.toCharArray(iridiumMessageBuffer, sizeof(iridiumMessageBuffer)); // Convert to C-style string

    // Apollo3 v2.1 does not support printf or sprintf correctly. We need to manually add preceeding zeros
    // and convert floats to strings

    // Convert the floating point values into strings
    char latStr[15]; // latitude string
    ftoa(agtLatitude,latStr,6,15);
    char lonStr[15]; // longitude string
    ftoa(agtLongitude,lonStr,6,15);
    char altStr[15]; // altitude string
    ftoa(agtAltitude,altStr,2,15);
    char vbatStr[6]; // battery voltage string
    ftoa(agtVbat,vbatStr,2,6);
    char speedStr[8]; // speed string
    ftoa(agtSpeed,speedStr,2,8);
    char pressureStr[9]; // pressure string
    ftoa(agtPascals,pressureStr,0,9);
    char temperatureStr[10]; // temperature string
    ftoa(agtTempC,temperatureStr,1,10);

    // Convert the date and time into strings
    char gnssDay[3];
    char gnssMonth[3];
    if (agtDay < 10)
      sprintf(gnssDay, "0%d", agtDay);
    else
      sprintf(gnssDay, "%d", agtDay);
    if (agtMonth < 10)
      sprintf(gnssMonth, "0%d", agtMonth);
    else
      sprintf(gnssMonth, "%d", agtMonth);
  
    char gnssHour[3];
    char gnssMin[3];
    char gnssSec[3];
    if (agtHour < 10)
      sprintf(gnssHour, "0%d", agtHour);
    else
      sprintf(gnssHour, "%d", agtHour);
    if (agtMinute < 10)
      sprintf(gnssMin, "0%d", agtMinute);
    else
      sprintf(gnssMin, "%d", agtMinute);
    if (agtSecond < 10)
      sprintf(gnssSec, "0%d", agtSecond);
    else
      sprintf(gnssSec, "%d", agtSecond);

    char destStr[8];
    if (myTrackerSettings.DEST < 10)
      sprintf(destStr, "000000%d", myTrackerSettings.DEST);
    else if (myTrackerSettings.DEST < 100)
      sprintf(destStr, "00000%d", myTrackerSettings.DEST);
    else if (myTrackerSettings.DEST < 1000)
      sprintf(destStr, "0000%d", myTrackerSettings.DEST);
    else if (myTrackerSettings.DEST < 10000)
      sprintf(destStr, "000%d", myTrackerSettings.DEST);
    else if (myTrackerSettings.DEST < 100000)
      sprintf(destStr, "00%d", myTrackerSettings.DEST);
    else if (myTrackerSettings.DEST < 1000000)
      sprintf(destStr, "0%d", myTrackerSettings.DEST);
    else
      sprintf(destStr, "%d", myTrackerSettings.DEST);
    
    char sourceStr[8];
    if (myTrackerSettings.SOURCE < 10)
      sprintf(sourceStr, "000000%d", myTrackerSettings.SOURCE);
    else if (myTrackerSettings.SOURCE < 100)
      sprintf(sourceStr, "00000%d", myTrackerSettings.SOURCE);
    else if (myTrackerSettings.SOURCE < 1000)
      sprintf(sourceStr, "0000%d", myTrackerSettings.SOURCE);
    else if (myTrackerSettings.SOURCE < 10000)
      sprintf(sourceStr, "000%d", myTrackerSettings.SOURCE);
    else if (myTrackerSettings.SOURCE < 100000)
      sprintf(sourceStr, "00%d", myTrackerSettings.SOURCE);
    else if (myTrackerSettings.SOURCE < 1000000)
      sprintf(sourceStr, "0%d", myTrackerSettings.SOURCE);
    else
      sprintf(sourceStr, "%d", myTrackerSettings.SOURCE);
    
    // Assemble the message using sprintf
    if (myTrackerSettings.DEST > 0) {
      sprintf(outBuffer, "RB%s,%s,%s,%s,RB%s", 
        latStr, lonStr, iridiumMessageBuffer);
    }
    else {
      sprintf(outBuffer, "%s,%s,%s", 
        latStr, lonStr, iridiumMessageBuffer);
    }

    // Send the message
    Serial.print(F("Transmitting message '"));
    Serial.print(outBuffer);
    Serial.println(F("'"));

#ifndef noTX
    agtErr = modem.sendSBDText(outBuffer); // This could take many seconds to complete and will call ISBDCallback() periodically
#else
    agtErr = ISBD_SUCCESS; // Fake success
    mtBufferSize = 0;
#endif

    // Check if the message sent OK
    if (agtErr != ISBD_SUCCESS)
    {
      Serial.print(F("Transmission failed with error code "));
      Serial.println(agtErr);
#ifndef noLED
      // Turn on LED to indicate failed send
      digitalWrite(LED, HIGH);
#endif
    }
    else
    {
      Serial.println(F(">>> Message sent! <<<"));
      messageSent = true;
#ifndef noLED
      // Flash LED rapidly to indicate successful send
      for (int i = 0; i < 10; i++)
      {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
      }
#endif
      }
  }
  // Clear the Mobile Originated message buffer
    Serial.println(F("Clearing the MO buffer."));
    agtErr = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
    if (agtErr != ISBD_SUCCESS) {
        Serial.print(F("*** modem.clearBuffers failed with error "));
        Serial.print(agtErr);
        Serial.println(F(" ***"));
    }

    // Power down the modem
    Serial.println(F("Putting the 9603N to sleep."));
    agtErr = modem.sleep();
    if (agtErr != ISBD_SUCCESS) {
        Serial.print(F("*** modem.sleep failed with error "));
        Serial.print(agtErr);
        Serial.println(F(" ***"));
    }
}

void receive_9603() {
  uint8_t mt_Buffer[200]; // Buffer to store received message
  size_t mtBufferSize = sizeof(mt_Buffer); // Size of MT buffer

  // Enable power for the 9603N
  Serial.println(F("Enabling 9603N power..."));
  digitalWrite(iridiumPwrEN, HIGH); // Enable Iridium Power
  delay(1000);

  // Relax timing constraints waiting for the supercap to recharge.
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

  // Begin satellite modem operation
  // Also begin the serial port connected to the satellite modem via IridiumSBD::beginSerialPort
  Serial.println(F("Starting modem..."));
  agtErr = modem.begin();

  // Check if the modem started correctly
  if (agtErr != ISBD_SUCCESS)
  {
    // If the modem failed to start, disable it and go to sleep
    Serial.print(F("*** modem.begin failed with error "));
    Serial.print(agtErr);
    Serial.println(F(" ***"));
  }
  else
  {
    
    
    // Send the message
    Serial.print(F("Transmitting NULL message to get received message"));
    

#ifndef noTX
    agtErr = modem.sendReceiveSBDText(NULL, mt_Buffer, mtBufferSize); // This could take many seconds to complete and will call ISBDCallback() periodically
#else
    agtErr = ISBD_SUCCESS; // Fake success
    mtBufferSize = 0;
#endif

    // Check if the message sent OK
    if (agtErr != ISBD_SUCCESS)
    {
      Serial.print(F("Transmission failed with error code "));
      Serial.println(agtErr);
#ifndef noLED
      // Turn on LED to indicate failed send
      digitalWrite(LED, HIGH);
#endif
    }
    else
    {
      Serial.println(F(">>> Message sent! <<<"));
      messageReceived = true;
#ifndef noLED
      // Flash LED rapidly to indicate successful send
      for (int i = 0; i < 10; i++)
      {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);
      }
#endif
      }
  }
  if (mtBufferSize > 0) { // Was an MT message received?
    // Check message content
    mt_Buffer[mtBufferSize] = 0; // Make sure message is NULL terminated
    String mt_str = String((char *)mt_Buffer); // Convert message into a String
    Serial.print(F("Received a MT message: ")); Serial.println(mt_str);
    receivedMessageString = mt_str; // Save the message for later processing

  }
  // Clear the Mobile Originated message buffer
  Serial.println(F("Clearing the MO buffer."));
  agtErr = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
  if (agtErr != ISBD_SUCCESS)
  {
    Serial.print(F("*** modem.clearBuffers failed with error "));
    Serial.print(agtErr);
    Serial.println(F(" ***"));
  }

  // Power down the modem
  Serial.println(F("Putting the 9603N to sleep."));
  agtErr = modem.sleep();
  if (agtErr != ISBD_SUCCESS) {
      Serial.print(F("*** modem.sleep failed with error "));
      Serial.print(agtErr);
      Serial.println(F(" ***"));
  }
}



void setup()
{
  // Let's begin by setting up the I/O pins

  Serial.begin(115200);
  mySerial.begin(9600);

  // while (!Serial) // Wait for the user to open the serial monitor
  //   ;
  delay(100);
  Serial.println();
  Serial.println();
  Serial.println(F("Artemis Global Tracker"));
  Serial.println(F("Example: Communication with ESP2"));
  Serial.println();

  //empty the serial buffer
  while(Serial.available() > 0)
    Serial.read();

  //wait for the user to press any key before beginning
  Serial.println(F("Please check that the Serial Monitor is set to 115200 Baud"));
  Serial.println(F("and that the line ending is set to Newline."));
  Serial.println(F("Then click Send to start the example."));
  Serial.println();

  // while(Serial.available() == 0)
  //   ;
  delay(1000*30); // Wait for 30 seconds to allow the user to open the serial monitor
   
  pinMode(LED, OUTPUT); // Make the LED pin an output

  gnssOFF(); // Disable power for the GNSS
  pinMode(gnssBckpBatChgEN, INPUT); // GNSS backup batttery charge control; input = disable charging; output+low=charging. 
  pinMode(geofencePin, INPUT); // Configure the geofence pin as an input

  pinMode(iridiumPwrEN, OUTPUT); // Configure the Iridium Power Pin (connected to the ADM4210 ON pin)
  digitalWrite(iridiumPwrEN, LOW); // Disable Iridium Power (HIGH = enable; LOW = disable)
  pinMode(superCapChgEN, OUTPUT); // Configure the super capacitor charger enable pin (connected to LTC3225 !SHDN)
  digitalWrite(superCapChgEN, LOW); // Disable the super capacitor charger (HIGH = enable; LOW = disable)
  pinMode(iridiumSleep, OUTPUT); // Iridium 9603N On/Off (Sleep) pin
  digitalWrite(iridiumSleep, LOW); // Put the Iridium 9603N to sleep (HIGH = on; LOW = off/sleep)
  pinMode(iridiumRI, INPUT); // Configure the Iridium Ring Indicator as an input
  pinMode(iridiumNA, INPUT); // Configure the Iridium Network Available as an input
  pinMode(superCapPGOOD, INPUT); // Configure the super capacitor charger PGOOD input
  pinMode(D4, OUTPUT); // D4 can be used as a handshake pin for the ESP communication
  pinMode(dropweightPin, OUTPUT); // Configure the dropweight pin as an output

  attachInterrupt(35, uartISR, RISING); // Adjust the interrupt mode as needed, pin 35 for the Artemis Global Tracker

  // Make sure the Serial1 RX pin is disabled to prevent the power-on glitch on the modem's RX(OUT) pin
  // causing problems with v2.1.0 of the Apollo3 core. Requires v3.0.5 of IridiumSBDi2c.
  modem.endSerialPort();

  pinMode(busVoltageMonEN, OUTPUT); // Make the Bus Voltage Monitor Enable an output
  digitalWrite(busVoltageMonEN, LOW); // Set it low to disable the measurementesolsave power
  analogReadResolution(14); //Set resolution to 14 bit

  // Adjust Iridium timeout settings
  modem.adjustStartupTimeout(1000); // Set the startup timeout to 1000 seconds

  // Initialise the globals
  iterationCounter = 0; // Make sure iterationCounter is set to zero (indicating a reset)
  loop_step = loop_init; // Make sure loop_step is set to loop_init
  secondsCount = 0; // Make sure seconds_count is reset
  intervalAlarm = false; // Make sure the interval alarm flag is clear

  // Set up the RTC for 1 second interrupts
  /*
    0: Alarm interrupt disabled
    1: Alarm match every year   (hundredths, seconds, minutes, hour, day, month)
    2: Alarm match every month  (hundredths, seconds, minutes, hours, day)
    3: Alarm match every week   (hundredths, seconds, minutes, hours, weekday)
    4: Alarm match every day    (hundredths, seconds, minute, hours)
    5: Alarm match every hour   (hundredths, seconds, minutes)
    6: Alarm match every minute (hundredths, seconds)
    7: Alarm match every second (hundredths)
  */
  rtc.setAlarmMode(7); // Set the RTC alarm mode
  rtc.attachInterrupt(); // Attach RTC alarm interrupt  
}


void loop()
{
  // loop is one large switch/case that controls the sequencing of the code
  switch (loop_step) {

    // ************************************************************************************************
    // Initialise things
    case loop_init:
    
      // Start the console serial port and send the welcome message


      // Setup the IridiumSBD
      modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // Change power profile to "low current"

      // getVbat(); // Get the battery (bus) voltage
      // if (agtVbat < VBAT_LOW) {
      //   Serial.print(F("*** LOW VOLTAGE (init) "));
      //   Serial.print(agtVbat,2);
      //   Serial.println(F(" ***"));
      //   // loop_step = zzz; // Go to sleep
      // } else if (poke == true) {
      //   Serial.println(F("Poke the ESP32 to see if it still runs"));
      //   digitalWrite(D4, HIGH); // Set D4 high to wake the ESP32

      //   delay(500); // Wait to see if we get an answer

      //   digitalWrite(D4, LOW); 

      //   if (uartWakeUpFlag == true) {
      //     Serial.println(F("ESP32 is awake"));
      //     uartWakeUpFlag = false;
      //     poke = false;
      //     loop_step = zzz; // Go to sleep
      //   } else {
      //     Serial.println(F("ESP32 is not awake"));
      //     poke = false;
      //     dropweight = true;
      //     loop_step = read_UART; // Send that the ESP32 is not awake and that the dropweight is released
      //   }
      // }
      // else {
      //   loop_step = read_UART; 
      // }

      if (poke && !uartWakeUpFlag) {
        Serial.println(F("Poke the ESP32 to see if it still runs"));
        digitalWrite(D4, HIGH); // Set D4 high to wake the ESP32

        delay(500); // Wait to see if we get an answer

        digitalWrite(D4, LOW); 

        if (uartWakeUpFlag) {
          Serial.println(F("ESP32 is awake"));
          uartWakeUpFlag = false;
          poke = false;
          loop_step = zzz; // Go to sleep
        } else {
          Serial.println(F("ESP32 is not awake"));
          mySerial.println("dropweight"); // Send that the dropweight is released
          poke = false;
          digitalWrite(dropweightPin, HIGH); // Release the dropweight
          delay(1000);
          digitalWrite(dropweightPin, LOW); // Reset the dropweight
          dropweight = true;
          loop_step = pickup; // Send that the ESP32 is not awake and that the dropweight is released
        }
      } else {
        loop_step = read_UART;
      }
      
      break; // End of case loop_init
    // ************************************************************************************************
    // Pickup case, wait for pickup and update the position to land every hour
    case pickup:
      pickupFlag = true; // Set the pickup flag
      INTERVAL = 60*60; // 1 hour
      if (dropweight) {
        IridiumMessage = "dropweight, pickup"; // Send that the dropweight is released, awaiting pickup
        Serial.println(IridiumMessage);
      }
      else {
        IridiumMessage = "pickup"; // Send that the glider is awaiting pickup
        Serial.println(IridiumMessage);
      }
      start_GNSS();
      if (gnssStarted) {
        read_GNSS();
        gnssStarted = false;
      }
      if (gnssRead) {
        start_LTC3225();
        gnssRead = false;
      }
      if (superCapStarted) {
        wait_LTC3225();
        superCapStarted = false;
      }
      // if (superCapCharged) {
      //   send_9603();
      //   superCapCharged = false;
      // }
      if (superCapCharged) {
        send_9603();
        if (messageSent) {
          Serial.println(IridiumMessage);
          messageSent = false;
          IridiumMessage = "";
        }
        else {
          Serial.println("No message Sent");
        }
      }
      loop_step = zzz;
      
      break;
    // ************************************************************************************************
    // Power up the GNSS (ZOE-M8Q)
    

    // ************************************************************************************************

    case read_UART:

      if (uartWakeUpFlag) {
        Serial.println(F("Woken up by UART interrupt"));

        Serial.println("Ready to recieve message!");

        mySerial.flush();
        digitalWrite(D4, HIGH); // Set D4 high to wake the ESP32

        // mySerial.println("r"); //request message

        //Wait for message
        while (mySerial.available() == 0) {
          delay(100);
          // Serial.println("Waiting for message...");
        } 

        //Read message
        if (mySerial.available()) {
          message = mySerial.readStringUntil(']');
          Serial.println("Received: " + message);
        }

        if (message.startsWith("m")) //The ESP wants to transmit a message
        {
          Serial.println("ESP wants to transmit a message");
          IridiumMessage = message;
          uartWakeUpFlag = false;
          delay(1000);
          digitalWrite(D4, LOW); // Set D4 low to signal that the message has been recieved
          clearSerialBuffer(mySerial);
          start_GNSS();
          if (gnssStarted) {
            read_GNSS();
            gnssStarted = false;
          }
          if (gnssRead) {
            send_9603();
            mySerial.println(agtLatitude, agtLongitude);
            gnssRead = false;
          }
          if (messageSent) {
            receive_9603();
            messageSent = false;
            mySerial.println(receivedMessageString + ']');
            receivedMessageString = "";
          }
          loop_step = zzz;
        }
        else if (message == "s") //The ESP wants the AGT to go to sleep
        {
          Serial.println("ESP wants you to go to sleep");
          uartWakeUpFlag = false;
          delay(1000);
          digitalWrite(D4, LOW); // Set D4 low to signal that the message has been recieved
          clearSerialBuffer(mySerial);
          loop_step = zzz;
        }
        else if (message == "g") {
          Serial.println("ESP wants to get GNSS fix");
          uartWakeUpFlag = false;
          delay(1000);
          digitalWrite(D4, LOW); // Set D4 low to signal that the message has been recieved
          clearSerialBuffer(mySerial);
          start_GNSS();
          if (gnssStarted) {
            read_GNSS();
            gnssStarted = false;
          }
          if (gnssRead) {
            mySerial.println(agtLatitude, agtLongitude);
            gnssRead = false;
          }
          else {
            mySerial.println("No GNSS fix");
          }
          loop_step = zzz;
        }
        else if (message == "r") {
          Serial.println("ESP wants to get message");
          // mySerial.println("Hi! This is my message");
          uartWakeUpFlag = false;
          delay(1000);
          receive_9603();
          digitalWrite(D4, LOW); // Set D4 low to signal that the message has been recieved
          clearSerialBuffer(mySerial);
          if (messageReceived) {
            mySerial.println(receivedMessageString);
            messageReceived = false;
            receivedMessageString = "";
          }
          else (mySerial.println("No message received"));
          loop_step = zzz;
        }
        else if (message == "p") {
          Serial.println("ESP wants to transmit dropweight released");
          uartWakeUpFlag = false;
          dropweight = true;
          delay(1000);
          digitalWrite(D4, LOW); // Set D4 low to signal that the message has been recieved
          clearSerialBuffer(mySerial);
          mySerial.println("dropweight");
          loop_step = pickup;
        }
        else
        {
          mySerial.println("Unknown message, going to sleep");
          delay(1000);
          digitalWrite(D4, LOW); // Set D4 low to signal that the message has been recieved
          clearSerialBuffer(mySerial);
          uartWakeUpFlag = false;
          loop_step = zzz;
        }
      } else if (dropweight) {
        Serial.println("Dropweight released, transmit to land");
        loop_step = pickup;
      } else {
        loop_step = zzz; //Go to sleep if the ESP doesn't wan't to do anything
      }

      // loop_step = zzz;

      break; // End of case read_UART

    // ************************************************************************************************
      
      
    // ************************************************************************************************
    // Go to sleep
    case zzz:
      {
        Serial.println(F("Getting ready to put the Apollo3 into deep sleep..."));
  
        // Power down the GNSS
        Serial.println(F("Powering down the GNSS..."));
        gnssOFF(); // Disable power for the GNSS

        // Make sure the Serial1 RX pin is disabled
        modem.endSerialPort();
  
        // Disable 9603N power
        Serial.println(F("Disabling 9603N power..."));
        digitalWrite(iridiumSleep, LOW); // Disable 9603N via its ON/OFF pin (modem.sleep should have done this already)
        delay(1000);
        digitalWrite(iridiumPwrEN, LOW); // Disable Iridium Power
        delay(1000);
      
        // Disable the supercapacitor charger
        Serial.println(F("Disabling the supercapacitor charger..."));
        digitalWrite(superCapChgEN, LOW); // Disable the super capacitor charger
  
        // Close the Iridium serial port
        Serial1.end();
  
        // Close the I2C port
        //agtWire.end(); //DO NOT Power down I2C - causes badness with v2.1 of the core: https://github.com/sparkfun/Arduino_Apollo3/issues/412
  
        digitalWrite(busVoltageMonEN, LOW); // Disable the bus voltage monitor
  
        digitalWrite(LED, LOW); // Disable the LED
  
        // Close and detach the serial console
        Serial.println(F("Going into deep sleep until next INTERVAL..."));
        Serial.flush(); //Finish any prints

        clearSerialBuffer(mySerial);
        // mySerial.end(); // Close the serial console
  
        // Code taken (mostly) from Apollo3 Example6_Low_Power_Alarm
        
        // Disable ADC
        powerControlADC(false);
      
        // Force the peripherals off
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0); // SPI
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1); // agtWire I2C
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4); // Qwiic I2C
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
        //am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0); // Leave UART0 on to avoid printing erroneous characters to Serial
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1); // Serial1
      
        // // Disable all unused pins - including: SCL (8), SDA (9), UART0 TX (48) and RX (49) and UART1 TX (24) and RX (25)
        const int pinsToDisable[] = {0,1,2,8,9,11,12,14,15,16,20,21,24,25,29,31,32,33,36,37,38,42,43,44,45,48,49,-1};
        for (int x = 0; pinsToDisable[x] >= 0; x++)
        {
          pin_config(PinName(pinsToDisable[x]), g_AM_HAL_GPIO_DISABLE);
        }
      
        // //Power down CACHE, flashand SRAM
        am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); // Power down all flash and cache
        am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_384K); // Retain all SRAM (0.6 uA)
        
        // // Keep the 32kHz clock running for RTC
        // am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
        // am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);
      
        // This while loop keeps the processor asleep until INTERVAL seconds have passed
        while (!intervalAlarm && !uartWakeUpFlag)  // Use && instead of ||
        {
          // // Go to Deep Sleep.
          am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        }

        if (intervalAlarm || uartWakeUpFlag) {
          if (intervalAlarm) {
            Serial.println(F("Woken up by RTC alarm"));
            intervalAlarm = false; // Clear the alarm flag now
            poke = true; // Set the poke flag to true
          } else {
            Serial.println(F("Woken up by UART interrupt"));
            // uartWakeUpFlag = false; // Clear the UART flag now
          }
        }

        // Wake up!
        loop_step = wakeUp;
      }
      break; // End of case zzz
      
    // ************************************************************************************************
    // Wake from sleep
    case wakeUp:

      Serial.println(F("Waking up from deep sleep..."));
      delay(3000); // Wait for the Artemis to wake up

      // Code taken (mostly) from Apollo3 Example6_Low_Power_Alarm
      
      // Go back to using the main clock
      // am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
      // am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);
      // mySerial.begin(9600);
      // clearSerialBuffer(mySerial);
      // attachInterrupt(35, uartISR, RISING);
      // // Power up SRAM, turn on entire Flash
      am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_MAX);
    
      // // Renable UART0 pins: TX (48) and RX (49)
      pin_config(PinName(48), g_AM_BSP_GPIO_COM_UART_TX);
      pin_config(PinName(49), g_AM_BSP_GPIO_COM_UART_RX);
    
      // Do not renable the UART1 pins here as the modem is still powered off. Let modem.begin do it via beginSerialPort.
    
      // Enable ADC
      powerControlADC(true);

      // Do it all again!
      loop_step = loop_init;

      break; // End of case wake
    
  } // End of switch (loop_step)
} // End of loop()

// IridiumSBD Callbacks
#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif

static uint64_t divu64_10(uint64_t ui64Val)
{
    uint64_t q64, r64;
    uint32_t q32, r32, ui32Val;

    //
    // If a 32-bit value, use the more optimal 32-bit routine.
    //
    if ( ui64Val >> 32 )
    {
        q64 = (ui64Val>>1) + (ui64Val>>2);
        q64 += (q64 >> 4);
        q64 += (q64 >> 8);
        q64 += (q64 >> 16);
        q64 += (q64 >> 32);
        q64 >>= 3;
        r64 = ui64Val - q64*10;
        return q64 + ((r64 + 6) >> 4);
    }
    else
    {
        ui32Val = (uint32_t)(ui64Val & 0xffffffff);
        q32 = (ui32Val>>1) + (ui32Val>>2);
        q32 += (q32 >> 4);
        q32 += (q32 >> 8);
        q32 += (q32 >> 16);
        q32 >>= 3;
        r32 = ui32Val - q32*10;
        return (uint64_t)(q32 + ((r32 + 6) >> 4));
    }
}

//*****************************************************************************
//
// Converts ui64Val to a string.
// Note: pcBuf[] must be sized for a minimum of 21 characters.
//
// Returns the number of decimal digits in the string.
//
// NOTE: If pcBuf is NULL, will compute a return ui64Val only (no chars
// written).
//
//*****************************************************************************
static int uint64_to_str(uint64_t ui64Val, char *pcBuf)
{
    char tbuf[25];
    int ix = 0, iNumDig = 0;
    unsigned uMod;
    uint64_t u64Tmp;

    do
    {
        //
        // Divide by 10
        //
        u64Tmp = divu64_10(ui64Val);

        //
        // Get modulus
        //
        uMod = ui64Val - (u64Tmp * 10);

        tbuf[ix++] = uMod + '0';
        ui64Val = u64Tmp;
    } while ( ui64Val );

    //
    // Save the total number of digits
    //
    iNumDig = ix;

    //
    // Now, reverse the buffer when saving to the caller's buffer.
    //
    if ( pcBuf )
    {
        while ( ix-- )
        {
            *pcBuf++ = tbuf[ix];
        }

        //
        // Terminate the caller's buffer
        //
        *pcBuf = 0x00;
    }

    return iNumDig;
}

#define OLA_FTOA_ERR_VAL_TOO_SMALL   -1
#define OLA_FTOA_ERR_VAL_TOO_LARGE   -2
#define OLA_FTOA_ERR_BUFSIZE         -3

typedef union
{
    int32_t I32;
    float F;
} ola_i32fl_t;

static int ftoa(float fValue, char *pcBuf, int iPrecision, int bufSize)
{
    ola_i32fl_t unFloatValue;
    int iExp2, iBufSize;
    int32_t i32Significand, i32IntPart, i32FracPart;
    char *pcBufInitial, *pcBuftmp;

    iBufSize = bufSize; // *(uint32_t*)pcBuf;
    if (iBufSize < 4)
    {
        return OLA_FTOA_ERR_BUFSIZE;
    }

    if (fValue == 0.0f)
    {
        // "0.0"
        *(uint32_t*)pcBuf = 0x00 << 24 | ('0' << 16) | ('.' << 8) | ('0' << 0);
        return 3;
    }

    pcBufInitial = pcBuf;

    unFloatValue.F = fValue;

    iExp2 = ((unFloatValue.I32 >> 23) & 0x000000FF) - 127;
    i32Significand = (unFloatValue.I32 & 0x00FFFFFF) | 0x00800000;
    i32FracPart = 0;
    i32IntPart = 0;

    if (iExp2 >= 31)
    {
        return OLA_FTOA_ERR_VAL_TOO_LARGE;
    }
    else if (iExp2 < -23)
    {
        return OLA_FTOA_ERR_VAL_TOO_SMALL;
    }
    else if (iExp2 >= 23)
    {
        i32IntPart = i32Significand << (iExp2 - 23);
    }
    else if (iExp2 >= 0)
    {
        i32IntPart = i32Significand >> (23 - iExp2);
        i32FracPart = (i32Significand << (iExp2 + 1)) & 0x00FFFFFF;
    }
    else // if (iExp2 < 0)
    {
        i32FracPart = (i32Significand & 0x00FFFFFF) >> -(iExp2 + 1);
    }

    if (unFloatValue.I32 < 0)
    {
        *pcBuf++ = '-';
    }

    if (i32IntPart == 0)
    {
        *pcBuf++ = '0';
    }
    else
    {
        if (i32IntPart > 0)
        {
            uint64_to_str(i32IntPart, pcBuf);
        }
        else
        {
            *pcBuf++ = '-';
            uint64_to_str(-i32IntPart, pcBuf);
        }
        while (*pcBuf)    // Get to end of new string
        {
            pcBuf++;
        }
    }

    //
    // Now, begin the fractional part
    //
    *pcBuf++ = '.';

    if (i32FracPart == 0)
    {
        *pcBuf++ = '0';
    }
    else
    {
        int jx, iMax;

        iMax = iBufSize - (pcBuf - pcBufInitial) - 1;
        iMax = (iMax > iPrecision) ? iPrecision : iMax;

        for (jx = 0; jx < iMax; jx++)
        {
            i32FracPart *= 10;
            *pcBuf++ = (i32FracPart >> 24) + '0';
            i32FracPart &= 0x00FFFFFF;
        }

        //
        // Per the printf spec, the number of digits printed to the right of the
        // decimal point (i.e. iPrecision) should be rounded.
        // Some examples:
        // Value        iPrecision          Formatted value
        // 1.36399      Unspecified (6)     1.363990
        // 1.36399      3                   1.364
        // 1.36399      4                   1.3640
        // 1.36399      5                   1.36399
        // 1.363994     Unspecified (6)     1.363994
        // 1.363994     3                   1.364
        // 1.363994     4                   1.3640
        // 1.363994     5                   1.36399
        // 1.363995     Unspecified (6)     1.363995
        // 1.363995     3                   1.364
        // 1.363995     4                   1.3640
        // 1.363995     5                   1.36400
        // 1.996        Unspecified (6)     1.996000
        // 1.996        2                   2.00
        // 1.996        3                   1.996
        // 1.996        4                   1.9960
        //
        // To determine whether to round up, we'll look at what the next
        // decimal value would have been.
        //
        if ( ((i32FracPart * 10) >> 24) >= 5 )
        {
            //
            // Yes, we need to round up.
            // Go back through the string and make adjustments as necessary.
            //
            pcBuftmp = pcBuf - 1;
            while ( pcBuftmp >= pcBufInitial )
            {
                if ( *pcBuftmp == '.' )
                {
                }
                else if ( *pcBuftmp == '9' )
                {
                    *pcBuftmp = '0';
                }
                else
                {
                    *pcBuftmp += 1;
                    break;
                }
                pcBuftmp--;
            }
        }
    }

    //
    // Terminate the string and we're done
    //
    *pcBuf = 0x00;

    return (pcBuf - pcBufInitial);
} // ftoa()


// void send_9603() test function, did not work properly just yet.

// void receive_9603() {
//   uint8_t mt_buffer[200]; // Buffer to store received message
//   size_t mtBufferSize = sizeof(mt_buffer); // Size of MT buffer
//   int err = ISBD_SUCCESS;
//   // bool ring = false;
//   int loop_count = 0;

//   // Enable power for the 9603N
//   Serial.println(F("Enabling 9603N power..."));
//   digitalWrite(iridiumPwrEN, HIGH); // Enable Iridium Power
//   delay(1000);

//   // Set up power profile
//   modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);

//   // Begin satellite modem operation
//   Serial.println(F("Starting modem..."));
//   err = modem.begin();
//   if (err != ISBD_SUCCESS) {
//       Serial.print(F("modem.begin failed with error "));
//       Serial.println(err);
//       if (err == ISBD_NO_MODEM_DETECTED) {
//           Serial.println(F("No modem detected: check wiring."));
//       }
//       return;
//   }

//   // Test the signal quality.
//   // This returns a number between 0 and 5.
//   // 2 or better is preferred.
//   err = modem.getSignalQuality(signalQuality);
//   if (err != ISBD_SUCCESS)
//   {
//     Serial.print(F("SignalQuality failed: error "));
//     Serial.println(err);
//     return;
//   }

//   Serial.print(F("On a scale of 0 to 5, signal quality is currently "));
//   Serial.print(signalQuality);
//   Serial.println(F("."));

//     // // Wait up to 10 minutes (600 seconds) for a ring indicator (RI) or message
//     // Serial.println(F("Waiting for RING..."));
//     // while (loop_count < 600) {  // Loop for up to 600 seconds (10 minutes)
//     //     ring = modem.hasRingAsserted();

//     //     if (ring || modem.getWaitingMessageCount() > 0) {
//     //         Serial.println(F("RING asserted or message available. Attempting to read incoming message."));
//     //         break;
//     //     }

//     //     delay(1000);  // Wait for 1 second before checking again
//     //     loop_count++;
//     // }

//   // Check network available.
//   Serial.println(F("Checking Network Available (for up to 60 seconds)..."));
//   int NA = LOW;
//   // int loop_count = 0;
//   while ((NA == LOW) && (loop_count < 10*60)) // Loop for up to 10 minutes
//   {
//     NA = digitalRead(iridiumNA);
//     Serial.print(F("Network is "));
//     if (NA == LOW) Serial.print(F("NOT "));
//     Serial.println(F("available!"));
//     if (NA == LOW) Serial.println(F("(This might be because the 9603N has not yet aquired the ring channel.)"));
//     delay(1000);
//     loop_count++;
//   }

//   Serial.println(F("Begin waiting for RING...")); // Wait for RING to be asserted
//   // This ends the code taken from setup of the AGT Ring example code

//   bool ring = modem.hasRingAsserted();
//   loop_count = 0;
//   while (ring == false && loop_count < 10*60) // Loop for up to 10 minutes{
//   { 
//     ring = modem.hasRingAsserted();
//     Serial.println(ring ? "true" : "false");
//     delay(1000);
//     loop_count++;
//   }
  
//   if ((ring == true) || (modem.getWaitingMessageCount() > 0) || (err != ISBD_SUCCESS))
//   {
//     if (ring == true)
//       Serial.println(F("RING asserted!  Let's try to read the incoming message."));
//     else if (modem.getWaitingMessageCount() > 0)
//       Serial.println(F("Waiting messages available.  Let's try to read them."));
//     else
//       Serial.println(F("Let's try again."));

//     // Clear the Mobile Originated message buffer - just in case it has an old message in it!
//     Serial.println(F("Clearing the MO buffer (just in case)."));
//     err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
//     if (err != ISBD_SUCCESS)
//     {
//       Serial.print(F("clearBuffers failed: error "));
//       Serial.println(err);
//       return;
//     }

//     // Attempt to receive the message
//     err = modem.sendReceiveSBDText(NULL, mt_buffer, mtBufferSize);
//     if (err != ISBD_SUCCESS)
//     {
//       Serial.print(F("sendReceiveSBDBinary failed: error "));
//       Serial.println(err);
//       return;
//     }

//     Serial.println(F("Message received!"));
//     Serial.print(F("Inbound message size is "));
//     messageReceived = true; // Set the flag to indicate that a message has been received
//     Serial.println(mtBufferSize);

//     mt_buffer[mtBufferSize] = '\0'; // Ensure null termination
//     receivedMessageString = String((char*)mt_buffer); // Convert to Arduino String
//     Serial.println("Received Message: " + receivedMessageString); // Print the received message

//     for (int i=0; i<(int)mtBufferSize; ++i)
//     {
//       Serial.print(mt_buffer[i], HEX);
//       if (isprint(mt_buffer[i]))
//       {
//         Serial.print(F("("));
//         Serial.write(mt_buffer[i]);
//         Serial.print(F(")"));
//       }
//       Serial.print(F(" "));
//     }
//     Serial.println();
//     Serial.print(F("Messages remaining to be retrieved: "));
//     Serial.println(modem.getWaitingMessageCount());

//   }

//     // if (!ring && modem.getWaitingMessageCount() == 0) {
//     //     Serial.println(F("No RING or messages received within the timeout period."));
//     //     return;
//     // }

//     // // Clear the Mobile Originated message buffer - just in case it has an old message in it!
//     // Serial.println(F("Clearing the MO buffer."));
//     // err = modem.clearBuffers(ISBD_CLEAR_MO);
//     // if (err != ISBD_SUCCESS) {
//     //     Serial.print(F("clearBuffers failed with error "));
//     //     Serial.println(err);
//     //     return;
//     // }

//     // // Attempt to receive the message
//     // err = modem.sendReceiveSBDText(NULL, mt_buffer, mtBufferSize);
//     // if (err != ISBD_SUCCESS) {
//     //     Serial.print(F("sendReceiveSBDText failed with error "));
//     //     Serial.println(err);
//     //     return;
//     // }

//     // Serial.println(F("Message received!"));
//     // Serial.print(F("Inbound message size is "));
//     // Serial.println(mtBufferSize);
//     // messageReceived = true;

//     // for (int i = 0; i < (int)mtBufferSize; ++i) {
//     //     Serial.print(mt_buffer[i], HEX);
//     //     if (isprint(mt_buffer[i])) {
//     //         Serial.print(F("("));
//     //         Serial.write(mt_buffer[i]);
//     //         Serial.print(F(")"));
//     //     }
//     //     Serial.print(F(" "));
//     // }
//     // Serial.println();

//     // Serial.print(F("Messages remaining to be retrieved: "));
//     // Serial.println(modem.getWaitingMessageCount());

//     // // Wait for the RING indicator to clear, if it's still asserted
//     // while (modem.hasRingAsserted()) {
//     //     Serial.println(F("RING is still asserted. Waiting for it to clear..."));
//     //     delay(1000);
//     // }

//     // Serial.println(F("RING has cleared."));

//     // Power down the modem
//     Serial.println(F("Putting the 9603N to sleep."));
//     err = modem.sleep();
//     if (err != ISBD_SUCCESS) {
//         Serial.print(F("*** modem.sleep failed with error "));
//         Serial.print(err);
//         Serial.println(F(" ***"));
//     }
// }
