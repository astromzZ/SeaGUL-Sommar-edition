/*
This is code for the Artemis Global Tracker that is mounted on the underwater glider "SeaGul".
This is custum code that is to a great degree inpired by the exaplecode that can be found on the AGT github https://github.com/sparkfun/SparkFun_Artemis_Global_Tracker/tree/main/Software/examples 
To run the code select the board RedBoard Artemis ATP and the correct COM. The board can be found here: https://raw.githubusercontent.com/sparkfun/Arduino_Apollo3/main/package_sparkfun_apollo3_index.json 
Simply add the apollo3 package URL under preferences in the addisional boards manager URL field.

Last updated 2024-07-02

The purpouse of this code is to be able to use usefull functions of the ATG and be able to communicate with another microcontroller, ESP32-S3.
The code should be able to handle GPS positioning, Iridium messaging, serial connectin to esp32, sensor readings, interups and so on.
*/

//TODO: se över om alla dessa behövs 
// Artemis Global Tracker pin definitions (found in all examples)
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

// Make sure you do not have gnssEN and iridiumPwrEN enabled at the same time!
// If you do, bad things might happen to the AS179 RF switch!
// We use Serial1 to communicate with the Iridium modem. Serial1 on the ATP uses pin 24 for TX and 25 for RX. AGT uses the same pins.

#include <SoftwareSerial.h>      // Used to create a SoftwareSerial UART, https://github.com/paulvha/apollo3, must be added manually by downloading the zip 
SoftwareSerial mySerial(D5, D6); // RX, TX - Any pins can be used, we use D5 and D6 (SCK, CIPO) see schematic: https://github.com/sparkfun/SparkFun_Artemis_Global_Tracker/blob/main/Documentation/SparkFun_Artemis_Global_Tracker_SCHEMATIC_v10.pdf 

#include <Wire.h>
// TwoWire agtWire(9,8); //Create an I2C port using pads 8 (SCL) and 9 (SDA)
//TwoWire qwiic(40,39); //Will use Artemis pads 39 (SCL) and 40 (SDA), activate if the quiic port is to be used

#include <SparkFun_PHT_MS8607_Arduino_Library.h> //http://librarymanager/All#SparkFun_MS8607 The MS8607 is connected to I2C Port 1: SCL = D8; SDA = D9
MS8607 barometricSensor; //Create an instance of the MS8607 object

#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS; //The ZOE-M8Q shares I2C Port 1 with the MS8607: SCL = D8; SDA = D9

#include <IridiumSBD.h> //http://librarymanager/All#IridiumSBDI2C
#define DIAGNOSTICS false // Change this to true to see IridiumSBD diagnostics, TODO: denna kanske är onödig. hör till #if DIAGNOSTIC
IridiumSBD modem(Serial1, iridiumSleep, iridiumRI); // Declare the IridiumSBD object (including the sleep (ON/OFF) and Ring Indicator pins)

#include <time.h>
#include "RTC.h" //Include RTC library included with the Arduino_Apollo3 core

#include <EEPROM.h> // Needed for EEPROM storage on the Artemis


/* TODO: lägg till korekta namn på alla de cases jag vill ha 
// Loop Steps - these are used by the switch/case in the main loop
// This structure makes it easy to go from any of the steps directly to zzz when (e.g.) the batteries are low
typedef enum {
  loop_init = 0, // Send the welcome message, check the battery voltage
  read_pressure, // Read the pressure and temperature from the MS8607
  start_GPS,     // Enable the ZOE-M8Q, check the battery voltage
  read_GPS,      // Wait for up to GNSS_timeout minutes for a valid 3D fix, check the battery voltage
  start_LTC3225, // Enable the LTC3225 super capacitor charger and wait for up to CHG_timeout minutes for PGOOD to go high
  wait_LTC3225,  // Wait TOPUP_timeout seconds to make sure the capacitors are fully charged
  start_9603,    // Power on the 9603N, send the message, check the battery voltage
  sleep_9603,    // Put the 9603N to sleep
  zzz,           // Turn everything off and put the processor into deep sleep
  wakeUp,        // Wake from deep sleep, restore the processor clock speed
  wait_for_ring, // Keep the 9603N powered up and wait for a ring indication
  configureMe    // Configure the tracker settings via USB Serial
} loop_steps;
volatile loop_steps loop_step = loop_init; // Make sure loop_step is set to loop_init
volatile loop_steps last_loop_step = loop_init; // Go back to this loop_step after doing a configure
*/


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


void setup() //TODO: denna är inte klar 
{
  pinMode(LED, OUTPUT); // Make the LED pin an output

  gnssOFF(); // Disable power for the GNSS
  pinMode(gnssBckpBatChgEN, INPUT); // GNSS backup battery charge control; input = disable charging; output+low=charging. 
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

  pinMode(busVoltageMonEN, OUTPUT); // Make the Bus Voltage Monitor Enable an output
  digitalWrite(busVoltageMonEN, LOW); // Set it low to disable the measurement (busV should be ~zero), Set it high to enable the measurement
  analogReadResolution(14); //Set resolution to 14 bit

  // Set up the I2C pins
  agtWire.begin();
}

void loop() 
{
  /* TODO: detta är bare ett exempel på hur det kan struktureras 
  switch (loop_step) {
    case loop_init:
      handleInit();
      break;
    case read_pressure:
      handleReadPressure();
      break;
    case start_GPS:
      handleStartGPS();
      break;
    case read_GPS:
      handleReadGPS();
      break;
    case start_LTC3225:
      handleStartLTC3225();
      break;
    case wait_LTC3225:
      handleWaitLTC3225();
      break;
    case start_9603:
      handleStart9603();
      break;
    case sleep_9603:
      handleSleep9603();
      break;
    case zzz:
      handleZzz();
      break;
    case wakeUp:
      handleWakeUp();
      break;
    case wait_for_ring:
      handleWaitForRing();
      break;
    case configureMe:
      handleConfigureMe();
      break;
    default:
      Serial.println("Unknown state!");
      break;
  }
  */
  // put your main code here, to run repeatedly:
  //TODO: använd cases, se global tracker exemplet, bygg en i taket för att utveckla funktioner

  // ************************************************************************************************
  //Case 1...

  // ************************************************************************************************
  //Case 2...

  // ************************************************************************************************
  //Case 3...

}


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


void setAGTWirePullups(uint32_t i2cBusPullUps) //is used to change internal pullups for the i2c defined AGTWire (MS8607 and ZOE-M8Q)
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
