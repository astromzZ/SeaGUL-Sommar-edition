// Code that enables command control of the Atlas Scientific Conductivity sensor in I2C mode

#include <Arduino.h>
#include <Ezo_i2c.h>                      // Link: https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>                         // Include Arduino's I2C library

#define SDA_PIN 13                        // Define SDA pin
#define SCL_PIN 14                        // Define SCL pin

Ezo_board EC = Ezo_board(100, "EC");      // Create an EC (Conductivity) circuit object with I2C address 100 and name "EC"

bool reading_request_phase = true;        // Selects our phase

uint32_t next_poll_time = 0;              // Holds the next time we receive a response, in milliseconds
const unsigned int response_delay = 1000; // How long we wait to receive a response, in milliseconds

String inputstring = "";                  // A string to hold incoming data from the PC
bool input_string_complete = false;       // Have we received all the data from the PC

void setup()
{
  Wire.begin(SDA_PIN, SCL_PIN);           // Start the I2C with specified SDA and SCL pins
  Serial.begin(9600);                     // Start the serial communication to the computer at baud rate of 9600
  inputstring.reserve(30);                // Set aside some bytes for receiving data from the PC

  // Add a small delay to ensure sensor is ready
  delay(2000);
}

void serialEvent()
{
  inputstring = Serial.readStringUntil(13); // Read the string until we see a <CR>
  input_string_complete = true;             // Set the flag used to tell if we have received a completed string from the PC
}

void loop()
{
  if (input_string_complete)             // If a string from the PC has been received in its entirety
  {
    Serial.print("Sending command to sensor: "); // Debug print
    Serial.println(inputstring);                 // Debug print

    EC.send_cmd(inputstring.c_str());    // Send that string to the Atlas Scientific product
    inputstring = "";                    // Clear the string
    input_string_complete = false;       // Reset the flag used to tell if we have received a completed string from the PC

    next_poll_time = millis() + response_delay; // Set when the response will arrive
    reading_request_phase = false;       // Switch to the receiving phase
  }

  if (!reading_request_phase && millis() >= next_poll_time) // If it's time to get the response
  {
    receive_response(EC);                // Get the response from the EC circuit
    reading_request_phase = true;        // Switch back to asking for readings
  }
}

void receive_response(Ezo_board &Sensor)  // Function to decode the response after a command was issued
{
  char sensordata_buffer[32];            // Buffer to hold the incoming data
  Ezo_board::errors error = Sensor.receive_cmd(sensordata_buffer, 32); // Receive the response

  Serial.print(Sensor.get_name());       // Print the name of the circuit getting the reading
  Serial.print(": ");

  if (error == Ezo_board::SUCCESS) {
    Serial.println(sensordata_buffer);   // Print the received data
  } else {
    switch (error) {
      case Ezo_board::FAIL:
        Serial.println("Failed");        // The command has failed
        break;
      case Ezo_board::NOT_READY:
        Serial.println("Pending");       // The command has not yet been finished calculating
        break;
      case Ezo_board::NO_DATA:
        Serial.println("No Data");       // The sensor has no data to send
        break;
      case Ezo_board::NOT_READ_CMD:
        Serial.println("Not Read Command"); // The command was not a read command
        break;
      default:
        Serial.println("Unknown error"); // An unknown error occurred
        break;
    }
  }
}
