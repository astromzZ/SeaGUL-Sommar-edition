#include <Arduino.h>
#include <Ezo_i2c.h>                      // Link: https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>                         // Include Arduino's I2C library

#define SDA_PIN 13                        // Define SDA pin
#define SCL_PIN 14                        // Define SCL pin

Ezo_board EC = Ezo_board(100, "EC");      // Create an EC (Conductivity) circuit object with I2C address 100 and name "EC"

bool reading_request_phase = true;        // Selects our phase

uint32_t next_poll_time = 0;              // Holds the next time we receive a response, in milliseconds
const unsigned int response_delay = 1000; // How long we wait to receive a response, in milliseconds

void setup()
{
  Wire.begin(SDA, SCL);                           // Start the I2C
  Serial.begin(9600);                     // Start the serial communication to the computer at baud rate of 9600
}

void loop()
{
  if (reading_request_phase)              // If we're in the phase where we ask for a reading
  {
    EC.send_read_cmd();                   // Send a read command

    next_poll_time = millis() + response_delay; // Set when the response will arrive
    reading_request_phase = false;             // Switch to the receiving phase
  }
  else                                    // If we're in the receiving phase
  {
    if (millis() >= next_poll_time)       // And it's time to get the response
    {
      receive_reading(EC);                // Get the reading from the EC circuit

      reading_request_phase = true;       // Switch back to asking for readings
    }
  }
}

void receive_reading(Ezo_board &Sensor)   // Function to decode the reading after the read command was issued
{
  Serial.print(Sensor.get_name());        // Print the name of the circuit getting the reading
  Serial.print(": ");

  Sensor.receive_read_cmd();              // Get the response data and put it into the Sensor.reading variable if successful

  switch (Sensor.get_error())             // Switch case based on what the response code is
  {
    case Ezo_board::SUCCESS:
      Serial.println(Sensor.get_last_received_reading()); // The command was successful, print the reading
      break;

    case Ezo_board::FAIL:
      Serial.print("Failed ");            // Means the command has failed
      break;

    case Ezo_board::NOT_READY:
      Serial.print("Pending ");           // The command has not yet been finished calculating
      break;

    case Ezo_board::NO_DATA:
      Serial.print("No Data ");           // The sensor has no data to send
      break;
  }
}
