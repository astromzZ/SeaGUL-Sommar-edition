#include <Ezo_i2c.h>
#include <sequencer2.h>
#include <Ezo_i2c_util.h>
#include <ICM_20948.h>

// Ezo_board PH = Ezo_board(99, "PH");       //create a PH circuit object, who's address is 99 and name is "PH"
Ezo_board EC = Ezo_board(100, "EC");      //create an EC circuit object who's address is 100 and name is "EC"

#define AD0_VAL 1 //The value of the AD0 pin on the ICM-20948 sensor
ICM_20948_I2C myICM;

uint32_t next_read_time = 0;        

float lastconductivityreading = 0; //The last conductivity reading
float pitch = 0; 
float roll = 0; 
float yaw = 0; 

bool readingConductivity = false; //A flag to indicate if the conductivity sensor is currently being read

#define SDA_PIN 13
#define SCL_PIN 14

void setup() {

    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    while (!Serial) // Wait for the user to open the serial monitor
        ;
    delay(100);
    Serial.println();
    Serial.println();
    Serial.println(F("REVERE Glider SeaGUL"));
    Serial.println(F("Test: Atlas och Orienteringssensor"));
    Serial.println();

    //empty the serial buffer
    while(Serial.available() > 0)
        Serial.read();

    //wait for the user to press any key before beginning
    Serial.println(F("Please check that the Serial Monitor is set to 115200 Baud"));
    Serial.println(F("Then click Send to start the test."));
    Serial.println();
    while(Serial.available() == 0)
        ;

    bool initialized = false;
    while (!initialized) {
        myICM.begin(Wire, AD0_VAL);
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok) {
            Serial.println("Trying again...");
            delay(500);
        } else {
            initialized = true;
        }
    }
    Serial.println("Device connected!");

    bool success = true;
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
        // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Set the FIFO mode to snapshot mode
    success &= (myICM.setFIFOmode(true) == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    if (success) {
        Serial.println("DMP configuration successful");
    } else {
        Serial.println("DMP configuration failed");
    }

}


void loop() {

    delay(80);

    icm_20948_DMP_data_t IMUdata;
    myICM.readDMPdataFromFIFO(&IMUdata);
    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {

        // myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
        if ((IMUdata.header & DMP_header_bitmap_Quat6) > 0) {
        // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
        // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
        // The quaternion data is scaled by 2^30.

        //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

        // Scale to +/- 1
        double q1 = ((double)IMUdata.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        double q2 = ((double)IMUdata.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        double q3 = ((double)IMUdata.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

        double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

        double q2sqr = q2 * q2;

        // roll (x-axis rotation)
        double t0 = +2.0 * (q0 * q1 + q2 * q3);
        double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
        roll = atan2(t0, t1) * 180.0 / PI;

        // pitch (y-axis rotation)
        double t2 = +2.0 * (q0 * q2 - q3 * q1);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        pitch = asin(t2) * 180.0 / PI;

        // yaw (z-axis rotation)
        double t3 = +2.0 * (q0 * q3 + q1 * q2);
        double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
        yaw = atan2(t3, t4) * 180.0 / PI;

        // Calculate display pitch and roll, the values that are displayed on the SeaGUL webpage.

        } else {
            Serial.println("Quat6 data not ready");
        }

    } else {
        // Serial.println("Failed to read DMP data");
    }

    if (!readingConductivity) {
        EC.send_cmd("R"); //Request a reading
        
        next_read_time = millis() + 1000; // Set when the response will arrive
        readingConductivity = true; // Switch to the receiving phase
    } 

    if (readingConductivity && millis() >= next_read_time) {
        receive_response(EC); // Get the response from the EC circuit
        readingConductivity = false; // Switch back to asking for readings
    }

    String logData = "Pitch: " + String(pitch) + " degrees, " +
                     "Reading Conductivity: " + String(readingConductivity) + ", " +
                     "Conductivity: " + String(lastconductivityreading) + " uS/cm";

    Serial.println(logData);
}

void receive_response(Ezo_board &Sensor)  // Function to decode the response after a command was issued
{
  char sensordata_buffer[32];            // Buffer to hold the incoming data
  Ezo_board::errors error = Sensor.receive_cmd(sensordata_buffer, 32); // Receive the response

//   Serial.print(Sensor.get_name());       // Print the name of the circuit getting the reading
//   Serial.print(": ");

  if (error == Ezo_board::SUCCESS) {
    // Serial.println(sensordata_buffer);   // Print the received data
    lastconductivityreading = atof(sensordata_buffer);
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