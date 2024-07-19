#include <Wire.h>
#include <Arduino.h>
#include "ICM_20948.h"

#define SDA_PIN 13
#define SCL_PIN 14

#define AD0_VAL 1
#define WIRE_PORT Wire

ICM_20948_I2C myICM;

// Calibration parameters (Replace these with your actual calibration values)
float hard_iron_offset[3] = {0, 0, 0};
float soft_iron_scale[3] = {1, 1, 1};

void setup()
{
    Serial.begin(115200); // Start the serial console

    delay(100);

#ifndef QUAT_ANIMATION
    while (Serial.available()) // Make sure the serial RX buffer is empty
        Serial.read();

    Serial.println(F("Press any key to continue..."));

    while (!Serial.available()) // Wait for the user to press a key (send any serial character))
        ;
#endif

#ifdef USE_SPI
    SPI_PORT.begin();
#else
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
#endif

#ifndef QUAT_ANIMATION
    myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
#endif

    bool initialized = false;
    while (!initialized)
    {
#ifdef USE_SPI
        myICM.begin(CS_PIN, SPI_PORT);
#else
        myICM.begin(WIRE_PORT, AD0_VAL);
#endif

#ifndef QUAT_ANIMATION
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
#endif
        if (myICM.status != ICM_20948_Stat_Ok)
        {
#ifndef QUAT_ANIMATION
            Serial.println(F("Trying again..."));
#endif
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }

#ifndef QUAT_ANIMATION
    Serial.println(F("Device connected!"));
#endif

    bool success = true; // Use success to show if the DMP configuration was successful

    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    if (success)
    {
#ifndef QUAT_ANIMATION
        Serial.println(F("DMP enabled!"));
#endif
    }
    else
    {
        Serial.println(F("Enable DMP failed!"));
        Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        while (1)
            ; // Do nothing more
    }
}

void loop()
{
    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);

    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
        {
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            double q2sqr = q2 * q2;

            double t0 = +2.0 * (q0 * q1 + q2 * q3);
            double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
            double roll = atan2(t0, t1) * 180.0 / PI;

            double t2 = +2.0 * (q0 * q2 - q3 * q1);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            double pitch = asin(t2) * 180.0 / PI;

            double t3 = +2.0 * (q0 * q3 + q1 * q2);
            double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
            double yaw = atan2(t3, t4) * 180.0 / PI;

            // Magnetometer data
            if ((data.header & DMP_header_bitmap_Compass) > 0)
            {
                float rawMag[3] = {data.Compass.Data.X, data.Compass.Data.Y, data.Compass.Data.Z};
                float calibratedMag[3];

                // Apply Hard Iron Offset
                for (int i = 0; i < 3; i++)
                {
                    rawMag[i] -= hard_iron_offset[i];
                }

                // Apply Soft Iron Scale
                for (int i = 0; i < 3; i++)
                {
                    calibratedMag[i] = rawMag[i] / soft_iron_scale[i];
                }

                float heading = atan2(calibratedMag[1], calibratedMag[0]) * 180.0 / PI;
                if (heading < 0)
                {
                    heading += 360;
                }

#ifndef QUAT_ANIMATION
                Serial.print(F("Roll:"));
                Serial.print(roll, 1);
                Serial.print(F(" Pitch:"));
                Serial.print(pitch, 1);
                Serial.print(F(" Yaw:"));
                Serial.print(yaw, 1);
                Serial.print(F(" Heading:"));
                Serial.println(heading, 1);
#else
                Serial.print(F("{\"quat_w\":"));
                Serial.print(q0, 3);
                Serial.print(F(", \"quat_x\":"));
                Serial.print(q1, 3);
                Serial.print(F(", \"quat_y\":"));
                Serial.print(q2, 3);
                Serial.print(F(", \"quat_z\":"));
                Serial.print(q3, 3);
                Serial.print(F(", \"heading\":"));
                Serial.print(heading, 1);
                Serial.println(F("}"));
#endif
            }
        }
    }

    if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
        delay(100);
    }
}
