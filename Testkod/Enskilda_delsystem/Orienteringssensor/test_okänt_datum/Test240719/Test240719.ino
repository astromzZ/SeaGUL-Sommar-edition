// I threw this together quickly, so beware!
// Reads ICM_20948 using Sparkfun library. Does fusion using Adafruit AHRS library. Outputs Euler angles.
// Runs at 100 Hz on Arduino Uno.
// I flipped some axes because I prefer NED orientation.

#include <ICM_20948.h>      // I'm using sparkfun's ICM20948 library (I couldn't get Adafruit's library to work well)
#include <Adafruit_AHRS.h>  // library works, except orientation takes many seconds to settle down if sensor wasn't pointed north at startup

#define AD0_VAL 1           // I2C address LSB, you may need to change this for your breakout
ICM_20948_I2C      myICM;   // ICM_20948_I2C object
ICM_20948_fss_t    myFSS;   // full scale settings structure that can contain values for all configurable sensors
ICM_20948_dlpcfg_t myDLP;   // configuration structure for the desired sensors

Adafruit_Mahony filter;
#define SAMPLERATE_HZ 100   // 100 Hz works fine on Arduino Uno
unsigned long tprev;        // time of previous measurement

void setup()
{
  Serial.begin(115200);
  while (!Serial) {};

  Wire.begin();

  myICM.begin(Wire, AD0_VAL);
  myFSS.a = gpm8;             // gpm2(default) gpm4 gpm8 gpm16
  myFSS.g = dps2000;          // dps250(default) dps500 dps1000 dps2000
  myICM.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, myFSS);
  #if 1    // enable accel and gyro lowpass filters, is there no mag filter?
    myDLP.a = acc_d23bw9_n34bw4;  // acc_d473bw_n499bw acc_d246bw_n265bw(default) acc_d111bw4_n136bw acc_d50bw4_n68bw8 acc_d23bw9_n34bw4 acc_d11bw5_n17bw acc_d5bw7_n8bw3 (3dB bandwidth nyquist bandwidth)
    myDLP.g = gyr_d23bw9_n35bw9;  // gyr_d361bw4_n376bw5 gyr_d196bw6_n229bw8(default) gyr_d151bw8_n187bw6 gyr_d119bw5_n154bw3 gyr_d51bw2_n73bw3 gyr_d23bw9_n35bw9 gyr_d11bw6_n17bw8 gyr_d5bw7_n8bw9 (3dB bandwidth nyquist bandwidth)
    myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, myDLP);
    myICM.enableDLPF(ICM_20948_Internal_Acc, true);
    myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  #endif

  filter.begin(SAMPLERATE_HZ);

  Wire.setClock(400000);

  tprev = micros();
}

void loop()
{
  myICM.getAGMT();
  float ax = -myICM.accX() * 0.00 + 0.000;  // these offsets calibrate MY sensor, YOUR sensor needs different offsets
  float ay =  myICM.accY() * 0.00 + 0.00;
  float az = -myICM.accZ() * 0.00 + 0.00;
  float gx = -myICM.gyrX()         - 0.00;
  float gy =  myICM.gyrY()         - 0.00;
  float gz = -myICM.gyrZ()         + 0.00;
  float mx = -myICM.magX()         -  0.00;
  float my = -myICM.magY()         -  0.00;
  float mz =  myICM.magZ()         -  0.00;

  filter.update(gx, gy, gz, -ax, -ay, -az, mx, my, mz);  // gyro deg/sec, acc and mag don't care
  float roll    = filter.getRoll();
  float pitch   = filter.getPitch();
  float heading = filter.getYaw() + 180;  // it pointed the wrong way, the 180 fixed it
  Serial.print("Orientation: ");
  Serial.print(heading);  Serial.print(" ");
  Serial.print(roll);     Serial.print(" ");
  Serial.print(pitch);    Serial.println();

  #define PERIOD_US (unsigned long)round(1000000.0 / SAMPLERATE_HZ)
  while (micros() - tprev < PERIOD_US);  // wait until next measurement
  tprev += PERIOD_US;
}