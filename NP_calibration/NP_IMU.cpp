//=============================================================================================
// NP_IMU.cpp
//=============================================================================================
//
// Generic wrapper for IMUs in NP
//
//  Date			  Author			Notes
//  2023-03-06  G. Lavabre  Added suport for LSM9DS1, MPU6050 and HMC5883L
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

//================================ Set hardware configuration
//Board type
#define BOARD_TYPE_NANO33
//#define BOARD_TYPE_MEGA2560

//IMU config
//#define IMU_CONFIG_LSM9DS1
#define IMU_CONFIG_MPU6050_HMC5883L



#include "NP_IMU.h" // NP_IMU
#include <math.h>
#include "Arduino.h"

#ifdef IMU_CONFIG_LSM9DS1
  #include <Arduino_LSM9DS1.h>
#endif
#ifdef IMU_CONFIG_MPU6050_HMC5883L
  #include <Adafruit_Sensor.h>
  #include <Adafruit_MPU6050.h>
  #include <QMC5883LCompass.h>
  Adafruit_MPU6050 mpu;
  QMC5883LCompass compass;
#endif
//-------------------------------------------------------------------------------------------
// Definitions


 
//============================================================================================
// Functions

NP_IMU::NP_IMU()
{
  gx = 0.0f;
  gy = 0.0f;
  gz = 0.0f;
  ax = 0.0f;
  ay = 0.0f;
  az = 0.0f;
  mx = 0.0f;
  my = 0.0f;
  mz = 0.0f;
  readiness = -1;
}


//-------------------------------------------------------------------------------------------
void NP_IMU::init()
{
  Serial.println("trying to initialize imu");
  #ifdef IMU_CONFIG_LSM9DS1
    if (IMU.begin()) {
      readiness = 1;
    }
  #endif
  #ifdef IMU_CONFIG_MPU6050_HMC5883L
    if (mpu.begin()) {
      readiness = 1;
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
      compass.init();
      Serial.println("initialized imu");
    }
  #endif

}

//============================================================================================
// Get readings from sensors

void NP_IMU::get_raw()
{

  #ifdef IMU_CONFIG_LSM9DS1
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
      }
       if (IMU.gyroscopeAvailable())  {
        IMU.readGyroscope(gx, gy, gz);
      }
      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
      }
  #endif
  #ifdef IMU_CONFIG_MPU6050_HMC5883L
    // Read mpu
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
    // Read compass
    compass.read();
    cx = compass.getX();
    cy = compass.getY();
    cz = compass.getZ();
    mx = float(cx);
    my = float(cy);
    mz = float(cz);
  #endif
}

//============================================================================================
// Correct values with calibration
/*
void NP_IMU::correct() {
  // Magnetometer calibration
  mx -= magXBias;   my -= magYBias;   mz -= magZBias;
  mx *= magXScale;  my *= magYScale;  mz *= magZScale;
  // Gyrometer calibration
  gx -= gyrXBias;   gy -= gyrYBias;   gz -= gyrZBias;
  gx *= gyrXScale;  gy *= gyrYScale;  gz *= gyrZScale;
  // Accelerometer calibration
  ax -= accXBias;   ay -= accYBias;   mz -= accZBias;
  ax *= accXScale;  ay *= accYScale;  mz *= accZScale;
}
*/


//============================================================================================
// END OF CODE
//============================================================================================
