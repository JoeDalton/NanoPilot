//=============================================================================================
// Hardware_Parameters.cpp
//=============================================================================================
//
// 
//
//  Date	      Author		  Notes
//  2023-03-06  G. Lavabre  How do I calibrate gyroScale ?
//
//=============================================================================================
#include "Hardware_Parameters.h"


#ifdef IMU_CONFIG_LSM9DS1
  // Magnetometer calibration
  float magXScale = 0.010812;   float magYScale = 0.010812;   float magZScale = 0.012151;
  float magXBias  = 13.0554;    float magYBias  = 8.2153;     float magZBias  = 1.3184;

  // Gyrometer calibration
  float gyrXBias   = 1.8447;    float gyrYBias   = -0.2165;   float gyrZBias   = -1.7681;
  float temp = (3.14159265358979323846 / 180.0); // Default output seems to be in deg/s and we want rad/s
  float gyrXScale  = temp;      float gyrYScale  = temp;      float gyrZScale  = temp; // There is a weird inversion in gyro rate after the filter but the gyro is ok. This is weird

  // Accelerometer calibration
  float accXScale = 9.801755;   float accYScale = 9.801755;   float accZScale = 9.698090; // Default output seems to be in g and we want m/s²
  float accXBias  = -0.018513;  float accYBias  = -0.015222;  float accZBias  = -0.050568;
#endif


#ifdef IMU_CONFIG_MPU6050_HMC5883L
  // Magnetometer calibration
  float magXScale = 0.000317;   float magYScale = 0.000317;   float magZScale = 0.000352;
  float magXBias  = -51.500000; float magYBias  = 12.500000;  float magZBias  = -189.500000;


  // Gyrometer calibration
  float gyrXBias = -0.043838;   float gyrYBias = -0.006720;   float gyrZBias = 0.032348;
  float temp = 1;
  float gyrXScale  = temp;      float gyrYScale  = temp;      float gyrZScale  = temp;

  // Accelerometer calibration
  float accXScale = 1.016660;   float accYScale = 1.016660;   float accZScale = 0.904103;
  float accXBias  = 0.440926;   float accYBias  = -0.293449;  float accZBias  = -2.051634;
#endif


#ifdef RF_CONFIG_VL53LOX
  // Range finder calibration
  float rfBias = 0.0;           float rfScale = 0.001; // Default output is in mm and we want everything in U.S.I.
#endif
