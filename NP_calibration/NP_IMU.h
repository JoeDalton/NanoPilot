//=============================================================================================
// NP_IMU.h
//=============================================================================================
//
// Generic wrapper for IMUs in NP
//
//  Date			  Author			Notes
//  2023-03-06  G. Lavabre  Added suport for LSM9DS1
//
//=============================================================================================
#ifndef NP_IMU_h
#define NP_IMU_h

#include <math.h>
#include "Arduino.h"
#ifdef IMU_CONFIG_LSM9DS1
  #include <Arduino_LSM9DS1.h>
#endif
#ifdef IMU_CONFIG_MPU6050_HMC5883L
  #include <Adafruit_MPU6050.h>
  #include <QMC5883LCompass.h>
  #include <Adafruit_Sensor.h>
#endif

//--------------------------------------------------------------------------------------------
// Variable declaration

class NP_IMU {
//-------------------------------------------------------------------------------------------
// Function declarations

public:

	NP_IMU();

  void init();
  void get_raw();
  void correct();
	
	//void dummy(float a, float b, int c);

  int   readiness;
  float gx, gy, gz, ax, ay, az, mx, my, mz;
	


private:
  int cx, cy, cz;
};

#endif
