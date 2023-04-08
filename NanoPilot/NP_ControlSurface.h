//=============================================================================================
// NP_ControlSurface.cpp
//=============================================================================================
//
// Control surface class in NP
//
//  Date        Author      Notes
//  2023-04-08  G. Lavabre  Minimal working code
//
//=============================================================================================

#ifndef NP_ControlSurface_h
#define NP_ControlSurface_h

#include "Hardware_Parameters.h"
#include "NP_ServoDriver.h"
#include <math.h>
#include "Arduino.h"

//--------------------------------------------------------------------------------------------
// Variable declaration

class NP_ControlSurface {
//-------------------------------------------------------------------------------------------
// Function declarations

public:
	NP_ControlSurface();

  void init(int csTypeInput, int csSideInput, int servoIndexInput, NP_ServoDriver servoDriverInput, float angleMinInput, float angleMaxInput);
  void ComputeCommand();
  void SetServo();


private:
  int   csType, csSide, servoIndex;
  float angleMin, angleMax, angleCommand;
  NP_ServoDriver servoDriver;
};

#endif
