//=============================================================================================
// NP_ControlSurface.cpp
//=============================================================================================
//
// Control surface class in NP
//
//  Date			  Author			Notes
//  2023-04-08  G. Lavabre  Minimal working code
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "NP_ControlSurface.h" // NP_ControlSurface
#include "Hardware_Parameters.h"
#include "NP_ServoDriver.h"

#include <math.h>
#include "Arduino.h"


/*#ifdef MOBILE_CONFIG_HYDROFOIL
  #include <Arduino_LSM9DS1.h>
#endif*/

//-------------------------------------------------------------------------------------------
// Definitions


 
//============================================================================================
// Functions

NP_ControlSurface::NP_ControlSurface()
{
  csType        = 0;        // 0: Aileron, 1: Flap, 2: Flaperon, 3: Elevon
  csSide        = 0;        // 0: Port, 1: Starboard
  servoIndex    = 0;
  angleMin      = 0.0f;
  angleMax      = 0.0f;
  angleCommand  = 0.0f;
}


//-------------------------------------------------------------------------------------------
void NP_ControlSurface::init(int csTypeInput, int csSideInput, int servoIndexInput, NP_ServoDriver servoDriverInput, float angleMinInput, float angleMaxInput)
{
  csType      = csTypeInput;
  csSide      = csSideInput;
  servoIndex  = servoIndexInput;
  servoDriver = servoDriverInput;
  angleMin    = angleMinInput;
  angleMax    = angleMaxInput;
}

//============================================================================================
// Compute the angle command for the servomotor
void NP_ControlSurface::ComputeCommand()
{
  angleCommand = 0.0; // Determine how to compute the command. Take a command array as input and extract relevant component as a function of csType and csSide, clip with angleMin and angleMax
}

//============================================================================================
// Set servomotor command
void NP_ControlSurface::SetServo()
{
  servoDriver.set_angleCommand(servoIndex,angleCommand);
}




//============================================================================================
// END OF CODE
//============================================================================================
