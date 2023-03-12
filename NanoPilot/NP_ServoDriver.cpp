//=============================================================================================
// NP_ServoDriver.h
//=============================================================================================
//
// Servos management on PCA9685 Driver board
//
//  Date			  Author			Notes
//  2023-03-06  G. Lavabre  Minimal working code
//
//=============================================================================================
//-------------------------------------------------------------------------------------------
// Header files


#include "NP_ServoDriver.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//-------------------------------------------------------------------------------------------
// Definitions


 
//============================================================================================
// Functions

NP_ServoDriver::NP_ServoDriver()
{
  oscillatorFrequency = 27000000;
  servoFrequency      = 50;
  servo_min           = 72;
  servo_max           = 460;
  angle_min           = -90.0;
  angle_max           = 90.0;
}


//-------------------------------------------------------------------------------------------
void NP_ServoDriver::init()
{
  pwm = Adafruit_PWMServoDriver();
  pwm.begin();
  pwm.setOscillatorFrequency(oscillatorFrequency);
  pwm.setPWMFreq(servoFrequency);
}



//============================================================================================
// Read commanded angle for servo "servoNum"

float NP_ServoDriver::get_angleCommand(uint8_t servoNum)
{
  uint16_t  pulsew = (*this).get_pwm(servoNum);
  float     angle = map(pulsew, servo_min, servo_max, angle_min, angle_max);
  return    angle;
}

//============================================================================================
// Set commanded angle for servo "servoNum"

void NP_ServoDriver::set_angleCommand(uint8_t servoNum, float angle)
{
  if (angle>angle_max ){angle=angle_max; }
  if (angle<angle_min ){angle=angle_min; }
  uint16_t  pulsew = map(angle, angle_min, angle_max, servo_min, servo_max);
  (*this).set_pwm(servoNum, pulsew);
}

//============================================================================================
// Private methods
//============================================================================================

//============================================================================================
// Read pulse width for servo "servoNum"

uint16_t NP_ServoDriver::get_pwm(uint8_t servoNum)
{
  return pwm.getPWM(servoNum);
}

//============================================================================================
// Set pulse width for servo "servoNum"

void NP_ServoDriver::set_pwm(uint8_t servoNum, uint16_t pulsew)
{
  pwm.setPWM(servoNum, 0, pulsew);
}

//============================================================================================
// END OF CODE
//============================================================================================
