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
#ifndef NP_ServoDriver_h
#define NP_ServoDriver_h

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//--------------------------------------------------------------------------------------------
// Variable declaration

class NP_ServoDriver {
//-------------------------------------------------------------------------------------------
// Function declarations

public:

	NP_ServoDriver();

  void      init();
  float     get_angleCommand(uint8_t servoNum);
  void      set_angleCommand(uint8_t servoNum, float angle);
  uint16_t  get_pwm(uint8_t servoNum);
  void      set_pwm(uint8_t servoNum, uint16_t pulsew);
  Adafruit_PWMServoDriver pwm;
	//void dummy(float a, float b, int c);

	


private:
  

  uint8_t   servoFrequency;
  uint32_t  oscillatorFrequency;
  uint16_t  servo_min; // Maybe use arrays here if all servos are not the same
  uint16_t  servo_max;
  float     angle_min;
  float     angle_max;
};

#endif
