//=============================================================================================
// NP_RangeFinder.h
//=============================================================================================
//
// Generic wrapper for range finders in NP
//
//  Date			  Author			Notes
//  2023-03-14  G. Lavabre  Added suport for VL53LOX
//
//=============================================================================================
#ifndef NP_RangeFinder_h
#define NP_RangeFinder_h

#include <math.h>
#include "Arduino.h"
#include "Hardware_Parameters.h"
#ifdef RF_CONFIG_VL53LOX
  #include "Adafruit_VL53L0X.h"
#endif


//--------------------------------------------------------------------------------------------
// Variable declaration

class NP_RangeFinder {
//-------------------------------------------------------------------------------------------
// Function declarations

public:

	NP_RangeFinder();

  void init();
  void Read();
  int   readiness;
  float range;



private:
  void correct();
  #ifdef RF_CONFIG_VL53LOX
    Adafruit_VL53L0X lox;
  #endif
};

#endif
