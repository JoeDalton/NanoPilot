//=============================================================================================
// NP_RangeFinder.cpp
//=============================================================================================
//
// Generic wrapper for range finders in NP
//
//  Date			  Author			Notes
//  2023-03-14  G. Lavabre  Added suport for VL53LOX
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "NP_RangeFinder.h" // NP_RangeFinder
#include <math.h>
#include "Arduino.h"
#include "Hardware_Parameters.h"
#ifdef RF_CONFIG_VL53LOX
  #include "Adafruit_VL53L0X.h"
#endif

//-------------------------------------------------------------------------------------------
// Definitions


 
//============================================================================================
// Functions

NP_RangeFinder::NP_RangeFinder()
{
  distance = 0.0f;
  readiness = 1;
}


//-------------------------------------------------------------------------------------------
void NP_RangeFinder::init()
{
  Serial.println("Initializing range finder");
  #ifdef RF_CONFIG_VL53LOX
    Serial.println("Range finder type found");
    Adafruit_VL53L0X lox = Adafruit_VL53L0X();
    Serial.println("lox defined");
    if (!lox.begin()) {
      Serial.println("Failed to boot VL53L0X");
      readiness = -1;
    }
  #endif
}

//============================================================================================
// Get readings from sensors

void NP_RangeFinder::Read()
{

  #ifdef RF_CONFIG_VL53LOX
    Serial.println("I read VL53L0X");
    Serial.println(lox.readRange());
    Serial.println(lox.isRangeComplete());
    if (!lox.isRangeComplete()) {
      Serial.println("Range is not complete");
      return;
    }
    Serial.println("VL53L0X range is complete");
    //distance = lox.readRange();
    Serial.println(lox.readRange());
    Serial.println("VL53L0X range is read");
    (*this).correct();
    Serial.println("VL53L0X range is corrected");
    
  #endif
}

//============================================================================================
// Correct values with calibration
void NP_RangeFinder::correct() {
  distance -= rfBias;
  distance *= rfScale;
}


//============================================================================================
// END OF CODE
//============================================================================================
