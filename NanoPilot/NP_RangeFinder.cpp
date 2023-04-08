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
  range = 0.0f;
  readiness = 1;
  #ifdef RF_CONFIG_VL53LOX
    Adafruit_VL53L0X lox = Adafruit_VL53L0X();
  #endif
}


//-------------------------------------------------------------------------------------------
void NP_RangeFinder::init()
{
  Serial.println("Initializing range finder");
  #ifdef RF_CONFIG_VL53LOX
    Serial.println("Range finder type found");
    Serial.println("lox defined");
    if (!lox.begin()) {
      Serial.println("Failed to boot VL53L0X");
      readiness = -1;
    }
    else{
      lox.startRangeContinuous();
      Serial.println("lox started");
    }
  #endif
}

//============================================================================================
// Get readings from sensors

float NP_RangeFinder::Read()
{

  #ifdef RF_CONFIG_VL53LOX
   // Serial.println("I read VL53L0X");
    
    /*Serial.println(lox.isRangeComplete());
    if (!lox.isRangeComplete()) {
      Serial.println("Range is not complete");
      return;
    }
    else {*/
      //Serial.println("VL53L0X range is complete");
      range = lox.readRange();
      (*this).correct();
      //Serial.println("VL53L0X range is read and corrected");

      // A problem remains: lox.isRangeComplete() is True only on the first try, then it's always false. However, "just" reading the range seems to work. Soooo... Good enough for now, I guess !
    //}
    
    
    
  #endif
  return range;
}

//============================================================================================
// Correct values with calibration
void NP_RangeFinder::correct() {
  range -= rfBias;
  range *= rfScale;
}


//============================================================================================
// END OF CODE
//============================================================================================
