#include "Streaming.h"    // needed for simpler Serial output https://github.com/geneReeves/ArduinoStreaming
#include "NP_SensorFusion.h"

//#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "NP_IMU.h"
#include "NP_ServoDriver.h"
#include "NP_RangeFinder.h"

SF fusion;
NP_IMU imu;
NP_ServoDriver sd;
//NP_RangeFinder rf;


float pitch, roll, yaw, alti;
float radRoll, radPitch;
float deltat;


#define RAW_DATA
#define EULER_DATA
//#define PROCESSING
//#define SERIAL_PLOTER
#define VERBOSE







#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();







void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  wait_for_serial();
  //===================== Initialize servo driver
  sd.init();
  test_servos();

  //===================== Initialize IMU
  imu.init();

  //===================== Initialize range finder
  //rf.init();

  //===================== Initialize fusion algorithm
  imu.Read();
  #ifdef VERBOSE
    if (fusion.initQuat(imu.ax,imu.ay,imu.az,imu.mx,imu.my,imu.mz)){
      Serial.println("Fusion algorithm setup succeeded");
    }
    else {
      Serial.println("Fusion algorithm setup failed");
    }
  #else
    fusion.initQuat(imu.ax,imu.ay,imu.az,imu.mx,imu.my,imu.mz);
  #endif
  


  //===================== Misc.
  #ifdef SERIAL_PLOTER
    Serial << "pitch" << " " << "roll" << " " << "yaw" << "altitude" << endl;

  #endif
  
  #ifdef VERBOSE
  if (imu.readiness < 0){
    Serial.println("Setup failed");
    Serial.println("");
    Serial.println("--------------------");
    Serial.println("");
  }
  else {
    Serial.println("Setup is successful");
    Serial.println("");
    Serial.println("--------------------");
    Serial.println("");
  }
  #endif




  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power
  Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));

  // start continuous ranging
  lox.startRangeContinuous();


  
  delay(1000);
}





void loop() {
  Serial.println("began loop");
  // ------------------ Read sensors
  imu.Read();
  Serial.println("1");
    if (lox.isRangeComplete()) {
    Serial.print("Distance in mm: ");
    Serial.println(lox.readRange());
  }
  delay(1000);
  //rf.Read();
  Serial.println("2");
  
  //DEBUG
  //imu.gx = 0; imu.gy = 0; imu.gz = 0;
  //imu.mx = 0; imu.my = 0; imu.mz = 0;
  

  // ------------------ Print raw sensor output
  #ifdef RAW_DATA
    print_raw();
  #endif
  Serial.println("3");
  // ------------------ Update AHRS
  AHRS_step();
  Serial.println("4");







  // ------------------ Move servos according to board orientation
  sd.set_angleCommand(0,roll);
  sd.set_angleCommand(1,pitch);
  sd.set_angleCommand(2,yaw-90.0);


  // ------------------ Print Euler
  #ifdef EULER_DATA
    Serial << "Pitch:\t" << pitch << "\t\tRoll:\t" << roll << "\t\tYaw:\t" << yaw << "\t\tAltitude:\t" << alti << endl << endl;
  #endif

  // ------------------ Print Rad
  #ifdef PROCESSING
    roll = fusion.getRollRadians();
    pitch = fusion.getPitchRadians();
    yaw = fusion.getYawRadians();
    Serial  << pitch << ":" << roll << ":" << yaw << endl;
  #endif
  // ------------------ Plot
  #ifdef SERIAL_PLOTER
    Serial << pitch << " " << roll << " " << yaw << " " << alti << endl;
  #endif

  


  
  //delay(10);
  //delay(1000); // For readability
}

void test_servos() {
  sd.set_angleCommand(0,0.0);
  sd.set_angleCommand(1,0.0);
  sd.set_angleCommand(2,0.0);
  delay(500);
  sd.set_angleCommand(0,90.0);
  sd.set_angleCommand(1,90.0);
  sd.set_angleCommand(2,90.0);
  delay(500);
  sd.set_angleCommand(0,0.0);
  sd.set_angleCommand(1,0.0);
  sd.set_angleCommand(2,0.0);
  /*delay(500);
  sd.set_angleCommand(0,-90.0);
  delay(500);
  sd.set_angleCommand(0,0.0);*/
}

void AHRS_step() {
  deltat = fusion.deltatUpdate();
  //fusion.MahonyUpdate(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, deltat);  //Mahony filter is suggested if no compass is available
  fusion.MadgwickUpdate(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz, deltat);  //else use Madgwick's filter

  roll      = fusion.getRoll();
  pitch     = fusion.getPitch();
  yaw       = fusion.getYaw();

  radRoll   = fusion.getRollRadians();
  radPitch  = fusion.getPitchRadians();

  // Approximate altitude
  //alti = rf.distance * fast_cosinus(radRoll) * fast_cosinus(radPitch);
  //alti = rf.distance * cos(radRoll) * cos(radPitch);
}

float fast_cosinus(float angle) {
  float anglesq = angle*angle;
  float cosinus = 1 - anglesq * 0.5 + anglesq*anglesq * 0.0416666666667; // Series expansion of cosinus in 0. Maybe unnecessary ? Should test the time it takes to compute the real cosinus...
  return cosinus;
}

void print_raw() {
  /* Print out the values */
  Serial.print("Acceleration   X: ");
  Serial.print(imu.ax);
  Serial.print(", Y: ");
  Serial.print(imu.ay);
  Serial.print(", Z: ");
  Serial.print(imu.az);
  Serial.println(" m/s^2");

  Serial.print("Rotation       X: ");
  Serial.print(imu.gx);
  Serial.print(", Y: ");
  Serial.print(imu.gy);
  Serial.print(", Z: ");
  Serial.print(imu.gz);
  Serial.println(" rad/s");

  Serial.print("Magnetic field X: ");
  Serial.print(imu.mx);
  Serial.print("   Y: ");
  Serial.print(imu.my);
  Serial.print("   Z: ");
  Serial.print(imu.mz);
  Serial.println(" muT");
  Serial.println("");
}

void wait_for_serial() {
  while (!Serial)
    delay(10); // Pauses the controller until a serial console opens
  #ifdef VERBOSE
    Serial.println("Found serial ! So happy !");
  #endif
}
