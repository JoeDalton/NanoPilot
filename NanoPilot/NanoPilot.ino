
// Only for vscode to get Arduino.h instead of WProgram.h
//#define ARDUINO 100

#include "Streaming.h"    // needed for simpler Serial output https://github.com/geneReeves/ArduinoStreaming

#include "NP_SensorFusion.h"
#include "NP_IMU.h"
#include "NP_ServoDriver.h"
#include "NP_RangeFinder.h"

SF fusion;
NP_IMU imu;
NP_ServoDriver sd;
NP_RangeFinder rf;


float q0, q1, q2, q3, quatArray[4];
float pitch, roll, yaw, alti, oldAlti;
float radRoll, radPitch;
float deltat;

//#define RAW_DATA
//#define EULER_DATA
#define QUAT_DATA
// #define PROCESSING
//#define SERIAL_PLOTTER
//#define VERBOSE


// Note: Yaw angle seems to drift A LOT, even using Madgwick's compass fusion.
// Also, there are spikes in the estimated range.
// I suspect they are linked to the issue with lox.isRangeComplete(), which does not allow verifying that the range measurement is meaningful when reading.


void setup(void) {
  //===================== Initialize serial link
  initialize_serial();
  
  //===================== Initialize servo driver
  sd.init();
  test_servos();
  
  Serial.println("serovs ok");

  //===================== Initialize IMU
  imu.init();
  
  Serial.println("imu ok");

  //===================== Initialize range finder
  rf.init();
  
  Serial.println("range finder ok");

  //===================== Initialize fusion algorithm
  
  Serial.println("fusion ok");
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
    Serial << "pitch" << " " << "roll" << " " << "yaw" << " " << "altitude" << endl;

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
  
  delay(1000);
}

void loop() {
  #ifdef VERBOSE
    Serial.println("began loop");
  #endif
  // ------------------ Read sensors
  imu.Read();
  rf.Read();
  
  //DEBUG
  //imu.gx = 0; imu.gy = 0; imu.gz = 0;
  //imu.mx = 0; imu.my = 0; imu.mz = 0;
  

  // ------------------ Print raw sensor output
  #ifdef RAW_DATA
    print_raw();
  #endif
  
  // ------------------ Update AHRS
  AHRS_step();

  // ------------------ Move servos according to board orientation
  sd.set_angleCommand(0,roll);
  sd.set_angleCommand(1,pitch);
  sd.set_angleCommand(2,yaw-90.0);


  // ------------------ Print Euler
  #ifdef EULER_DATA
    Serial << "Pitch:\t" << pitch << "\t\tRoll:\t" << roll << "\t\tYaw:\t" << yaw << "\t\tAltitude:\t" << alti << endl << endl;
  #endif

  #ifdef QUAT_DATA
    Serial << q0 << " " << q1 << " " << q2 << " " << q3 << " " << alti << endl;
  #endif

  // ------------------ Print Rad
  #ifdef PROCESSING
    roll = fusion.getRollRadians();
    pitch = fusion.getPitchRadians();
    yaw = fusion.getYawRadians();
    Serial  << pitch << ":" << roll << ":" << yaw << endl;
  #endif
  // ------------------ Plot
  #ifdef SERIAL_PLOTTER
    Serial << pitch << " " << roll << " " << yaw << " " << alti*100 << endl;
  #endif

  


  
  delay(10);
  //delay(500); // For readability
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
  delay(500);
  sd.set_angleCommand(0,-90.0);
  delay(500);
  sd.set_angleCommand(0,0.0);
}

void AHRS_step() {
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, deltat);  //Mahony filter is suggested if no compass is available
  // fusion.MadgwickUpdate(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz, deltat);  //else use Madgwick's filter

  roll      = fusion.getRoll();
  pitch     = fusion.getPitch();
  yaw       = fusion.getYaw();

  fusion.getQuatArray(quatArray);
  q0 = quatArray[0];
  q1 = quatArray[1];
  q2 = quatArray[2];
  q3 = quatArray[3];

  radRoll   = fusion.getRollRadians();
  radPitch  = fusion.getPitchRadians();

  // Approximate altitude with attitude and low-pass filter
  //alti = rf.range * fast_cosinus(radRoll) * fast_cosinus(radPitch);
  // alti = 0.6 * (rf.range * cos(radRoll) * cos(radPitch)) + 0.4 * oldAlti;
  alti = 0.6 * rf.range + 0.4 * oldAlti;
  oldAlti = alti;
}

/*float fast_cosinus(float angle) {
  float anglesq = angle*angle;
  float cosinus = 1 - anglesq * 0.5 + anglesq*anglesq * 0.0416666666667; // Series expansion of cosinus in 0. Maybe unnecessary ? Should test the time it takes to compute the real cosinus...
  return cosinus;
}*/

void print_raw() {
  // Print out the values
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


void initialize_serial() {
  bool is_serial_init = false;
  #ifdef RAW_DATA
    Serial.begin(115200);
    wait_for_serial();
    is_serial_init = true;
  #endif
  #ifdef EULER_DATA
    if (!is_serial_init){
      Serial.begin(115200);
      wait_for_serial();
      is_serial_init = true;
    }
  #endif
  #ifdef QUAT_DATA
    if (!is_serial_init){
      Serial.begin(115200);
      wait_for_serial();
      is_serial_init = true;
    }
  #endif
  #ifdef PROCESSING
    if (!is_serial_init){
      Serial.begin(115200);
      wait_for_serial();
      is_serial_init = true;
    }
  #endif
  #ifdef SERIAL_PLOTTER
    if (!is_serial_init){
      Serial.begin(115200);
      wait_for_serial();
      is_serial_init = true;
    }
  #endif
  #ifdef VERBOSE
    if (!is_serial_init){
      Serial.begin(115200);
      wait_for_serial();
      is_serial_init = true;
    }
  #endif
  Serial.println("Serial link initialized");
  #ifdef VERBOSE
    Serial.println("is verbose");
  #endif
}

void wait_for_serial() {
  while (!Serial)
    delay(10); // Pauses the controller until a serial console opens
  #ifdef VERBOSE
    Serial.println("Found serial ! So happy !");
  #endif
}
