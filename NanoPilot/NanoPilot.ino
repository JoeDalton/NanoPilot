#include "Streaming.h"    // needed for simpler Serial output https://github.com/geneReeves/ArduinoStreaming
#include "NP_SensorFusion.h"

//#include <Adafruit_Sensor.h>
//#include <Wire.h>
#include "NP_IMU.h"
#include "NP_ServoDriver.h"

SF fusion;
NP_IMU imu;
NP_ServoDriver sd;







//float gx, gy, gz, ax, ay, az, mx, my, mz;
//int   readiness;
float pitch, roll, yaw;
float deltat;

//#define RAW_DATA
//#define EULER_DATA
//#define PROCESSING
#define SERIAL_PLOTER
//#define VERBOSE


void setup(void) {
  Serial.begin(115200);
  wait_for_serial();
  sd.init();
  test_servos();
  
  imu.init();
  #ifdef SERIAL_PLOTER
    Serial << "pitch" << " " << "roll" << " " << "yaw" << endl;
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
  delay(100);
}





void loop() {
  // ------------------ Read sensors
  imu.get_raw();
  imu.correct();
  //DEBUG
  //imu.gx = 0; imu.gy = 0; imu.gz = 0;
  //imu.mx = 0; imu.my = 0; imu.mz = 0;
  

  // ------------------ Print raw sensor output
  #ifdef RAW_DATA
    print_raw();
  #endif

  // ------------------ Update AHRS
  update_AHRS();

  // ------------------ Move servos according to board orientation
  sd.set_angleCommand(0,roll);
  sd.set_angleCommand(1,pitch);
  sd.set_angleCommand(2,yaw-90.0);


  // ------------------ Print Euler
  #ifdef EULER_DATA
    Serial << "Pitch:\t" << pitch << "\t\tRoll:\t" << roll << "\t\tYaw:\t" << yaw << endl << endl;
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
    Serial << pitch << " " << roll << " " << yaw << endl;
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

void update_AHRS() {
  deltat = fusion.deltatUpdate();
  //fusion.MahonyUpdate(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, deltat);  //Mahony filter is suggested if no compass is available
  fusion.MadgwickUpdate(imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz, deltat);  //else use Madgwick's filter

  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();
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
}

/*
void get_imu() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
   if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(gx, gy, gz);
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }
}

void init_imu() {
    if (!IMU.begin()) {
      readiness = -1;
    }
}
*/
/*void set_mpu_range() {
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  #ifdef VERBOSE
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  #endif
}

void set_mpu_bandwidth() {
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  #ifdef VERBOSE
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  #endif
}

void test_mpu_presence() {
  #ifdef VERBOSE
    Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    readiness = -1;
    }
  Serial.println("MPU6050 Found!");
  #else
    mpu.begin();
  #endif
}*/
