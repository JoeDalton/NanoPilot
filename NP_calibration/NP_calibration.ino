//=============================================================================================
// NP_IMU.cpp
//=============================================================================================
//
// Generic calibrator for imus
//
//  Date        Author      Notes
//  2023-03-06  G. Lavabre  Only magnetometer for now
//
//=============================================================================================





#include "NP_IMU.h" // NP_IMU


NP_IMU imu;



float calMagData[3][2];
float calGyroData[3];
float calAccelData[3][2];
bool changed = false;
bool done = false;
int t = 0;
int c = 0;
float magXScale, magXBias, magYScale, magYBias, magZScale, magZBias;
float accXScale, accXBias, accYScale, accYBias, accZScale, accZBias;
bool magCalibrated = false;
bool gyroCalibrated = false;
bool accelCalibrated = false;
int counter = 0;

int   counter1  = 0;
int   counter2  = 0;
int   counter3  = 0;
bool  done1     = false;
bool  done2     = false;
bool  done3     = false;



void setup() {
  Serial.begin(115200);
  while (!Serial){
    delay(10);} // Pauses the controller until a serial console opens
  imu.init();
  
  Serial.println("This program provides calibration settings for your imu.");
  Serial.println("It will take about 2 minutes to complete");
  Serial.println("");
  Serial.println("Magnetometer calibration will begin in 2 seconds.");
  Serial.println("When prompted, move the magnetometer in all directions until the calibration is complete.");

  delay(2000);
  Serial.println("Keep moving your sensor around.");
  c=millis();
  
  // DEBUG
  //magCalibrated = true;
  //gyroCalibrated = true;
  //accelCalibrated = true;
  
}

void loop() {

  imu.get_raw();

  if (!magCalibrated) {
  changed = false;

  if(imu.mx < calMagData[0][0]) {
    calMagData[0][0] = imu.mx;
    changed = true;
  }
  if(imu.mx > calMagData[0][1]) {
    calMagData[0][1] = imu.mx;
    changed = true;
  }

  if(imu.my < calMagData[1][0]) {
    calMagData[1][0] = imu.my;
    changed = true;
  }
  if(imu.my > calMagData[1][1]) {
    calMagData[1][1] = imu.my;
    changed = true;
  }

  if(imu.mz < calMagData[2][0]) {
    calMagData[2][0] = imu.mz;
    changed = true;
  }
  if(imu.mz > calMagData[2][1]) {
    calMagData[2][1] = imu.mz;
    changed = true;
  }

  if (changed && !done) {
    c = millis();
  }
    t = millis();
  
  
  if ( (t - c > 10000) && !done) {
    done = true;
    Serial.println("Done");
    Serial.println("Computing bias and scale");
    
    magXBias    = (calMagData[0][1] + calMagData[0][0]) * 0.5;
    magYBias    = (calMagData[1][1] + calMagData[1][0]) * 0.5;
    magZBias    = (calMagData[2][1] + calMagData[2][0]) * 0.5;

    magXScale   = 1.0 / (calMagData[0][1] - calMagData[0][0]);
    magYScale   = 1.0 / (calMagData[1][1] - calMagData[1][0]);
    magZScale   = 1.0 / (calMagData[2][1] - calMagData[2][0]);
    
    Serial.println("DONE. Copy the line below and paste it into Hardware_Parameters.h");
    Serial.println();
    Serial.println("// Magnetometer calibration");
    Serial.print("float magXScale = ");
    Serial.print(magXScale,6);
    Serial.print("; ");
    Serial.print("float magYScale = ");
    Serial.print(magXScale,6);
    Serial.print("; ");
    Serial.print("float magZScale = ");
    Serial.print(magZScale,6);
    Serial.println(";");
    
    Serial.print("float magXBias  = ");
    Serial.print(magXBias,6);
    Serial.print(";  ");
    Serial.print("float magYBias  = ");
    Serial.print(magYBias,6);
    Serial.print(";  ");
    Serial.print("float magZBias  = ");
    Serial.print(magZBias,6);
    Serial.println(";");
    magCalibrated = true;
    Serial.println("Gyro calibration will begin in 3 seconds.");
    delay(3000);
    Serial.println("Keep the gyrometer still until the calibration is complete.");
    c=millis();
    }
  }

  if (magCalibrated && !gyroCalibrated){
    done = false;
    calGyroData[0] += imu.gx; // Accumulate gyro measurements
    calGyroData[1] += imu.gy;
    calGyroData[2] += imu.gz;
    counter++;
    t=millis();

    if ( (t - c > 20000) && !done) {
      done = true;
      calGyroData[0] /= counter; // Divide accumulated measureres by number of observations
      calGyroData[1] /= counter;
      calGyroData[2] /= counter;

      Serial.println("DONE. Copy the line below and paste it into Hardware_Parameters.h");
      Serial.println();
      Serial.println("// Gyrometer calibration");
      Serial.print("float gyrXBias = ");
      Serial.print(calGyroData[0],6);
      Serial.print("; ");
      Serial.print("float gyrYBias = ");
      Serial.print(calGyroData[1],6);
      Serial.print("; ");
      Serial.print("float gyrZBias = ");
      Serial.print(calGyroData[2],6);
      Serial.println(";");
      done = false;
      gyroCalibrated = true;

      Serial.println("");
      Serial.println("Accelero calibration will begin in 2 seconds.");
      Serial.println("Put z-axis upwards. You have 3 seconds to get in position");
      delay(3000);
      Serial.println("Keep z-axis steadily upwards until the calibration is complete.");
      c=millis();
    }
  }

  if (magCalibrated && gyroCalibrated && !accelCalibrated){
      
    if (!done1){
      counter1++;
      t=millis();
      calAccelData[0][0] += imu.ax; // accumulate zero
      calAccelData[1][0] += imu.ay; // accumulate zero
      calAccelData[2][1] += imu.az; // accumulate max value
      
      if ( (t - c > 10000) && !done1) {
        done1 = true;
        Serial.println("");
        Serial.println("Put y-axis upwards. You have 3 seconds to get in position");
        delay(3000);
        Serial.println("Keep y-axis steadily upwards until the calibration is complete.");
        c=millis();



        
      }
    }
    if (done1 && !done2){
      
      counter2++;
      t=millis();
      calAccelData[0][0] += imu.ax; // accumulate zero
      calAccelData[1][1] += imu.ay; // accumulate max value
      calAccelData[2][0] += imu.az; // accumulate zero
      
      if ( (t - c > 10000) && !done2) {
        done2 = true;
        Serial.println("");
        Serial.println("Put x-axis upwards. You have 3 seconds to get in position");
        delay(3000);
        Serial.println("Keep x-axis steadily upwards until the calibration is complete.");
        c=millis();
      }
    }
    if (done1 && done2 && !done3){
      
      counter3++;
      t=millis();
      calAccelData[0][1] += imu.ax; // accumulate max value
      calAccelData[1][0] += imu.ay; // accumulate zero
      calAccelData[2][0] += imu.az; // accumulate zero
      
      if ( (t - c > 10000) && !done3) {
        done3 = true;
        Serial.println("Done");
        accelCalibrated=true;
      }
    }


    if (accelCalibrated){
    Serial.println("Computing bias and scale");

    calAccelData[0][0] /= (counter1 + counter2); // zero time
    calAccelData[0][1] /= (counter3);            // max time
    
    calAccelData[1][0] /= (counter3 + counter1); // zero time
    calAccelData[1][1] /= (counter2);            // max time

    calAccelData[2][0] /= (counter2 + counter3); // zero time
    calAccelData[2][1] /= (counter1);            // max time
    
    accXBias  = calAccelData[0][0];
    accYBias  = calAccelData[1][0];
    accZBias  = calAccelData[2][0];

    accXScale = 9.81 / (calAccelData[0][1] - calAccelData[0][0]);
    accYScale = 9.81 / (calAccelData[1][1] - calAccelData[1][0]);
    accZScale = 9.81 / (calAccelData[2][1] - calAccelData[2][0]);

    Serial.println("DONE. Copy the line below and paste it into Hardware_Parameters.h");
    Serial.println();
    Serial.println("// Accelerometer calibration");
    Serial.print("float accXScale = ");
    Serial.print(accXScale,6);
    Serial.print("; ");
    Serial.print("float accYScale = ");
    Serial.print(accXScale,6);
    Serial.print("; ");
    Serial.print("float accZScale = ");
    Serial.print(accZScale,6);
    Serial.println(";");
    
    Serial.print("float accXBias  = ");
    Serial.print(accXBias,6);
    Serial.print(";  ");
    Serial.print("float accYBias  = ");
    Serial.print(accYBias,6);
    Serial.print(";  ");
    Serial.print("float accZBias  = ");
    Serial.print(accZBias,6);
    Serial.println(";");
    }

/*

      done = true;
      calGyroData[0] /= counter; // Divide accumulated measureres by number of observations
      calGyroData[1] /= counter;
      calGyroData[2] /= counter;

      Serial.println("DONE. Copy the line below and paste it into Hardware_Parameters.h");
      Serial.println();
      

      Serial.println("// Gyrometer calibration");
      Serial.print("float gyroXBias = ");
      Serial.print(calGyroData[0],4);
      Serial.print("; ");
      Serial.print("float gyroYBias = ");
      Serial.print(calGyroData[1],4);
      Serial.print("; ");
      Serial.print("float gyroZBias = ");
      Serial.print(calGyroData[2],4);
      Serial.println(";");
      done = false;
      gyroCalibrated = true;

      Serial.println("Accelero calibration will begin in 2 seconds.");
      Serial.println("Keep z-axis steadily upwards until the calibration is complete.");
      delay(2000);
      c=millis();



*/







    
  }

    
}
