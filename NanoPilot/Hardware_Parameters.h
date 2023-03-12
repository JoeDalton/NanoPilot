//=============================================================================================
// Hardware_Parameters.h
//=============================================================================================
//
// 
//
//  Date			  Author			Notes
//  2023-03-06  G. Lavabre  Added suport for both cards and both sensor combos
//
//=============================================================================================
#ifndef HardParam_h
#define HardParam_h

//================================ Set hardware configuration
//Board type
#define BOARD_TYPE_NANO33
//#define BOARD_TYPE_MEGA2560

//IMU config
//#define IMU_CONFIG_LSM9DS1
#define IMU_CONFIG_MPU6050_HMC5883L

extern float magXScale, magXBias, magYScale, magYBias, magZScale, magZBias; // Magnetometer bias and scale, calibrated with NP_Calibration
extern float gyrXScale, gyrXBias, gyrYScale, gyrYBias, gyrZScale, gyrZBias; // Gyrometer bias and scale, calibrated with NP_Calibration
extern float accXScale, accXBias, accYScale, accYBias, accZScale, accZBias; // Magnetometer bias and scale, calibrated with NP_Calibration



#endif
