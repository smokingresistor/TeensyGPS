#ifndef LSM9DS0_h
#define LSM9DS0_h
#include <SFE_LSM9DS0.h>
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include "BMsg838.h"

#define ACC_SCALE 1000
#define YAW_SCALE 100

extern BMsg838 gps;
extern int INT1XM;
extern int INT2XM;
extern int DRDYG;
extern float ax, ay, az, gx, gy, gz, mx, my, mz; 
extern float heading, roll, pitch, yaw, temp, inclination, yaw_rate; 
extern float bmp280_pressure;
extern float q[4];

extern void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
extern float get_declination(float lat, float lon);
extern void sensor_9dof_configure();
extern void sensor_9dof_read();
extern void print_9dof_data();
void scale_accel_16g();

#endif
