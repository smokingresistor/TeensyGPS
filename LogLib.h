#ifndef LogLib_h
#define LogLib_h
#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <EEPROMex.h>
#include "BMsg838.h"
#include <stdlib.h>

#define TIMECONV_JULIAN_DATE_START_OF_GPS_TIME (2444244.5)  // [days]
#define TIMECONV_DAYS_IN_JAN 31
#define TIMECONV_DAYS_IN_MAR 31
#define TIMECONV_DAYS_IN_APR 30
#define TIMECONV_DAYS_IN_MAY 31
#define TIMECONV_DAYS_IN_JUN 30
#define TIMECONV_DAYS_IN_JUL 31
#define TIMECONV_DAYS_IN_AUG 31
#define TIMECONV_DAYS_IN_SEP 30
#define TIMECONV_DAYS_IN_OCT 31
#define TIMECONV_DAYS_IN_NOV 30
#define TIMECONV_DAYS_IN_DEC 31

extern int chipSelect;
extern int mosi;
extern int miso;
extern int sck;
extern int led;

extern Sd2Card card;
extern boolean led_on;
extern File dataFile;
extern boolean sd_datalog;
extern boolean file_open;
extern int fileNum;
extern char namefile[13];
extern const char* classConfig[3];
extern float CNF [17];
extern boolean TPV [21];
extern boolean ATT [18];

extern void LogSetup();
extern void incFileNum();
extern void create_newlog();
extern void LogTPV();
extern void LogATT();
extern String GetUTCTime (unsigned int gps_week, unsigned long gps_tow);

String null_add(int value);
void dataFloat(float value, int mode);

boolean TIMECONV_GetNumberOfDaysInMonth(
   const unsigned short year,        //!< Universal Time Coordinated    [year]
   const unsigned char month,        //!< Universal Time Coordinated    [1-12 months] 
   unsigned char* days_in_month      //!< Days in the specified month   [1-28|29|30|31 days]
   );
   
boolean TIMECONV_IsALeapYear(const unsigned short year );

extern boolean log_output;
extern int count;
extern int filesize;
extern int checksums;
extern BMsg838 gps;
extern String UTC_Time;
extern float course_angle; 

typedef struct DOF_DATA{
  float heading;
  float pitch;
  float yaw;
  float roll;
  float dip;
  float mag_len;
  float mag_x;
  float mag_y;
  float mag_z;
  float acc_len;
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float quat1;
  float quat2;
  float quat3;
  float quat4;
  float temp;
};

extern float ax, ay, az, gx, gy, gz, mx, my, mz; 
extern float heading, roll, pitch, yaw, temp, inclination; 
extern float q[4];
extern struct DOF_DATA att;

#endif
