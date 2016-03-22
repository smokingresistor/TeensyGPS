#include "MatrixMath.h"
//#include <SoftwareSerial.h>    //in case arduino board
#include "BMsg838.h"
#include "KalmanFilter.h"
//#include "KalmanFilterVA"
#include "GPSSerialMessageCom.h"
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROMex.h>
#include "LogLib.h"
#include <ArduinoJson.h>
#include <FlexCAN.h>
#include <Metro.h>

const String fixmodmask[]={"no fix", "2D", "3D", "3D+DGNSS"};
/* This sample code demonstrates the normal use of the binary message of 
   SkyTraq Venus 8 GNSS Receiver.
   this is used on teensy3.1
*/
  union conv 
  {
    int32_t f;
    uint8_t b[4];
  };
  
    union conv_short 
  {
    int16_t f;
    uint8_t b[2];
  };
 
static CAN_message_t can_pos,can_nav,can_pos_fil,can_nav_fil, can_lap;
FlexCAN CANbus(1000000);

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);
// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
  
/*gps interfacce relate class*/
BMsg838 gps;
Metro beacon_timeout = Metro(30000);
Metro beacon_trigger = Metro(250);
Metro file_time_cut= Metro(300000);

/*kalman filter class*/
KalmanFilter filter;
KalmanFilterVA filterVA;

union conv lat, lon, lat_fil, lon_fil;
union conv_short alt, vel, alt_fil, vel_fil;

int led = 13;
int pin_beacon=21;

boolean led_on=1;
boolean can_output=1;
boolean beacon_output=1;
boolean file_cutted=false;

double t0,t1,dt=0;
uint8_t sats,fix;
const float lat_beacon[4]={45.618967, 2.760700  , 14.958108,    45.533341};
const float lon_beacon[4]={9.281226,  101.738322, 103.085608,   10.219222};
//                        0 Monza     1 Sepang   2 Buri ram     3 Via XX Settembre

const float beacon_distance=9; //radious in m for lap beacon trigger

boolean log_en;
int newlog, rate, max_filesize, filesize, log_type, trig, intv;
float blat, blong, btime, btol, course_angle, trigv, min_val, max_val;
String UTC_Time;

void setup()
{
  Serial.begin(115200); 
  lsm.begin();
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  delay(1000);
  pinMode(led, OUTPUT);
  LogSetup();
  can_output = CNF[1];
  newlog = CNF[2];
  rate = CNF[3];
  max_filesize = CNF[4];
  blat = CNF[5];
  blong = CNF[6];
  btime = CNF[7];
  btol = CNF[8];
  log_type = CNF[9];
  trig = CNF[10];
  trigv = CNF[11];
  intv = CNF[12];
  min_val = CNF[13];
  max_val = CNF[14];
 
  file_time_cut.interval(btime*1000);
  Serial.println(filesize*1048576);
  Serial.println(newlog);
  if (can_output) 
    {       
       can_pos.id=0x302;
       can_pos.len = 8;
       can_nav.id=0x304;
       can_nav.len = 8;
       can_pos_fil.id=0x312;
       can_pos_fil.len = 8;
       can_nav_fil.id=0x314;
       can_nav_fil.len = 8;
       can_lap.id=0x308;
       can_lap.len=8;
       CANbus.begin(); 
       digitalWrite(led,led_on); 
       Serial.print("Canbus output enabled on fames:");
       Serial.print(can_pos.id,HEX);
       Serial.print(",");
       Serial.print(can_nav.id,HEX);   
       Serial.print(",");
       Serial.print(can_pos_fil.id,HEX);
       Serial.print(",");
       Serial.println(can_nav_fil.id,HEX);     
    }
    
  if (beacon_output==true)
    { 
      pinMode(pin_beacon, OUTPUT);
      digitalWrite(pin_beacon,HIGH);
      Serial.println("Lap beacon enabled");
    }
  Serial2.begin(115200);
  char messagetype[64];
  memset(messagetype,0,64);
  Serial.print("Testing BMsg838 binary message library v. "); 
  Serial.println();
  
  Serial.print(" BMsg838 System reset.\r\n"); 
  SendBinaryMessagetoGPSreceiver(gps.ResetGNSS(1, 15, 6, 9,11, 30, 25, 20, 133, 1200), gps.SendStream,gps.RecVBinarybuf,0,2000);
  SendBinaryMessagetoGPSreceiver(gps.SetSerialPort(115200, 1), gps.SendStream,gps.RecVBinarybuf,0,2000);
  SendBinaryMessagetoGPSreceiver(gps.SetBinaryMessagetype(), gps.SendStream,gps.RecVBinarybuf,0,2000); 
  SendBinaryMessagetoGPSreceiver(gps.SetPositionRate(rate), gps.SendStream,gps.RecVBinarybuf,0,2000);
  if(SendBinaryMessagetoGPSreceiver(gps.GetSoftVersion(), gps.SendStream,gps.RecVBinarybuf,0,2000)==3);
      if(waitingRespondandReceive(gps.RecVBinarybuf,0x80,2000)>7){
              BinaryRecvMsgtype(messagetype,gps.RecVBinarybuf);
              Serial.println(messagetype); 
              GPSSoftVersiondata* versioninfo=gps.ResponseSoftVersion();
              printSoftversion(versioninfo);        
      }     
} 

//receiv Navigation binary data from GPS receiver and find positopn and velocity data 
//and after do kalmanfiltering 

void loop()
{
  dataFile = SD.open(namefile, FILE_WRITE);
  while (1)
  {
      int ret=0;
      char messagetype[64];
      memset(messagetype,0,64);
      if(Serial2.available()){
           ret=waitingRespondandReceive(gps.RecVBinarybuf,0xA8,2000); 
           if(ret>7){                
                if(!GPSNavigationMsgProcessing(&(gps.venus838data_raw),&(gps.venus838data_filter),gps,&filter, &filterVA))
                {
                     Serial.println("Checksum error has been occured\n");
                     checksums++;
                }   
                /*
                Serial.print(millis());
                Serial.print(";");     
                Serial.print(gps.venus838data_raw.Latitude,12);
                Serial.print(";");
                Serial.print(gps.venus838data_raw.Longitude,12);
                Serial.print(";");
                Serial.print(gps.venus838data_raw.SealevelAltitude,12);
                Serial.print(";");
                Serial.print(gps.venus838data_raw.velocity,12);
                Serial.print(";");
                Serial.print(gps.venus838data_filter.Latitude,12);
                Serial.print(";");
                Serial.print(gps.venus838data_filter.Longitude,12);
                Serial.print(";");
                Serial.print(gps.venus838data_filter.SealevelAltitude,12);
                Serial.print(";");
                Serial.print(gps.venus838data_filter.velocity,12);
                Serial.print(";");
                Serial.print(fixmodmask[gps.venus838data_raw.fixmode]);
                Serial.print(";");
                Serial.print(gps.venus838data_raw.NumSV);
                Serial.print(";");
                Serial.print(checksums);
                Serial.println(";");
                */
    Serial.print("UTC time: ");
    UTC_Time = GetUTCTime(gps.venus838data_raw.gps_week, gps.venus838data_raw.timeofweek);
    Serial.println(UTC_Time);
    course_angle = gps.course_to(gps.venus838data_filter.Longitude, gps.venus838data_filter.Latitude, 0 , 0);          
    sensor_9dof_read();
    if (can_output)
        can_send();              
    if (beacon_output)
        check_beacon_dist();
    if (log_output){
        if((newlog==0)&&(filesize > (max_filesize*1048576))){ //file size more than max 
            Serial.println("File's size exceeds max size"); 
            dataFile.flush();
            dataFile.close(); 
            Serial.println("Create new log");
            create_newlog();
        }//if
        log_en = false; //disable before check
        if (log_type==0) // continuous log
            log_en = true;
        if (log_type==1) //trigger log
            log_en = check_triggers();
        if (log_type==2) //interval log
            log_en = check_intervals();
        if (log_en){
            led_on=!led_on;
            digitalWrite(led,led_on);
            LogTPV(); // log TPV object
            LogATT(); // log ATT object  
        }//if      
//                Serial.println("filtered data:\n");                
//                printpositionfloatformat(gps.venus838data_filter.Latitude, 10000000, "  Latitude= ", "degree");
//                printpositionfloatformat(gps.venus838data_filter.Longitude, 10000000, "  Longitude= ", "degree");
//                printpositionfloatformat(gps.venus838data_filter.SealevelAltitude, 100, "  SealevelAltitude= ", "meter");
//                printpositionfloatformat(gps.venus838data_filter.velocity, 100, "  velocity= ", "meter/second");
//                printpositionfloatformat(gps.venus838data_filter.receivedtime, 1, "  receivedtime= ", "second");
//                Serial.println("raw data:\n");
//                printpositionfloatformat(gps.venus838data_raw.Latitude, 10000000, "  Latitude= ", "degree");
//                printpositionfloatformat(gps.venus838data_raw.Longitude, 10000000, "  Longitude= ", "degree");
//                printpositionfloatformat(gps.venus838data_raw.SealevelAltitude, 100, "  SealevelAltitude= ", "meter");
//                printpositionfloatformat(gps.venus838data_raw.velocity, 100, "  velocity= ", "meter/second");
//                
//                Serial.println("Added data:\n");
//                const String fixmodmask[]={"no fix", "2D", "3D", "3D+DGNSS"};
//                Serial.println("fixmode=");
//                Serial.println(fixmodmask[gps.venus838data_raw.fixmode]);
//                Serial.println("NumSV=");
//                Serial.println(gps.venus838data_raw.NumSV);
//                
//                printpositionfloatformat(gps.venus838data_raw.GDOP, 100, "  GDOP= ", "");
//                printpositionfloatformat(gps.venus838data_raw.PDOP, 100, "  PDOP= ", "");
//                printpositionfloatformat(gps.venus838data_raw.HDOP, 100, "  HDOP= ", "");
//                printpositionfloatformat(gps.venus838data_raw.VDOP, 100, "  VDOP= ", "");
//                printpositionfloatformat(gps.venus838data_raw.TDOP, 100, "  TDOP= ", "");
      
         }//if
       }//if ret
    }//if serial                   
  }//while
}//loop  


void check_beacon_dist(){
   float distance;
   distance = gps.distance_between(gps.venus838data_raw.Latitude,gps.venus838data_raw.Longitude,blat,blong); // distance to beacon
   Serial.print("Distance: ");
   Serial.println(distance);
   if ((((distance < btol)&&(beacon_timeout.check()==true)))||(file_time_cut.check()==true)){ //check distance or time_cut
     Serial.println("Beacon Trigger");
     if (file_time_cut.check()==true)
           Serial.println("Time Cut Trigger"); 
     //Rise level of beacon pin output or lowers for McLaren
     digitalWrite(pin_beacon,LOW); 
     if (can_output){
         can_lap.buf[0]=1; 
     }                 
     if (sd_datalog && log_output) {
         if (newlog==1) {
             dataFile.flush();
             dataFile.close(); 
             Serial.println("Create new log");
             create_newlog();
         }
     }
     beacon_timeout.reset();
     beacon_trigger.reset();
     file_time_cut.reset();
  }
  else {
     if (beacon_trigger.check()==true) {
          digitalWrite(pin_beacon,HIGH);
          can_lap.buf[0]=0;
     }
  }
}

boolean check_intervals(){
  if (intv==0)
      log_en = check_constr(min_val, millis()/1000.0, max_val);
  if (intv==1){
      float distance = gps.distance_between(gps.venus838data_raw.Latitude,gps.venus838data_raw.Longitude,blat,blong);
      log_en = check_constr(min_val, distance, max_val);
  }
  if (intv==2)
      log_en = check_constr(min_val, gps.venus838data_filter.velocity*3.6, max_val);  
  return log_en;
}

boolean check_triggers(){
  if (trig==0) 
      log_en = check_trval(gps.venus838data_filter.Longitude, 0.0001);
  if (trig==1)
      log_en = check_trval(gps.venus838data_filter.Latitude, 0.0001);
  if (trig==2)
      log_en = check_trval(gps.venus838data_filter.SealevelAltitude, 0.0001);
  if (trig==3)
      log_en = check_trval(gps.venus838data_filter.velocity, 0.01);
  return log_en;
}

boolean check_trval(float value, float offset){
  if (((1-offset)*trigv < value)&&(value < (1+offset)*trigv))
    return true;
  return false;
}

boolean check_constr(float value, float min, float max){
  if ((min < value)&&(value < max))
    return true;
  return false;
}

void can_send(){
  lat.f=gps.venus838data_raw.Latitude*1E7; 
  lat_fil.f=gps.venus838data_filter.Latitude*1E7;
  lon.f=gps.venus838data_raw.Longitude*1E7;
  lon_fil.f=gps.venus838data_filter.Longitude*1E7;
  alt.f=gps.venus838data_raw.SealevelAltitude*10;
  alt_fil.f=gps.venus838data_filter.SealevelAltitude*10;
  vel.f=gps.venus838data_raw.velocity*10*3.6;
  vel_fil.f=gps.venus838data_filter.velocity*10*3.6;
  //raw data
  can_pos.buf[0]=lat.b[3];
  can_pos.buf[1]=lat.b[2];
  can_pos.buf[2]=lat.b[1];
  can_pos.buf[3]=lat.b[0];
  can_pos.buf[4]=lon.b[3];
  can_pos.buf[5]=lon.b[2];
  can_pos.buf[6]=lon.b[1];
  can_pos.buf[7]=lon.b[0];
  can_nav.buf[0]=alt.b[1];
  can_nav.buf[1]=alt.b[0];
  can_nav.buf[2]=vel.b[1];
  can_nav.buf[3]=vel.b[0];
  //filter data
  can_pos_fil.buf[0]=lat_fil.b[3];
  can_pos_fil.buf[1]=lat_fil.b[2];
  can_pos_fil.buf[2]=lat_fil.b[1];
  can_pos_fil.buf[3]=lat_fil.b[0];
  can_pos_fil.buf[4]=lon_fil.b[3];
  can_pos_fil.buf[5]=lon_fil.b[2];
  can_pos_fil.buf[6]=lon_fil.b[1];
  can_pos_fil.buf[7]=lon_fil.b[0];
  can_nav_fil.buf[0]=alt_fil.b[1];
  can_nav_fil.buf[1]=alt_fil.b[0];
  can_nav_fil.buf[2]=vel_fil.b[1];
  can_nav_fil.buf[3]=vel_fil.b[0];
  sats=gps.venus838data_raw.NumSV;
  fix=gps.venus838data_raw.fixmode;
  fix=fix<<4;
  can_nav.buf[7]=fix|sats; 
  can_nav_fil.buf[7]=can_nav.buf[7];     
  CANbus.write(can_pos);  
  CANbus.write(can_nav); 
  CANbus.write(can_pos_fil);  
  CANbus.write(can_nav_fil);
  CANbus.write(can_lap);               
}

void sensor_9dof_read()
{
  float prev_heading, period, prev_time;
  sensors_vec_t   orientation;
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  if (ahrs.getOrientation(&orientation))
    {
       att.heading = orientation.heading;
       att.pitch = orientation.pitch;
       att.roll = orientation.roll;
    }
  att.dip = get_declination (gps.venus838data_filter.Latitude, gps.venus838data_raw.Longitude);
  period = (millis() - prev_time)/1000.0;
  prev_time = millis();
  att.yaw = (att.heading - prev_heading)/period;
  prev_heading =  att.heading;
  att.mag_x = mag.magnetic.x;
  att.mag_y = mag.magnetic.y;
  att.mag_z = mag.magnetic.z;
  att.mag_len = sqrt(sq(att.mag_x)+sq(att.mag_x)+sq(att.mag_z));
  att.acc_x = accel.acceleration.x;
  att.acc_y = accel.acceleration.y;
  att.acc_z = accel.acceleration.z;
  att.acc_len = sqrt (sq(att.acc_x)+sq(att.acc_y)+sq(att.acc_z));
  att.gyro_x = gyro.gyro.x;
  att.gyro_y = gyro.gyro.y;
  // print out accelleration data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");

  // print out magnetometer data
  Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");
  
  // print out gyroscopic data
  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");

  // print out temperature data
  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");
  /* 'orientation' should have valid .roll and .pitch fields */
  Serial.print(F("Orientation: "));
  Serial.print(orientation.roll);
  Serial.print(F(" "));
  Serial.print(orientation.pitch);
  Serial.print(F(" "));
  Serial.print(orientation.heading);
  Serial.println(F(""));
  Serial.println("**********************\n");
}





