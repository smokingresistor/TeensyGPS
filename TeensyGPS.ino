#include "MatrixMath.h"
//#include <SoftwareSerial.h>    //in case arduino board
#include "BMsg838.h"
#include "KalmanFilter.h"
//#include "KalmanFilterVA"
#include "GPSSerialMessageCom.h"
#include <SFE_LSM9DS0.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROMex.h>
#include "LogLib.h"
#include "LSM9DS0.h"
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
 
static CAN_message_t can_pos,can_nav,can_pos_fil,can_nav_fil, can_dof1, can_dof2, can_lap;
FlexCAN CANbus(1000000);

/*gps interfacce relate class*/
BMsg838 gps;
Metro beacon_timeout = Metro(30000);
Metro beacon_trigger = Metro(250);
Metro file_time_cut= Metro(300000);

/*kalman filter class*/
KalmanFilter filter;
KalmanFilterVA filterVA;

union conv lat, lon, lat_fil, lon_fil;
union conv_short alt, vel, alt_fil, vel_fil, course, lap_dist, dof_roll, dof_pitch, dof_yaw, pressure, acc_x, acc_y, acc_z, q1, q2, q3, q4;

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

struct DOF_DATA att;
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float heading, roll, pitch, yaw, temp, inclination; 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

//finish line segment
struct FLS_DATA{
  boolean en;
  boolean set_norm_low;
  float lat_a;
  float long_a;
  float lat_b;
  float long_b;
  int output_line;
  float max_speed;
  float min_speed;
  boolean check_min_speed;
  boolean check_max_speed;
}fls;

void setup()
{
  Serial.begin(115200); 
  sensor_9dof_configure();
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
  fls.en = CNF[15];
  fls.set_norm_low = CNF[16];
  fls.lat_a = CNF[17];
  fls.long_a = CNF[18];
  fls.lat_b = CNF[19];
  fls.long_b = CNF[20];
  fls.output_line = CNF[21];
  fls.max_speed = CNF[22];
  fls.min_speed = CNF[23];
  fls.check_min_speed = CNF[24];
  fls.check_max_speed = CNF[25];
  file_time_cut.interval(btime*1000);
  Serial.println(filesize*1048576);
  Serial.println(newlog);
  if (can_output) 
    {       
       can_pos.id=0x301;
       can_pos.len = 8;
       can_nav.id=0x302;
       can_nav.len = 8;
       can_pos_fil.id=0x303;
       can_pos_fil.len = 8;
       can_nav_fil.id=0x304;
       can_nav_fil.len = 8;
       can_dof1.id=0x305;
       can_dof1.len = 8;
       can_lap.id=0x306;
       can_lap.len=8;
       can_dof2.id=0x307;
       can_dof2.len=8;
       CANbus.begin(); 
       digitalWrite(led,led_on); 
       Serial.print("Canbus output enabled on frames:");
       Serial.print(can_pos.id,HEX);
       Serial.print(",");
       Serial.print(can_nav.id,HEX);   
       Serial.print(",");
       Serial.print(can_pos_fil.id,HEX);
       Serial.print(",");
       Serial.println(can_nav_fil.id,HEX);
       Serial.print(",");
       Serial.print(can_dof1.id,HEX);
       Serial.print(",");
       Serial.print(can_lap.id,HEX);   
       Serial.print(",");
       Serial.print(can_dof2.id,HEX);  
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
                //     Serial.println("Checksum error has been occured\n");
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
    if (beacon_output)
        check_beacon_dist();
    if (can_output)
        can_send(); 
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
   boolean finish_line_cross;
   //distance = gps.distance_between(gps.venus838data_filter.Latitude,gps.venus838data_filter.Longitude,blat,blong); // distance to beacon
   finish_line_cross = check_line_crossing(fls.lat_a, fls.lat_b, fls.long_a, fls.long_b, gps.venus838data_filter.Latitude,gps.venus838data_filter.Longitude);
//   Serial.print("Distance: ");
//   Serial.println(distance);
   if (((finish_line_cross && (beacon_timeout.check()==true)))||(file_time_cut.check()==true)){ //check distance or time_cut
     Serial.println("Finish line crossing");
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
  if ((((1-offset)*trigv) < value)&&(value < ((1+offset)*trigv)))
    return true;
  return false;
}

boolean check_constr(float value, float min, float max){
  if ((min < value)&&(value < max))
    return true;
  return false;
}

boolean check_line_crossing(float lat_a, float lat_b, float lon_a, float lon_b, float lat, float lon){
  float min_lat, max_lat, min_lon, max_lon, delta;
  delta = 0,005/111; //delta 5 meters
  if (lat_a > lat_b){
      min_lat = lat_b - delta;
      max_lat = lat_a + delta;
  }
  else{
      min_lat = lat_a - delta;
      max_lat = lat_b + delta;
  };
  if (lon_a > lon_b){
      min_lon = lon_b - delta;
      max_lon = lon_a + delta;
  }
  else{
      min_lon = lon_a - delta;
      max_lon = lon_b + delta;
  };
  if ((lat >= min_lat)&&(lat <= max_lat)&&(lon >= min_lon)&&(lon <= max_lon))
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
  course.f=course_angle*100;
  sats=gps.venus838data_raw.NumSV;
  fix=gps.venus838data_raw.fixmode;
  fix=fix<<4;
  lap_dist.f = 0*4;          //to do
  dof_roll.f = att.roll;
  dof_pitch.f = att.pitch;
  dof_yaw.f = att.yaw*10;
  pressure.f = 0;          //to do
  acc_x.f = att.acc_x*100;
  acc_y.f = att.acc_y*100;
  acc_z.f = att.acc_z*100;
  q1.f = att.quat1;
  q2.f = att.quat2;
  q3.f = att.quat3;
  q4.f = att.quat4;
  //raw data
  //1 frame
  can_pos.buf[0]=lat.b[3];
  can_pos.buf[1]=lat.b[2];
  can_pos.buf[2]=lat.b[1];
  can_pos.buf[3]=lat.b[0];
  can_pos.buf[4]=lon.b[3];
  can_pos.buf[5]=lon.b[2];
  can_pos.buf[6]=lon.b[1];
  can_pos.buf[7]=lon.b[0];
  //2 frame - filter data
  can_pos_fil.buf[0]=lat_fil.b[3];
  can_pos_fil.buf[1]=lat_fil.b[2];
  can_pos_fil.buf[2]=lat_fil.b[1];
  can_pos_fil.buf[3]=lat_fil.b[0];
  can_pos_fil.buf[4]=lon_fil.b[3];
  can_pos_fil.buf[5]=lon_fil.b[2];
  can_pos_fil.buf[6]=lon_fil.b[1];
  can_pos_fil.buf[7]=lon_fil.b[0];
  //3 frame
  can_nav.buf[0]=alt.b[1];
  can_nav.buf[1]=alt.b[0];
  can_nav.buf[2]=vel.b[1];
  can_nav.buf[3]=vel.b[0];
  can_nav.buf[4]=course.b[1];
  can_nav.buf[5]=course.b[0];
  can_nav.buf[6]=checksums;
  can_nav.buf[7]=fix|sats; 
  //4 frame
  can_nav_fil.buf[0]=alt_fil.b[1];
  can_nav_fil.buf[1]=alt_fil.b[0];
  can_nav_fil.buf[2]=vel_fil.b[1];
  can_nav_fil.buf[3]=vel_fil.b[0];
  can_nav_fil.buf[4]=course.b[1];
  can_nav_fil.buf[5]=course.b[0];
  can_nav_fil.buf[6]=lap_dist.b[1];
  can_nav_fil.buf[7]=lap_dist.b[0];  
  //5 frame
  can_dof1.buf[0]=dof_roll.b[1];
  can_dof1.buf[1]=dof_roll.b[0];
  can_dof1.buf[2]=dof_pitch.b[1];
  can_dof1.buf[3]=dof_pitch.b[0];
  can_dof1.buf[4]=dof_yaw.b[1];
  can_dof1.buf[5]=dof_yaw.b[0];
  can_dof1.buf[6]=pressure.b[1];
  can_dof1.buf[7]=pressure.b[0];  
  //6 frame
  can_lap.buf[1]=att.temp;
  can_lap.buf[2]=acc_x.b[1];
  can_lap.buf[3]=acc_x.b[0];
  can_lap.buf[4]=acc_y.b[1];
  can_lap.buf[5]=acc_y.b[0];
  can_lap.buf[6]=acc_z.b[1];
  can_lap.buf[7]=acc_z.b[0];  
  //7 frame
  can_dof2.buf[0]=q1.b[1];
  can_dof2.buf[1]=q1.b[0];
  can_dof2.buf[2]=q2.b[1];
  can_dof2.buf[3]=q2.b[0];
  can_dof2.buf[4]=q3.b[1];
  can_dof2.buf[5]=q3.b[0];
  can_dof2.buf[6]=q4.b[1];
  can_dof2.buf[7]=q4.b[0];  
  
  CANbus.write(can_pos);  
  CANbus.write(can_pos_fil); 
  CANbus.write(can_nav); 
  CANbus.write(can_nav_fil);
  CANbus.write(can_dof1);
  CANbus.write(can_lap);    
  CANbus.write(can_dof2);  
}



