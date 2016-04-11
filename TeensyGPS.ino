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
#include <PString.h>

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
  



FlexCAN CANbus(1000000);
static CAN_message_t can_pos,can_nav,can_pos_fil,can_nav_fil, can_dof1, can_dof2, can_lap;


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
boolean beacon_output=1;
boolean file_cutted=false;

double t0,t1,dt=0;
uint8_t sats,fix;
const float lat_beacon[4]={45.618967, 2.760700  , 14.958108,    45.533341};
const float lon_beacon[4]={9.281226,  101.738322, 103.085608,   10.219222};
//                        0 Monza     1 Sepang   2 Buri ram     3 Via XX Settembre

const float beacon_distance=9; //radious in m for lap beacon trigger

boolean log_en;
int newlog, rate, can_speed, max_filesize, filesize, log_type, trig, intv;
float blat, blong, btime, btol, course_angle, trigv, min_val, max_val;
String UTC_Time;

struct DOF_DATA att;
struct CAN_DATA CAN[7];
struct FLS_DATA FLS[3];
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float heading, roll, pitch, yaw, temp, inclination; 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

void setup()
{
  Serial.begin(115200); 
  sensor_9dof_configure();
  delay(1000);
  pinMode(led, OUTPUT);
  LogSetup();
  can_speed = CNF[1];
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
  FlexCAN CANbus(can_speed);
  if (can_speed) 
    {       
       can_pos.id=CAN[0].id;
       can_pos.len = 8;
       can_nav.id=CAN[1].id;
       can_nav.len = 8;
       can_pos_fil.id=CAN[2].id;
       can_pos_fil.len = 8;
       can_nav_fil.id=CAN[3].id;
       can_nav_fil.len = 8;
       can_dof1.id = CAN[4].id;
       can_dof1.len = 8;
       can_lap.id=CAN[5].id;
       can_lap.len=8;
       can_dof2.id=CAN[6].id;
       can_dof2.len=8;
       CANbus.begin(); 
       digitalWrite(led,led_on); 
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
    //course_angle = gps.course_to(gps.venus838data_filter.Longitude, gps.venus838data_filter.Latitude, 0 , 0);          
    sensor_9dof_read();   
    if (beacon_output)
        check_beacon_dist();
    if (can_speed)
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
            Serial.println(millis());
            LogTPV(); // log TPV object
            LogATT(); // log ATT object  
            Serial.println(millis());
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
   boolean line_cross[3];
   pt pointA, pointB, curr_gps, prev_gps;
   //distance = gps.distance_between(gps.venus838data_filter.Latitude,gps.venus838data_filter.Longitude,blat,blong); // distance to beacon
   curr_gps.x = gps.venus838data_filter.Latitude;
   curr_gps.y = gps.venus838data_filter.Longitude;
//   curr_gps.x = 54.7386700;
//   curr_gps.y = 55.9748100;
//   prev_gps.x = 54.7386100;
//   prev_gps.y = 55.9747700;
   for (int i = 0; i < 3; i++){
     if (FLS[i].en)
//       pointA.x = 54.7386700;
//       pointA.y = 55.9747200;
//       pointB.x = 54.7386400;
//       pointB.y = 55.9748600;
       pointA.x = FLS[i].lat_A;
       pointA.y = FLS[i].lon_A;
       pointB.x = FLS[i].lat_B;
       pointB.y = FLS[i].lon_B;
//       Serial.print(pointA.x, 7);
//       Serial.print(" ");
//       Serial.println(pointA.y, 7);
//       Serial.print(pointB.x, 7);
//       Serial.print(" ");
//       Serial.println(pointB.y, 7);
       line_cross[i] = get_line_intersection (pointA, pointB, prev_gps, curr_gps);
   };
   prev_gps.x = curr_gps.x;
   prev_gps.y = curr_gps.y;
   if (line_cross[0])
     Serial.println("Finish line crossing");
   if (line_cross[1])
     Serial.println("Pit entry line crossing");
   if (line_cross[2])
     Serial.println("Pit exit line crossing");
   if (((line_cross[0] && (beacon_timeout.check()==true)))||(file_time_cut.check()==true)){ //check distance or time_cut
     Serial.println("Finish line crossing");
     if (file_time_cut.check()==true)
           Serial.println("Time Cut Trigger"); 
     //Rise level of beacon pin output or lowers for McLaren
     digitalWrite(pin_beacon,LOW); 
     if (can_speed){
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


float det(float a, float b, float c, float d){
  return a*d - b*c;
}

boolean between(double a, double b, double c){
  return min(a,b) <= c + EPS && c <= max(a,b) + EPS;
}

void swap(double &a, double &b){
  double temp;
  temp = a;
  a = b;
  b = temp;
}

boolean intersect_line(double a, double b, double c, double d){
   if (a > b) swap(a, b);
   if (c > d) swap(c, d);
   return max(a,c) <= min(b,d);
}

boolean get_line_intersection(pt p0, pt p1, pt p2, pt p3){
  pt s10, s32;
  double c1, c2, denom;
  s10.x = p1.x-p0.x;  s10.y = p1.y-p0.y;   
  s32.x = p3.x-p2.x;  s32.y = p3.y-p2.y;  
  denom = det (s10.y, s10.x, s32.y, s32.x);
//  Serial.println(s10.y, 10);
//  Serial.println(s10.x, 10);
//  Serial.println(s32.y, 10);
//  Serial.println(s32.x, 10);
//  Serial.println(denom, 15);
  if (denom != 0) { // lines are not collinear
    c1 = s10.y*p0.x + s10.x*p0.y;
    c2 = s32.y*p2.x + s32.x*p2.y;
//    Serial.println(c1, 7);
//    Serial.println(c2, 7);
    //find crossing point (x,y)
    double x = det (c1, s10.x, c2, s32.x) * 1.0 / denom;
    double y = det (s10.y, c1, s32.y, c2) * 1.0 / denom;
//    Serial.println(x, 7);
//    Serial.println(y, 7);
    return between (p0.x, p1.x, x) && between (p0.y, p1.y, y)
           && between (p2.x, p3.x, x) && between (p2.y, p3.y, y);
	}
  else //lines are collinear
    return det (s10.y, c1, s32.y, c2) == 0 && det (s10.x, c1, s32.x, c2) == 0
		&& intersect_line (p0.x, p1.x, p2.x, p3.x)
		&& intersect_line (p0.y, p1.y, p2.y, p3.y);
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
  
  if (CAN[0].en)
      CANbus.write(can_pos);  
  if (CAN[1].en)    
      CANbus.write(can_pos_fil); 
  if (CAN[2].en)
      CANbus.write(can_nav); 
  if (CAN[3].en)
      CANbus.write(can_nav_fil);
  if (CAN[4].en)
      CANbus.write(can_dof1);
  if (CAN[5].en)
      CANbus.write(can_lap); 
  if (CAN[6].en)   
      CANbus.write(can_dof2);  
}


