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
#define SIMPLEFIFO_LARGE
#include <SimpleFIFO.h>

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
  
   union conv_short_u
  {
    uint16_t f;
    uint8_t b[2];
  };


FlexCAN CANbus(1000000);
static CAN_message_t can_pos,can_nav,can_pos_fil,can_nav_fil, can_dof1, can_dof2, can_lap;


/*gps interfacce relate class*/
BMsg838 gps;
Metro beacon_timeout = Metro(30000);
Metro beacon_trigger = Metro(250);
Metro file_time_cut= Metro(300000);
Metro led_blink= Metro(50);

/*kalman filter class*/
KalmanFilter filter;
KalmanFilterVA filterVA;

union conv lat, lon, lat_fil, lon_fil;
union conv_short alt, alt_fil, dof_roll, dof_pitch, acc_x, acc_y, acc_z, q1, q2, q3, q4;
union conv_short_u dof_yaw, pressure, vel, vel_fil, course, lap_dist, lap_time;

int led = 13;

boolean led_on=1;
boolean file_cutted=false;

uint32_t t0,t1,dt,laptime=0;
uint8_t sats,fix;

boolean log_en;
int newlog, rate, can_speed, max_filesize, filesize, log_type, trig, intv;
float blat, blong, btime, btol, course_angle, trigv, min_val, max_val;
String UTC_Time;

struct DOF_DATA att;
struct CAN_DATA CAN[7];
struct FLS_DATA FLS[3];
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float heading, roll, pitch, yaw, temp, inclination, lap_distance, temp_x, temp_y, temp_z; 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
pt prev_gps;  //curr_gps removed, structs of points defined in loglib.h
/////////////////////////////////////
// trigger line virables
boolean beacon_output=1;
typedef struct
{
     double p0_lat;
     double p0_lon;
     double p1_lat;
     double p1_lon;
     double min_spd;  // minimum speed for trigger in m/s
     double max_spd;  // maximum speed for trigger in m/s
     bool output_level; //output level of the digital line
}  line;

line beacons[3];
int pin_beacon[3] = {26, 27, 28}; //26 27 28 for Bt board 33 32 31 for new board
//////////////////////////////////////

const int bearing_buffer=10;
SimpleFIFO<float,bearing_buffer> Lat_buffer;
SimpleFIFO<float,bearing_buffer> Long_buffer;

//////////////////////////////////////

void setup()
{
  //intizialize the fifo buffer with garbage
  for (int i=0; i < bearing_buffer; i++)
  {
    Lat_buffer.enqueue(0.0);
    Long_buffer.enqueue(0.0);
  }
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
       Serial.println(CAN[0].id,HEX);
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
      for (int i=0; i<3; i++){
        pinMode(pin_beacon[i], OUTPUT);
        if (FLS[i].en)
          beacons[i].p0_lat = FLS[i].lat_A;
          beacons[i].p0_lon = FLS[i].lon_A;
          beacons[i].p1_lat = FLS[i].lat_B;
          beacons[i].p1_lon = FLS[i].lon_B;
          beacons[i].min_spd = FLS[i].minSpeed;
          beacons[i].max_spd = FLS[i].maxSpeed;
          if (FLS[i].lineOut==1)
            {
              beacons[i].output_level=0;               
            }
         if ((FLS[i].lineOut==2)||(FLS[i].lineOut==0))
            {
              beacons[i].output_level=1;
            }
         digitalWrite(pin_beacon[i],!beacons[i].output_level);
        // digitalWrite(pin_beacon[i],HIGH);
      }
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

}  //setup 

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
    Lat_buffer.enqueue(gps.venus838data_raw.Latitude);
    Long_buffer.enqueue(gps.venus838data_raw.Longitude);     
    prev_gps.lat=Lat_buffer.dequeue();
    prev_gps.lon=Long_buffer.dequeue();              
    course.f = gps.course_to(gps.venus838data_raw.Latitude, gps.venus838data_raw.Longitude, prev_gps.lat, prev_gps.lon);   
    //Calculate lap distance
    
    t1=millis();    
    dt=t1-t0;
    t0=t1;                
    laptime = laptime + dt;  //laptime in milliseconds
    
    temp=dt;
    temp=temp/1000.0;
    lap_distance=lap_distance+temp*gps.venus838data_raw.velocity;  //Lap distance in meters    
    sensor_9dof_read();   
    att.heading = heading;
    att.pitch = pitch;
    att.yaw = yaw;
    att.roll = roll;
    att.dip = inclination;
    att.mag_x = mx;
    att.mag_y = my;
    att.mag_z = mz;
    temp_x = mx;
    temp_y = my;
    temp_z = mz;
//    att.mag_len = (pow(temp_x, 2) + pow(temp_y, 2) + pow(temp_z, 2));
    att.acc_x = ax;
    att.acc_y = ay;
    att.acc_z = az;
    temp_x = ax;
    temp_y = ay;
    temp_z = az;
//    att.acc_len = (pow(temp_x, 2) + pow(temp_y, 2) + pow(temp_z, 2));
    att.gyro_x = gx;
    att.gyro_y = gy;
    att.gyro_z = gz;
    att.quat1 = q[0];
    att.quat2 = q[1];
    att.quat3 = q[2];
    att.quat4 = q[3];
    att.temp = temp;
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
            if (led_blink.check())
              {
               led_on=!led_on;
               digitalWrite(led,led_on);
               led_blink.reset();
              }
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
   boolean crossing_true;
   // pt curr_gps, prev_gps; //moved to global
   for (int i = 0; i < (sizeof(beacons))/(sizeof(beacons[0])); i++){
     if (FLS[i].en)
      {
       crossing_true = get_line_intersection (beacons[i].p0_lat, beacons[i].p0_lon, beacons[i].p1_lat, beacons[i].p1_lon, prev_gps.lat, prev_gps.lon, gps.venus838data_raw.Latitude, gps.venus838data_raw.Longitude);
       if (crossing_true && (beacon_timeout.check()==true)&&(gps.venus838data_raw.velocity>0))
        {
          if ((gps.venus838data_raw.velocity>beacons[i].min_spd )&&(gps.venus838data_raw.velocity<beacons[i].min_spd))
          {
         if (i==0)
           Serial.println("Finish line crossing");
         if (i==1)
           Serial.println("Pit entry line crossing");
         if (i==2)
           Serial.println("Pit exit line crossing");
         if (file_time_cut.check()==true)
               Serial.println("Time Cut Trigger"); 
         //Triggers digital outputs according to setup
         digitalWrite(pin_beacon[i],beacons[i].output_level); 
         lap_dist.f = 0;
         lap_time.f = 0;
         can_lap.buf[0] = i+1;               
         if (sd_datalog && log_output) {
             // if (file_time_cut.check()==true) dataFile.println("Time Trigger");
             if (newlog==1) {
                 Serial.println("Create new log");
                 create_newlog();
                 if (!dataFile) 
                    dataFile = SD.open(namefile, FILE_WRITE);
                 dataFile.print("Beacon Trigger ");
                 dataFile.print(i+1);
                 dataFile.flush();
                 dataFile.close(); 
             } //end if newlog
         } //end if sd datalog and log output
         beacon_timeout.reset();
         beacon_trigger.reset();
         file_time_cut.reset();         
       }  //end if speed range
     } //end if crossing true
    }  //end if FLS enable
   }//for
   if ((beacon_trigger.check()==true)&&(can_lap.buf[0]>0))
       {
       //Reset the digital lines according to the config
       digitalWrite(pin_beacon[0],!beacons[0].output_level);
       digitalWrite(pin_beacon[1],!beacons[1].output_level);
       digitalWrite(pin_beacon[2],!beacons[2].output_level);
       can_lap.buf[0]=0;
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

boolean get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,float p2_x, float p2_y, float p3_x, float p3_y)
{
    float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    float i_x,i_y=0;
    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;
 
    denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return 0; // Collinear
    bool denomPositive = denom > 0;
 
    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return 0; // No collision
 
    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return 0; // No collision
 
    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return 0; // No collision
    // Collision detected
   
        t = t_numer / denom;
 
        i_x = p0_x + (t * s10_x);
        Serial.println(i_x, 7);
 
        i_y = p0_y + (t * s10_y);
        Serial.println(i_y, 7);
 
    return true;
}

void can_send(){
              if (led_blink.check())
              {
                 led_on=!led_on;
                 digitalWrite(led,led_on);
              }
  lat.f=(int32_t)(gps.venus838data_raw.Latitude*1E7); 
  lat_fil.f=(int32_t)(gps.venus838data_filter.Latitude*1E7);
  lon.f=(int32_t)(gps.venus838data_raw.Longitude*1E7);
  lon_fil.f=(int32_t)(gps.venus838data_filter.Longitude*1E7);
  alt.f=(int16_t)(gps.venus838data_raw.SealevelAltitude*10);
  alt_fil.f=(int16_t)(gps.venus838data_filter.SealevelAltitude*10);
  vel.f=(uint16_t)(gps.venus838data_raw.velocity*100*3.6);
  vel_fil.f=(uint16_t)(gps.venus838data_filter.velocity*100*3.6);
  course.f=(uint16_t)(course_angle*100);
  sats=gps.venus838data_raw.NumSV;
  fix=gps.venus838data_raw.fixmode;
  fix=fix<<4;
  lap_dist.f = (uint16_t)(lap_distance*2.5);     
  dof_roll.f = (int16_t)(att.roll*100);
  dof_pitch.f = (int16_t)(att.pitch*100);
  dof_yaw.f = (uint16_t)(att.yaw*100);
  pressure.f = (uint16_t)0;               //to do
  acc_x.f = (int16_t)(att.acc_x*100);
  acc_y.f = (int16_t)(att.acc_y*100);
  acc_z.f = (int16_t)(att.acc_z*100);
  q1.f = (int16_t)(att.quat1*100);
  q2.f = (int16_t)(att.quat2*100);
  q3.f = (int16_t)(att.quat3*100);
  q4.f = (int16_t)(att.quat4*100);
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



