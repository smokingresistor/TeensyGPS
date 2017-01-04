#include "MatrixMath.h"
//#include <SoftwareSerial.h>    //in case arduino board
#include "BMsg838.h"
#include "KalmanFilter.h"
#include "KalmanFilterVA.h"
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
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "NXPMotionSense.h"

Adafruit_BMP280 bme; // I2C

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

/*can bus and can messages initialisation*/
FlexCAN CANbus(1000000);
static CAN_message_t can_pos,can_nav,can_pos_fil,can_nav_fil,can_dof1,can_dof2,can_lap,can_gyr,can_dop;

/*gps interfacce relate class*/
BMsg838 gps;
Metro beacon_timeout = Metro(30000);
Metro beacon_trigger = Metro(250);
Metro file_time_cut= Metro(300000);
Metro led_blink= Metro(50);


/*kalman filter class*/
KalmanFilter filter;
KalmanFilterVA filterVA;

/*variables for conversion from float to 4 uint8_t*/
union conv lat, lon, lat_fil, lon_fil;
/*variables for conversion from int16_t to 2 uint8_t*/
union conv_short alt, alt_fil, dof_yaw, dof_roll, dof_pitch, acc_x, acc_y, acc_z, q1, q2, q3, q4, gyr_x, gyr_y, gyr_z, r_o;
/*variables for conversion from uint16_t to 2 uint8_t*/
union conv_short_u pressure, vel, vel_fil, course, lap_dist, stint_time, g_dop, p_dop, h_dop, v_dop, t_dop ;

/*led pin*/
int led = 13;
boolean led_on=1;
boolean file_cutted=false;

uint32_t t0,t1,dt,laptime=0;
uint8_t sats,fix;

boolean log_en;
int newlog, rate, can_speed, max_filesize, filesize, log_type, trig, intv;
float blat, blong, btime, btol, course_angle,course_angle2, trigv, min_val, max_val, stint_duration;
String UTC_Time;
String Delta_Time;

/*structures for config data*/
struct DOF_DATA att;
struct CAN_DATA CAN[NUM_CAN_FRAME];
struct FLS_DATA FLS[3];
struct PIT_DATA PIT[1];

/*variables for 9dof data*/
float ax, ay, az, gx, gy, gz, mx, my, mz, x, y, z; // variables to hold latest sensor data values 
float heading, roll, pitch, yaw, temperature, inclination, lap_distance; 
float yaw_rate, prev_yaw;
float bmp280_pressure;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
pt prev_gps;  //curr_gps removed, structs of points defined in loglib.h
pt prev_gps_2;
float Timer_50Hz = 20; //20ms
float prev_time;
/////////////////////////////////////
// Trajectory Radius Variables
float x_0,y_0,x_1,y_1,x_2,y_2,ro,delta_course=0;
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
int pin_beacon[3] = {26, 27, 28}; //26 27 28 for New board 33 32 31 for Bt board

//////////////////////////////////////
// buffers fo gps coordinates 
const int bearing_buffer=25;
const int bearing_buffer_2=50;
SimpleFIFO<float,bearing_buffer> Lat_buffer;
SimpleFIFO<float,bearing_buffer> Long_buffer;
//added a second FIFO buffer to output corner radius from a circonference passing from 3 points
SimpleFIFO<float,bearing_buffer_2> Lat_buffer_2;
SimpleFIFO<float,bearing_buffer_2> Long_buffer_2;

//////////////////////////////////////
/// \fn setup
/// \brief Initialisation hardware
///
void setup()
{
  //intizialize the fifo buffer with garbage
  for (int i=0; i < bearing_buffer; i++)
  {
    Lat_buffer.enqueue(0.0);
    Long_buffer.enqueue(0.0);
  }

    for (int i=0; i < bearing_buffer_2; i++)
  {
    Lat_buffer_2.enqueue(0.0);
    Long_buffer_2.enqueue(0.0);
  }
  
  // Serial start with 115200 baudrate
  Serial.begin(115200); 
  //delay(30000);
  // BMP280 start
  bme.begin();
  // LSM9DS0 start
  sensor_9dof_configure();
  // Delay 1 sec
  delay(1000);
  // Setup pin for led
  pinMode(led, OUTPUT);
  // Call function in Loglib.cpp
  LogSetup();
  // Set can speed (1000000, 500000, 250000, 100000, 50000 bps)
  can_speed = CNF[1];
  // Set new log (0:Disabled, 1:Enabled)
  newlog = CNF[2];
  // Set GPS update rate (1, 2, 4, 5, 8, 10, 20, 25, 40, 50 Hz)
  rate = CNF[3];
  // Set maximum file size (MBytes)
  max_filesize = CNF[4];
  // Set beacon latitude
  blat = CNF[5];
  // Set beacon longitude
  blong = CNF[6];
  // Set beacon timeout
  btime = CNF[7];
  // Set beacon tolerance
  btol = CNF[8];
  // Set type of data logging (0:Continuous, 1:Trigger, 2:Interval)
  log_type = CNF[9];
  // Set data trigger type (0:Longitude, 1:Latitude, 2:Altitude, 3:Speed, 4:UTC Time, 5:UTC Date)
  trig = CNF[10];
  // Set trigger value
  trigv = CNF[11];
  // Set the type of interval (0:min/max time, 1:min/max distance, 2:min/max speed)
  intv = CNF[12];
  // Set minimum value to enable logging (ex: 60 seconds after power on)
  min_val = CNF[13];
  // Set maximum value to disable logging (ex: 3600 seconds after power on)
  max_val = CNF[14];
  // Set file timeout time from beacon timeout
  file_time_cut.interval(btime*1000);
  // Print file size in bytes
  Serial.println(filesize*1048576);
  // Print newlog
  Serial.println(newlog);
  // init Can bus
  FlexCAN CANbus(can_speed);
  if (can_speed) 
    {       
       // setup CAN messages id and len
       // 1 Frame
       can_pos.id=CAN[0].id;
       can_pos.len = 8;
       can_pos.timeout = 1;
       
       // 2 Frame
       can_pos_fil.id=CAN[1].id;
       can_pos_fil.len = 8;
       can_pos_fil.timeout = 1;
       
       // 3 Frame
       can_nav.id=CAN[2].id;
       can_nav.len = 8;
       can_nav.timeout = 1;
       
       // 4 Frame
       can_nav_fil.id=CAN[3].id;
       can_nav_fil.len = 8;
       can_nav_fil.timeout = 1;
       
       // 5 Frame
       can_dof1.id = CAN[4].id;
       can_dof1.len = 8;
       can_dof1.timeout = 1;
       
       // 6 Frame
       can_lap.id=CAN[5].id;
       can_lap.len=8;
       can_lap.timeout = 1;
       
       // 7 Frame
       can_dof2.id=CAN[6].id;
       can_dof2.len=8;
       can_dof2.timeout = 1;
       
       // 8 Frame
       can_gyr.id=CAN[7].id;
       can_gyr.len=8;
       can_gyr.timeout = 1;
       
       // 9 Frame       
       can_dop.id=CAN[8].id;
       can_dop.len=8;
       can_dop.timeout = 1;
       
       // start CAN bus 
       CANbus.begin(); 
       // Set led power on
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
  // Start serial for gps communication
  Serial2.begin(115200);
  char messagetype[64];
  memset(messagetype,0,64);
  Serial.print("Testing BMsg838 binary message library v. "); 
  Serial.println();
  Serial.print(" BMsg838 System reset.\r\n"); 
  // setup GPS with messages
  // SendBinaryMessagetoGPSreceiver(gps.ResetGNSS(1, 15, 6, 9,11, 30, 25, 20, 133, 1200), gps.SendStream,gps.RecVBinarybuf,0,2000);
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

// main loop
void loop()
{
  int ret=0;
  float temp;
  char messagetype[64];
  memset(messagetype,0,64);
  // Open file for logging
  dataFile = SD.open(namefile, FILE_WRITE);
  while (1)
  {
      float now_time = millis();
      // read 9 dof data and BMP280 with 50Hz cycle
      if (now_time - prev_time >= Timer_50Hz){
          //Serial.println("Start loop");
          //Serial.println(now_time);
          prev_time = now_time;
          sensor_9dof_read();
          bmp280_pressure = bme.readPressure()/100.0; //pressure in mBar
          //Serial.print("UTC time: ");
          // Get UTC Time
          UTC_Time = GetUTCTime(gps.venus838data_raw.gps_week, gps.venus838data_raw.timeofweek);
          //Serial.println(UTC_Time);
          // Get Delta Time
          float time = millis();
          Delta_Time = GetDeltaTime(time);
          //Serial.println(Delta_Time);
          // Put last gps coordinates to buffer
          Lat_buffer.enqueue(gps.venus838data_raw.Latitude);
          Long_buffer.enqueue(gps.venus838data_raw.Longitude); 
          Lat_buffer_2.enqueue(gps.venus838data_raw.Latitude);
          Long_buffer_2.enqueue(gps.venus838data_raw.Longitude);       
          // Take first gps coordinates from buffer
          prev_gps.lat=Lat_buffer.dequeue();
          prev_gps.lon=Long_buffer.dequeue();     
          prev_gps_2.lat=Lat_buffer_2.dequeue();
          prev_gps_2.lon=Long_buffer_2.dequeue(); 
          // Calc course angle    
          course_angle = gps.course_to(gps.venus838data_raw.Latitude, gps.venus838data_raw.Longitude, prev_gps.lat, prev_gps.lon);   
          course_angle2 = gps.course_to(gps.venus838data_raw.Latitude, gps.venus838data_raw.Longitude, prev_gps_2.lat, prev_gps_2.lon); 
           
           //calculate trayctory radius
          x_0=0;
          y_0=0;
          x_2=6372795*sin(radians(prev_gps_2.lon-gps.venus838data_raw.Longitude));
          y_2=6372795*sin(radians(prev_gps_2.lat-gps.venus838data_raw.Latitude));
          x_1=6372795*sin(radians(prev_gps.lon-gps.venus838data_raw.Longitude));
          y_1=6372795*sin(radians(prev_gps.lat-gps.venus838data_raw.Latitude));  

          ro=calculate_trajecotry_curvature(x_0,y_0,x_1,y_1,x_2,y_2);
  
          delta_course=course_angle2-course_angle;

          if ((delta_course<=0)||(delta_course>=300)) ro=-ro;
           
          //Calculate lap distance
          t1=millis();    
          dt=t1-t0;
          t0=t1;                
          laptime = laptime + dt;  //laptime in milliseconds
          
          temp=dt;
          temp=temp/1000;  //dt in seconds
          
          lap_distance=lap_distance+temp*gps.venus838data_raw.velocity;  //Lap distance in meters   
          temp=temp/60;
          stint_duration = stint_duration + temp;  //stint duration in minutes
      
          if (beacon_output)
              check_beacon_dist(); 
          
          if (can_speed)
          {
              if (!log_output) LogATT_nosd();
              can_send();
          }    
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
                  //Serial.print("Time start log: ");
                  //Serial.println(millis());
                  LogTPV(); // log TPV object
                  LogATT(); // log ATT object 
                  //Serial.print("Time stop log: ");
                  //Serial.println(millis());
              }//if      
          }
      }
      if(Serial2.available()){
           // read number of returned bytes
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
      
    
       }//if ret
    }//if serial     
  }//while
}//loop  


void check_beacon_dist(){
   boolean crossing_true;
   // pt curr_gps, prev_gps; //moved to global
   //curr_gps.lat = gps.venus838data_filter.Latitude;
   //curr_gps.lon = gps.venus838data_filter.Longitude;
   for (unsigned int i = 0; i < (sizeof(beacons))/(sizeof(beacons[0])); i++){
     if (FLS[i].en)
      {
       crossing_true = get_line_intersection (beacons[i].p0_lat, beacons[i].p0_lon, beacons[i].p1_lat, beacons[i].p1_lon, prev_gps.lat, prev_gps.lon, gps.venus838data_raw.Latitude, gps.venus838data_raw.Longitude);
       if (crossing_true && (beacon_timeout.check()==true)&&(gps.venus838data_raw.velocity>0))
        {
          if ((gps.venus838data_raw.velocity>beacons[i].min_spd )&&(gps.venus838data_raw.velocity<beacons[i].max_spd))
          {
         if (i==0)
           Serial.println("Finish line crossing");
         if (i==1)
         {
               //Serial.println("Pit entry line crossing");
               stint_time.f = 0;
               stint_duration = 0;
           }
           //Serial.println("Pit entry line crossing");
         if (i==2)
            {
               //Serial.println("Pit exit line crossing");
               stint_time.f = 0;
               stint_duration = 0;
           }
           
         if (file_time_cut.check()==true)
               Serial.println("Time Cut Trigger"); 
               
         //Triggers digital outputs according to setup
         digitalWrite(pin_beacon[i],beacons[i].output_level); 
         lap_dist.f = 0;        
         laptime=0;
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
   //prev_gps.lat = curr_gps.lat;
   //prev_gps.lon = curr_gps.lon;
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

// fn calculate_trajecotry_curvature

float calculate_trajecotry_curvature(float _x1, float _y1, float _x2, float _y2, float _x3, float _y3){
   float a,b,c,s,area,radius,_ro;
   
        a = sqrt(pow((_x1-_x2),2)+pow((_y1-_y2),2));
        b = sqrt(pow((_x2-_x3),2)+pow((_y2-_y3),2));
        c = sqrt(pow((_x3-_x1),2)+pow((_y3-_y1),2));
        s = (a+b+c)/2;
        area = sqrt(s*(s-a)*(s-b)*(s-c)); 
        //Serial.print("Area ");
        //Serial.println(area,20);
        if (area==0) {
         _ro=0;         
        }
        else {
         radius = (a*b*c)/(4*area);
        //Serial.print("Radius ");
        //Serial.println(radius,20);
          _ro=1/radius;
        }
        return _ro;
}

//fn can_send

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
  
  g_dop.f=(uint16_t)(gps.venus838data_raw.gdop*100);
  p_dop.f=(uint16_t)(gps.venus838data_raw.pdop*100);
  h_dop.f=(uint16_t)(gps.venus838data_raw.hdop*100);
  v_dop.f=(uint16_t)(gps.venus838data_raw.vdop*100);
  t_dop.f=(uint16_t)(gps.venus838data_raw.tdop*100);
  

  course_angle = course_angle*100;
  course.f=(uint16_t)(course_angle);
  sats=gps.venus838data_raw.NumSV;
  fix=gps.venus838data_raw.fixmode;
  fix=fix<<4;
  lap_dist.f = (uint16_t)(lap_distance*2.5);     
  dof_roll.f = (int16_t)(att.roll*100);
  dof_pitch.f = (int16_t)(att.pitch*100);
  dof_yaw.f = (int16_t)(att.yaw*100);
  pressure.f = (uint16_t)(bmp280_pressure*10);

  r_o.f=(int16_t)((ro)*100000);
  
  acc_x.f = (int16_t)(att.acc_x*100);
  acc_y.f = (int16_t)(att.acc_y*100);
  acc_z.f = (int16_t)(att.acc_z*100);
  
  gyr_x.f = (int16_t)(att.gyro_x*100);
  gyr_y.f = (int16_t)(att.gyro_y*100);
  gyr_z.f = (int16_t)(att.gyro_z*100);
  stint_time.f=stint_duration*100; //Stint duration in 1/100 of minutes, max 655 minutes of stint duration.
  
  /*
  q1.f = q[0]*100;//att.quat1;
  q2.f = q[1]*100;//att.quat2;
  q3.f = q[2]*100;//att.quat3;
  q4.f = q[3]*100;//att.quat4;
 */
  
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
  
  // can_nav_fil.buf[0]=alt_fil.b[1];
  //can_nav_fil.buf[1]=alt_fil.b[0];

  can_nav_fil.buf[0]=r_o.b[1];
  can_nav_fil.buf[1]=r_o.b[0];
  can_nav_fil.buf[2]=vel_fil.b[1];
  can_nav_fil.buf[3]=vel_fil.b[0];
  can_nav_fil.buf[4]=stint_time.b[1]; //changed to stint time
  can_nav_fil.buf[5]=stint_time.b[0];
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
  
  q1.f=q[0]*10000;
  q2.f=q[1]*10000;
  q3.f=q[2]*10000;
  q4.f=q[3]*10000;
  
  can_dof2.buf[0]=q1.b[1];
  can_dof2.buf[1]=q1.b[0];
  can_dof2.buf[2]=q2.b[1];
  can_dof2.buf[3]=q2.b[0];
  can_dof2.buf[4]=q3.b[1];
  can_dof2.buf[5]=q3.b[0];
  can_dof2.buf[6]=q4.b[1];
  can_dof2.buf[7]=q4.b[0];  
  
  //8 frame
  can_gyr.buf[0]=gyr_x.b[1];
  can_gyr.buf[1]=gyr_x.b[0];
  can_gyr.buf[2]=gyr_y.b[1];
  can_gyr.buf[3]=gyr_y.b[0];
  can_gyr.buf[4]=gyr_z.b[1];
  can_gyr.buf[5]=gyr_z.b[0];
  can_gyr.buf[6]=g_dop.b[1];
  can_gyr.buf[7]=g_dop.b[0];

  //9 frame
  can_dop.buf[0]=p_dop.b[1];
  can_dop.buf[1]=p_dop.b[0];
  can_dop.buf[2]=h_dop.b[1];
  can_dop.buf[3]=h_dop.b[0];
  can_dop.buf[4]=v_dop.b[1];
  can_dop.buf[5]=v_dop.b[0];
  can_dop.buf[6]=t_dop.b[1];
  can_dop.buf[7]=t_dop.b[0];
  
  
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
  if (CAN[7].en)   
      CANbus.write(can_gyr); 
  if (CAN[8].en)   
      CANbus.write(can_dop);     

}

