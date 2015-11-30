#include "MatrixMath.h"
//#include <SoftwareSerial.h>    //in case arduino board
#include "BMsg838.h"
#include "KalmanFilter.h"
#include "GPSSerialMessageCom.h"
//#include <Adafruit_LSM9DS0.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
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
 
static CAN_message_t can_pos,can_nav;
FlexCAN CANbus(1000000);
/*gps interfacce relate class*/
BMsg838 gps;
Metro beacon_timeout = Metro(30000);
Metro beacon_trigger = Metro(250);
/*kalman filter class*/
KalmanFilter filter;

union conv lat, lon;
union conv_short alt, vel;
int fileNum = 10000; 
char namefile[13]="GPS10000.TXT";
int chipSelect = 6; //TeensyGPS version 1.0
//int chipSelect = 15; //TeensyGPS version 1.1
int mosi = 7;
int miso = 8;
int sck = 14;
int led = 13;
int pin_beacon=21;

// File dataFile;
Sd2Card card;
// SdVolume volume;
// SdFile root;
boolean led_on=1;
boolean sd_datalog=1;
boolean file_open=false;
boolean can_output=1;
boolean file_log=true;
boolean beacon_output=1;
boolean file_cutted=false;

double t0,t1,dt=0;
int count,checksums=0;
uint8_t sats,fix;
const float lat_beacon[4]={45.618967, 2.760700  , 14.958108,    45.533341};
const float lon_beacon[4]={9.281226,  101.738322, 103.085608,   10.219222};
//                        0 Monza     1 Sepang   2 Buri ram     3 Via XX Settembre
File dataFile;

const float beacon_distance=15; //radius in m for lap beacon trigger


void setup()
{
 
 Serial.begin(115200); 
 delay(10000); 
 SPI.setMOSI(mosi);
 SPI.setMISO(miso);
 SPI.setSCK(sck);
 pinMode(chipSelect, OUTPUT);
 pinMode(led, OUTPUT);
 if (can_output) 
    {       
       can_pos.id=0x302;
       can_pos.len = 8;
       can_nav.id=0x304;
       can_nav.len = 8;
       CANbus.begin(); 
       digitalWrite(led,led_on); 
       Serial.print("Canbus output enabled on fames:");
       Serial.print(can_pos.id,HEX);
       Serial.print(",");
       Serial.println(can_nav.id,HEX);
       
    }
    
 if (beacon_output==true)
    { 
      pinMode(pin_beacon, OUTPUT);
      digitalWrite(pin_beacon,LOW);
      Serial.println("Lap beacon enabled");
    }
 if ((!SD.begin(chipSelect))&&(file_log==true))
   {  
      Serial.println("File datalog ebabled");    
      Serial.println("Card Not Present");
      sd_datalog=false;
   }
 else
   {
      Serial.println("File datalog ebabled");    
      Serial.println("Card Present");
      sd_datalog=true;
      card.init(SPI_FULL_SPEED, chipSelect);
   }
  
  if (sd_datalog==true);
  {
  
  // Serial.println(namefile);
    while (SD.exists(namefile)) 
       {      
          Serial.print(namefile);
          Serial.println(" file present."); 
          incFileNum();  
       }
          Serial.print("Logging Data to "); 
          Serial.println(namefile);
          dataFile = SD.open(namefile, FILE_WRITE);           
             delay(100);
            if (dataFile)
             {
              dataFile.println("-------------------------------------------------------------------------------------------------------------------------------------");
              dataFile.println("Received_time;Latitude;Longitude;Altitude;Velocity;Latitude_fil;Longitude_fil;Altitude_fil;Velocity_fil;fix_mode;sat_num;checksums");
              dataFile.println("-------------------------------------------------------------------------------------------------------------------------------------");
              dataFile.close();
              Serial.println("Header wrote ok");
             }
          else 
          {
          Serial.println("Impossible to open datafile to write header");
          sd_datalog=0;
         } 
      } 
      



  Serial2.begin(115200);
  char messagetype[64];
  memset(messagetype,0,64);
  Serial.print("Testing BMsg838 binary message library v. "); 
  Serial.println();
  /*
  Serial.print(" BMsg838 System reset.\r\n"); 
  SendBinaryMessagetoGPSreceiver(gps.ResetGNSS(1, 15, 6, 9,11, 30, 25, 20, 133, 1200), gps.SendStream,gps.RecVBinarybuf,0,2000);
  SendBinaryMessagetoGPSreceiver(gps.SetSerialPort(115200, 1), gps.SendStream,gps.RecVBinarybuf,0,2000);
  SendBinaryMessagetoGPSreceiver(gps.SetBinaryMessagetype(), gps.SendStream,gps.RecVBinarybuf,0,2000); 
  SendBinaryMessagetoGPSreceiver(gps.SetPositionRate(50), gps.SendStream,gps.RecVBinarybuf,0,2000);
  if(SendBinaryMessagetoGPSreceiver(gps.GetSoftVersion(), gps.SendStream,gps.RecVBinarybuf,0,2000)==3);
      if(waitingRespondandReceive(gps.RecVBinarybuf,0x80,2000)>7){
              BinaryRecvMsgtype(messagetype,gps.RecVBinarybuf);
              Serial.println(messagetype); 
              GPSSoftVersiondata* versioninfo=gps.ResponseSoftVersion();
              printSoftversion(versioninfo);        
      }
  
  Serial.println("-------------------------------------------------------------------------------------------------------------------------------------");
  Serial.println("Received_time;Latitude;Longitude;Altitude;Velocity;Latitude_fil;Longitude_fil;Altitude_fil;Velocity_fil;fix_mode;sat_num;checksums");
  Serial.println("-------------------------------------------------------------------------------------------------------------------------------------");
  */
   
      

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
           // t1=millis();
           ret=waitingRespondandReceive(gps.RecVBinarybuf,0xA8,2000); 
           if(ret>7){                
                 if(!GPSNavigationMsgProcessing(&(gps.venus838data_raw),&(gps.venus838data_filter),gps,&filter))
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
                if (can_output)
                          {
                            lat.f=gps.venus838data_raw.Latitude*1E7;                       
                            lon.f=gps.venus838data_raw.Longitude*1E7;
                            alt.f=gps.venus838data_filter.SealevelAltitude*10;
                            vel.f=gps.venus838data_filter.velocity*10*3.6;
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
                            sats=gps.venus838data_raw.NumSV;
                            fix=gps.venus838data_raw.fixmode;
                            fix=fix<<4;
                            can_nav.buf[7]=fix|sats;           
                            CANbus.write(can_pos);  
                            CANbus.write(can_nav); 
                            led_on=!led_on;
                            digitalWrite(led,led_on);                       
                          }
                   
                        if (beacon_output==true)
                           {
                             float distanza=0;
                             for (int i=0; i<=(sizeof(lat_beacon))/(sizeof(lat_beacon[0]))-1;i++)
                             {
                               distanza=haverSine(gps.venus838data_raw.Latitude,gps.venus838data_raw.Longitude,lat_beacon[i],lon_beacon[i]);
                               
                               if ((distanza<beacon_distance)&&(beacon_timeout.check()==true))
                               
                                  {
                                    //Rise level of beacon pin outpu
                                    digitalWrite(pin_beacon,HIGH);                                    
                                    if (sd_datalog==1)
                                        {
                                          dataFile.println("Beacon Trigger"); 
                                          dataFile.flush();                                          
                                          dataFile.close(); 
                                                                                  
                                              while (SD.exists(namefile)) 
                                                   {      
                                                      Serial.print(namefile);
                                                      Serial.println("file present."); 
                                                      incFileNum();  
                                                   }                                          
                                             dataFile = SD.open(namefile, FILE_WRITE);
                                                       if (dataFile)
                                                              {
                                                                  dataFile.println("-------------------------------------------------------------------------------------------------------------------------------------");
                                                                  dataFile.println("Received_time;Latitude;Longitude;Altitude;Velocity;Latitude_fil;Longitude_fil;Altitude_fil;Velocity_fil;fix_mode;sat_num;checksums");
                                                                  dataFile.println("-------------------------------------------------------------------------------------------------------------------------------------");    
                                                                  Serial.print("Lap cut, new datafile is:"); 
                                                                  Serial.println(namefile); 
                                                                  dataFile.close();                                                 
                                                              } 
                                                        else 
                                                              {
                                                                  sd_datalog=0;
                                                                  Serial.println("Datalog disabled:"); 
                                                              }
                                        }
                                        // file_cutted=true;
                                        beacon_timeout.reset();
                                        beacon_trigger.reset();
                                        
                                  }
                                  else
                                  {
                                    if (beacon_trigger.check()==true) digitalWrite(pin_beacon,LOW);
                                    
                                    // file_cutted=false;
                                  }                                  
                              } 
                              
                           }
                        
                      if ((sd_datalog==1)&&(gps.venus838data_raw.fixmode>0))
                     
                          {
                                if (!dataFile) dataFile = SD.open(namefile, FILE_WRITE);
                                dataFile.print(millis());
                                dataFile.print(";");       
                                dataFile.print(gps.venus838data_raw.Latitude,12);
                                dataFile.print(";");
                                dataFile.print(gps.venus838data_raw.Longitude,12);
                                dataFile.print(";");
                                dataFile.print(gps.venus838data_raw.SealevelAltitude,12);
                                dataFile.print(";");
                                dataFile.print(gps.venus838data_raw.velocity,12);
                                dataFile.print(";");                                
                                dataFile.print(gps.venus838data_filter.Latitude,12);
                                dataFile.print(";");
                                dataFile.print(gps.venus838data_filter.Longitude,12);
                                dataFile.print(";");
                                dataFile.print(gps.venus838data_filter.SealevelAltitude,12);
                                dataFile.print(";");
                                dataFile.print(gps.venus838data_filter.velocity,12);
                                dataFile.print(";");                                
                                dataFile.print(fixmodmask[gps.venus838data_raw.fixmode]);
                                dataFile.print(";");
                                dataFile.print(gps.venus838data_raw.NumSV);
                                dataFile.print(";");
                                dataFile.print(checksums);
                                dataFile.println(";");                                
                                // int temp=count%5;
                                if (count%500==0) 
                                  {
                                    dataFile.flush();
                                    Serial.println("Datafile Saved");
                                  }
                                count++;
                                //dataFile.close();
                          }

                
                /*
                Serial.println("filtered data:\n");                
                printpositionfloatformat(gps.venus838data_filter.Latitude, 10000000, "  Latitude= ", "degree");
                printpositionfloatformat(gps.venus838data_filter.Longitude, 10000000, "  Longitude= ", "degree");
                printpositionfloatformat(gps.venus838data_filter.SealevelAltitude, 100, "  SealevelAltitude= ", "meter");
                printpositionfloatformat(gps.venus838data_filter.velocity, 100, "  velocity= ", "meter/second");
                printpositionfloatformat(gps.venus838data_filter.receivedtime, 1, "  receivedtime= ", "second");
                Serial.println("raw data:\n");
                printpositionfloatformat(gps.venus838data_raw.Latitude, 10000000, "  Latitude= ", "degree");
                printpositionfloatformat(gps.venus838data_raw.Longitude, 10000000, "  Longitude= ", "degree");
                printpositionfloatformat(gps.venus838data_raw.SealevelAltitude, 100, "  SealevelAltitude= ", "meter");
                printpositionfloatformat(gps.venus838data_raw.velocity, 100, "  velocity= ", "meter/second");
                
                Serial.println("Added data:\n");
                const String fixmodmask[]={"no fix", "2D", "3D", "3D+DGNSS"};
                Serial.println("fixmode=");
                Serial.println(fixmodmask[gps.venus838data_raw.fixmode]);
                Serial.println("NumSV=");
                Serial.println(gps.venus838data_raw.NumSV);
             
                printpositionfloatformat(gps.venus838data_raw.GDOP, 100, "  GDOP= ", "");
                printpositionfloatformat(gps.venus838data_raw.PDOP, 100, "  PDOP= ", "");
                printpositionfloatformat(gps.venus838data_raw.HDOP, 100, "  HDOP= ", "");
                printpositionfloatformat(gps.venus838data_raw.VDOP, 100, "  VDOP= ", "");
                printpositionfloatformat(gps.venus838data_raw.TDOP, 100, "  TDOP= ", "");

                */
                
                 
           }else
                  Serial.println("Receiving has been failed");
                  
        // dt=t1-t0;
        // Serial.println(dt);
        // t0=t1;
       }
     }
    dataFile.close();
}  

void incFileNum() 
{ // generate next file name:
  String s = "GPS" + String(++fileNum) + ".TXT";
  s.toCharArray(namefile,13);
}

float haverSine(float lat1, float lon1, float lat2, float lon2)
{
  double ToRad = PI / 180.0;
//  float R = 6371;   // radius earth in Km, change for other planets :)
  float R = 6371000; // radius earth in meters
  
  double dLat = (lat2-lat1) * ToRad;
  double dLon = (lon2-lon1) * ToRad;
  
  double a = sin(dLat/2) * sin(dLat/2) +
        cos(lat1 * ToRad) * cos(lat2 * ToRad) *
        sin(dLon/2) * sin(dLon/2);
        
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  float d = R * c;
  return d;
}




