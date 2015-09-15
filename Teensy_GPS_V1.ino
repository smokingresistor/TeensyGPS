#include "MatrixMath.h"
//#include <SoftwareSerial.h>    //in case arduino board
#include "BMsg838.h"
#include "KalmanFilter.h"
#include "GPSSerialMessageCom.h"
#include <SPI.h>
#include <SD.h>
#include <FlexCAN.h>

const String fixmodmask[]={"no fix", "2D", "3D", "3D+DGNSS"};
/* This sample code demonstrates the normal use of the binary message of 
   SkyTraq Venus 8 GNSS Receiver.
   this is used on teensy3.1
*/
  union conv 
  {
    float f;
    uint8_t b[8];
  };
  
  
static CAN_message_t can_lat,can_lon,can_vel,can_alt,can_service;
FlexCAN CANbus(500000);
/*gps interfacce relate class*/
BMsg838 gps;

/*kalman filter class*/
KalmanFi1ter filter;
union conv lat, lon, alt, vel;
int fileNum = 10000; 
char namefile[13]="GPS10000.TXT";
int chipSelect = 6;
int mosi = 7;
int miso = 8;
int sck = 14;
int led = 13;
const byte id_lat=0x001;
const byte id_lon=0x002;
const byte id_alt=0x003;
const byte id_vel=0x004;
const byte id_service=0x005;
// File dataFile;
Sd2Card card;
// SdVolume volume;
// SdFile root;
boolean led_on=1;
boolean sd_datalog=1;
boolean file_open=0;
boolean can_output=1;
boolean file_log=1;
double t0,t1,dt=0;
int count,checksums=0;

void incFileNum() 
{ // generate next file name:
  String s = "GPS" + String(++fileNum) + ".TXT";
  s.toCharArray(namefile,13);
}



void setup()
{
 delay(10000);    
 SPI.setMOSI(mosi);
 SPI.setMISO(miso);
 SPI.setSCK(sck);
 pinMode(chipSelect, OUTPUT);
 pinMode(led, OUTPUT);
 if (can_output) 
    {       
       can_lat.id=id_lat;
       can_lat.len = 8;
       can_lon.id=id_lon;
       can_lon.len = 8;
       can_vel.id=id_vel;
       can_vel.len = 8;
       can_alt.id=id_alt;
       can_alt.len = 8;
       can_service.id=id_service;
       can_service.len = 8;
       CANbus.begin(); 
       digitalWrite(led,led_on); 
    }
 
 if ((!SD.begin(chipSelect))&&(file_log))
   {      
      Serial.println("Card Not Present");
      sd_datalog=0;
   }
 else
   {
      Serial.println("Card Present");
      sd_datalog=1;
      card.init(SPI_FULL_SPEED, chipSelect);
   }
  
  if (sd_datalog==1);
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
  } 
      


  Serial.begin(115200);
  Serial2.begin(115200);
  char messagetype[64];
  memset(messagetype,0,64);
  Serial.print("Testing BMsg838 binary message library v. "); 
  Serial.println();
  // Serial.print(" BMsg838 System reset.\r\n"); 
  // SendBinaryMessagetoGPSreceiver(gps.ResetGNSS(1, 15, 6, 9,11, 30, 25, 20, 133, 1200), gps.SendStream,gps.RecVBinarybuf,0,2000);
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
  Serial.println("Received_time;Latitude;Logitude;Altitude;Velocity;Latitude_fil;Logitude_fil;Altitude_fil;Velocity_fil;fix_mode;sat_num;checksums");
  Serial.println("-------------------------------------------------------------------------------------------------------------------------------------");

    if (sd_datalog==1) 
      {
        File dataFile = SD.open(namefile, FILE_WRITE);           
        delay(100);
        if (dataFile)
          {
            dataFile.println("-------------------------------------------------------------------------------------------------------------------------------------");
            dataFile.println("Received_time;Latitude;Logitude;Altitude;Velocity;Latitude_fil;Logitude_fil;Altitude_fil;Velocity_fil;fix_mode;sat_num;checksums");
            dataFile.println("-------------------------------------------------------------------------------------------------------------------------------------");
            dataFile.close();
          }
        else Serial.println("Impossible to open datafile to write header");
      }
      

} 

//receiv Navigation binary data from GPS receiver and find positopn and velocity data 
//and after do kalmanfiltering 

void loop()
{
  File dataFile = SD.open(namefile, FILE_WRITE);   
  while (1)
  {
  int ret=0;
  char messagetype[64];
  memset(messagetype,0,64);
        if(Serial2.available()){
           // t1=millis();
           ret=waitingRespondandReceive(gps.RecVBinarybuf,0xA8,2000); 
           if(ret>7){                
                 if(!GPSNavigationMsgProcessing(&(gps.venus838data_raw),&(gps.venus838data_filter),gps,filter))
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
                        lat.f=gps.venus838data_raw.Latitude;                       
                        lon.f=gps.venus838data_raw.Longitude;
                        alt.f=gps.venus838data_raw.SealevelAltitude;
                        vel.f=gps.venus838data_raw.velocity;                        
                        for( int idx=0; idx<8; ++idx ) 
                              {
                                 can_lat.buf[idx]=lat.b[idx]; 
                              }
                        for( int idx=0; idx<8; ++idx ) 
                              {
                                 can_lon.buf[idx]=lon.b[idx]; 
                              }
                        for( int idx=0; idx<8; ++idx ) 
                              {
                                 can_alt.buf[idx]=alt.b[idx]; 
                              }    
                        for( int idx=0; idx<8; ++idx ) 
                              {
                                 can_vel.buf[idx]=vel.b[idx]; 
                              }
                       can_service.buf[0]=gps.venus838data_raw.NumSV;
                       can_service.buf[1]=gps.venus838data_raw.fixmode;   
                       CANbus.write(can_lat);  
                       CANbus.write(can_lon); 
                       CANbus.write(can_alt);
                       CANbus.write(can_vel);
                       CANbus.write(can_service);
                       led_on=!led_on;
                       digitalWrite(led,led_on);
                       
                   }
                   
                      if ((sd_datalog==1)&&(gps.venus838data_raw.fixmode>0))
                     
                          {
                               // File dataFile = SD.open(namefile, FILE_WRITE);
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
                                if (count%500==0) dataFile.flush();
                                count++;
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

