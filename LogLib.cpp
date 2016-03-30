#include "LogLib.h"

#define memBase1      0
#define memBase2    512
#define memBase3   1024
#define EEPROMSize 2048
//int chipSelect = 4; //Arduino Mega
//int chipSelect = 6; //TeensyGPS version 1.0
int chipSelect = 15; //TeensyGPS version 1.1
int mosi = 7;
int miso = 8;
int sck = 14;


Sd2Card card;
File dataFile;
boolean sd_datalog = 1;
boolean log_output;
boolean parse_success;
boolean file_open = false;
boolean file_log = true;
boolean config_open = true;
int count;
int checksums = 0;
int fileNum = 0; 
char namefile[13]="LOG00001.CSV";
char nameConfig[12]="config.jsn";

StaticJsonBuffer<500> jsonBuffer1;
StaticJsonBuffer<500> jsonBuffer2;
StaticJsonBuffer<500> jsonBuffer3;
const char* classConfig[3];

float CNF [27];
boolean TPV [22];
boolean ATT [24];
String CNF_name[27] = {"log_en", 
                       "can_en", 
                       "newlog", 
                       "rate", 
                       "size", 
                       "blat", 
                       "blong",
                       "btime", 
                       "btol", 
                       "log_type", 
                       "trig", 
                       "trigv", 
                       "intv", 
                       "min", 
                       "max",
                       "fls_en",
                       "set_norm_low",
                       "lat_a",
                       "long_a",
                       "long_b",
                       "output_line",
                       "max_speed",
                       "min_speed",
                       "check_min_speed",
                       "check_max_speed"};
String TPV_name[22] = {"device", 
                       "mode", 
                       "sv",
                       "time", 
                       "lat", 
                       "lon", 
                       "alt", 
                       "latf", 
                       "lonf", 
                       "altf", 
                       "track", 
                       "speed", 
                       "speedf", 
                       "gdop", 
                       "pdop", 
                       "hdop", 
                       "vdop", 
                       "tdop",
                       "error"};
String ATT_name[24] = {"device", 
                       "time", 
                       "heading", 
                       "pitch", 
                       "yaw", 
                       "roll",
                       "dip", 
                       "mag_len", 
                       "mag_x", 
                       "mag_y", 
                       "mag_z", 
                       "acc_len", 
                       "acc_x", 
                       "acc_y", 
                       "acc_z", 
                       "gyro_x", 
                       "gyro_y",
                       "gyro_z",
                       "quat1",
                       "quat2",
                       "quat3",
                       "quat4",
                       "temp"};


String SetNulls(){
  String n; 
  int numNulls = 0;
  if (fileNum > 65533)
     fileNum = 0;
  if (fileNum < 10000)
     numNulls = 1;
  if (fileNum < 1000)
     numNulls = 2;
  if (fileNum < 100)
     numNulls = 3;
  if (fileNum < 10)
     numNulls = 4;
  //generate next file name:
  while (numNulls){
    n = n + "0";
    numNulls--;
  }
  return n;
}

void incFileNum() { 
  fileNum++;
  String s = "LOG" + SetNulls() + String(fileNum) + ".CSV";
  s.toCharArray(namefile,13);
}


void update_EEPROM() {
  EEPROM.setMemPool(memBase1, EEPROMSize);
  EEPROM.writeBlock(memBase1, CNF);
  EEPROM.writeBlock(memBase2, TPV);
  EEPROM.writeBlock(memBase3, ATT);
}

void read_EEPROM() {
  EEPROM.setMemPool(memBase1, EEPROMSize);
  EEPROM.readBlock(memBase1, CNF);
  EEPROM.readBlock(memBase2, TPV);
  EEPROM.readBlock(memBase3, ATT);
}

void parseJSON() {
    char data, configData[3][500];
    int j = 0;
    int i = 0;
    int len[3];
    dataFile = SD.open(nameConfig, FILE_READ);
    if (dataFile){
        // make data to String;
        while (((data = dataFile.read()) >= 0)&&(i < 3)){
             if (data==10){}
             else if (data==0x7D){
                 configData[i][j++] = data;
                 len[i] = j;
                 j = 0;
                 i++;
             }
             else{
                 configData[i][j++] = data;
             }
        }
        Serial.println(nameConfig);
        for (i = 0; i < 3; i++){
            for (j = 0; j < len[i]; j++)
                Serial.print(configData[i][j]);
            Serial.println();
        }
    }
    else {
        Serial.println("config file is not found");
        config_open = false;
    }
    dataFile.close();
    if (config_open) {
      JsonObject& config1 = jsonBuffer1.parseObject(configData[0]);
      JsonObject& config2 = jsonBuffer2.parseObject(configData[1]);
      JsonObject& config3 = jsonBuffer3.parseObject(configData[2]);
      if (!(config1.success() && config2.success() && config3.success())) {
          Serial.println("parseObject() failed");
          parse_success = 0;
      }
      else{
          parse_success = 1;
      }
      classConfig[0] = config1["class"];
      for (i = 0; i < 26; i++){
          CNF[i] = config1 [CNF_name[i]];
      };
      classConfig[1] = config2["class"];
      for (i = 0; i < 21; i++){
          TPV[i] = config2 [TPV_name[i]];
      };
      classConfig[2] = config3["class"];
      for (i = 0; i < 23; i++){
          ATT[i] = config3 [ATT_name[i]];
      };
      Serial.println("Update EEPROM");
      update_EEPROM();
   }
   else{
      Serial.println("Read EEPROM");
      read_EEPROM();
      parse_success = 1;
   }
}

void printJSON(){
    Serial.print("CNF");
    Serial.print(" ");
    for (int i = 0; i < 26; i++){
        Serial.print(CNF[i]);
        Serial.print(" ");
    };
    Serial.println();
    Serial.print("TPV");
    Serial.print(" ");
    for (int i = 0; i < 21; i++){
        Serial.print(TPV[i]);
        Serial.print(" ");
    };
    Serial.println();
    Serial.print("ATT");
    Serial.print(" ");
    for (int i = 0; i < 23; i++){
        Serial.print(ATT[i]);
        Serial.print(" ");
    };
    Serial.println();
}

String check_TPV(int num){
  if (TPV[num])
     return TPV_name[num]+",";
  else 
     return "";
}

String check_ATT(int num){
  if (ATT[num])
     return ATT_name[num]+",";
  else 
     return "";
}

void LogSetup() {
 SPI.setMOSI(mosi);
 SPI.setMISO(miso);
 SPI.setSCK(sck);
 pinMode(chipSelect, OUTPUT);
 delay(1000);
 if ((!SD.begin(chipSelect))&&(file_log==true))
   {  
      Serial.println("File datalog enabled");    
      Serial.println("Card Not Present");
      led_on = LOW;
      digitalWrite(led,led_on);
      sd_datalog=false;
   }
 else
   {
      Serial.println("File datalog enabled");    
      Serial.println("Card Present");
      led_on = HIGH;
      digitalWrite(led,led_on);
      sd_datalog=true;
      card.init(SPI_FULL_SPEED, chipSelect);
   }
 if (sd_datalog==true) {
    parseJSON();
    printJSON();
    if (parse_success)
        log_output = CNF[0];
    if (log_output)
        create_newlog();
 }//if 
}

void create_newlog(){
    String header;
    while (SD.exists(namefile)) {      
        Serial.print(namefile);
        Serial.println(" file present."); 
        incFileNum();
    }
    Serial.print("Logging Data to "); 
    Serial.println(namefile);
    dataFile = SD.open(namefile, FILE_WRITE);           
    delay(100);
    if (dataFile){
        for (int i = 0; i < 21; i++){
           header = header + check_TPV(i);
        }
        header = "class," + header + "class,";
        for (int i = 0; i < 23; i++){
           header = header + check_ATT(i);
        }
        dataFile.println(header);
        Serial.println("Header wrote ok");
        dataFile.close();
    }
    else {
        Serial.println("Impossible to open datafile to write header");
        sd_datalog=0;
    }//if
}

void dataFloat(float value, int mode){
    char outstr[21];
    dtostrf(value, 20, 12, outstr);
    if (gps.venus838data_raw.fixmode >= mode)
        dataFile.print (outstr);
    dataFile.print(",");
}

void LogTPV(){
     Serial.println("Print TPV object"); 
     if (!dataFile) 
          dataFile = SD.open(namefile, FILE_WRITE);
      dataFile.print("TPV,");
      if(TPV[0])
          dataFile.print("Venus838,");       
      if(TPV[1]){
          dataFile.print(gps.venus838data_raw.fixmode);
          dataFile.print(",");   
      };
      if(TPV[2]){
          dataFile.print(gps.venus838data_raw.NumSV);
          dataFile.print(",");  
      };
      if(TPV[3]){
          dataFile.print(UTC_Time);
          dataFile.print(",");
      };
      if(TPV[4])
          dataFloat(gps.venus838data_raw.Latitude, 1);
      if(TPV[5])
          dataFloat(gps.venus838data_raw.Longitude, 1);
      if(TPV[6])
          dataFloat(gps.venus838data_raw.SealevelAltitude, 2);
      if(TPV[7])
          dataFloat(gps.venus838data_filter.Latitude, 1);
      if(TPV[8])
          dataFloat(gps.venus838data_filter.Longitude, 1);
      if(TPV[9])
          dataFloat(gps.venus838data_filter.SealevelAltitude, 2);
      if(TPV[10])
          dataFloat(course_angle, 0);
      if(TPV[11])
          dataFloat(gps.venus838data_raw.velocity, 0);
      if(TPV[12])
          dataFloat(gps.venus838data_filter.velocity, 0);
      if(TPV[13])
          dataFloat(gps.venus838data_raw.gdop, 0);
      if(TPV[14])
          dataFloat(gps.venus838data_raw.pdop, 0);
      if(TPV[15])
          dataFloat(gps.venus838data_raw.hdop, 0);
      if(TPV[16])
          dataFloat(gps.venus838data_raw.vdop, 0);      
      if(TPV[17])
          dataFloat(gps.venus838data_raw.tdop, 0);
      if(TPV[18]){
          dataFile.print(checksums);  
          dataFile.print(",");
      };
}

void LogATT(){
    att.heading = heading;
    att.pitch = pitch;
    att.yaw = yaw;
    att.roll = roll;
    att.dip = inclination;
    att.mag_len = sqrt(sq(mx)+sq(my)+sq(mz));
    att.mag_x = mx;
    att.mag_y = my;
    att.mag_z = mz;
    att.acc_len = sqrt(sq(ax)+sq(ay)+sq(az));
    att.acc_x = ax;
    att.acc_y = ay;
    att.acc_z = az;
    att.gyro_x = gx;
    att.gyro_y = gy;
    att.gyro_z = gz;
    att.quat1 = q[0];
    att.quat2 = q[1];
    att.quat3 = q[2];
    att.quat4 = q[3];
    att.temp = temp;
    Serial.println("Print ATT object"); 
    if (!dataFile) 
         dataFile = SD.open(namefile, FILE_WRITE);
    dataFile.print("ATT,");
    if(ATT[0])
        dataFile.print("LSM9DS0TR,");
    if(ATT[1]){
        dataFile.print(UTC_Time);
        dataFile.print(",");
    };
    if(ATT[2])
        dataFloat(att.heading, 0);
    if(ATT[3])
        dataFloat(att.pitch, 0);
    if(ATT[4])
        dataFloat(att.yaw, 0);
    if(ATT[5])
        dataFloat(att.roll, 0);
    if(ATT[6])
        dataFloat(att.dip, 0);
    if(ATT[7])
        dataFloat(att.mag_len, 0);
    if(ATT[8])
        dataFloat(att.mag_x, 0);
    if(ATT[9])
        dataFloat(att.mag_y, 0);
    if(ATT[10])
        dataFloat(att.mag_z, 0);
    if(ATT[11])
        dataFloat(att.acc_len, 0);
    if(ATT[12])
        dataFloat(att.acc_x, 0);
    if(ATT[13])
        dataFloat(att.acc_y, 0);
    if(ATT[14])
        dataFloat(att.acc_z, 0);
    if(ATT[15])
        dataFloat(att.gyro_x, 0);
    if(ATT[16])
        dataFloat(att.gyro_y, 0);
    if(ATT[17])
        dataFloat(att.gyro_z, 0);
    if(ATT[18])
        dataFloat(att.quat1, 0);
    if(ATT[19])
        dataFloat(att.quat2, 0);
    if(ATT[20])
        dataFloat(att.quat3, 0); 
    if(ATT[21])
        dataFloat(att.quat4, 0);
    if(ATT[22])
        dataFloat(att.temp, 0);
    dataFile.println();          
    dataFile.flush();
    filesize = dataFile.size();
    Serial.println("Datafile Saved");
    Serial.println(filesize);
    dataFile.close();
}

boolean TIMECONV_GetJulianDateFromGPSTime(
   const int              gps_week,      //!< GPS week (0-1024+)             [week]
   const unsigned long    gps_tow,       //!< GPS time of week (0-604800.0)  [s]
   const int              utc_offset,    //!< Integer seconds that GPS is ahead of UTC time, always positive [s]
   double                 &julian_date    //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
   )
{   
   if( gps_tow < 0.0  || gps_tow > 604800.0 )
     return false;  
   // GPS time is ahead of UTC time and Julian time by the UTC offset
   julian_date = (double(gps_week) + (gps_tow-utc_offset)/604800.0)*7.0 + TIMECONV_JULIAN_DATE_START_OF_GPS_TIME;  
   return true;
}

boolean TIMECONV_GetUTCTimeFromJulianDate(
   const double        julian_date,  //!< Number of days since noon Universal Time Jan 1, 4713 BCE (Julian calendar) [days]
   unsigned short*     utc_year,     //!< Universal Time Coordinated    [year]
   unsigned char*      utc_month,    //!< Universal Time Coordinated    [1-12 months] 
   unsigned char*      utc_day,      //!< Universal Time Coordinated    [1-31 days]
   unsigned char*      utc_hour,     //!< Universal Time Coordinated    [hours]
   unsigned char*      utc_minute,   //!< Universal Time Coordinated    [minutes]
   float*              utc_seconds   //!< Universal Time Coordinated    [s]
   )
 {
   int a, b, c, d, e; // temporary values
   
   unsigned short year;  
   unsigned char month;
   unsigned char day;
   unsigned char hour;
   unsigned char minute;        
   unsigned char days_in_month = 0;  
   double td; // temporary double
   double seconds;
   boolean result;
 
   // Check the input.
   if( julian_date < 0.0 )
     return false;
   
   a = (int)(julian_date+0.5);
   b = a + 1537;
   c = (int)( ((double)b-122.1)/365.25 );
   d = (int)(365.25*c);
   e = (int)( ((double)(b-d))/30.6001 );
   td      = b - d - (int)(30.6001*e) + fmod( julian_date+0.5, 1.0 );   // [days]
   day     = (unsigned char)td;     
   td     -= day;
   td     *= 24.0;        // [hours]
   hour    = (unsigned char)td;
   td     -= hour;
   td     *= 60.0;        // [minutes]
   minute  = (unsigned char)td;
   td     -= minute;
   td     *= 60.0;        // [s]
   seconds = td;
   month   = (unsigned char)(e - 1 - 12*(int)(e/14));
   year    = (unsigned short)(c - 4715 - (int)( (7.0+(double)month) / 10.0 ));
   
   // check for rollover issues
   if( seconds >= 60.0 )
   {
     seconds -= 60.0;
     minute++;
     if( minute >= 60 )
     {
       minute -= 60;
       hour++;
       if( hour >= 24 )
       {
         hour -= 24;
         day++;
         
         result = TIMECONV_GetNumberOfDaysInMonth( year, month, &days_in_month );
         if( result == false )
           return false;
         
         if( day > days_in_month )
         {
           day = 1;
           month++;
           if( month > 12 )
           {
             month = 1;
             year++;
           }
         }
       }
     }
   }   
   
   *utc_year       = year;
   *utc_month      = month;
   *utc_day        = day;
   *utc_hour       = hour;
   *utc_minute     = minute;
   *utc_seconds    = (float)seconds;   
 
   return true;
}

boolean TIMECONV_GetNumberOfDaysInMonth(
   const unsigned short year,        //!< Universal Time Coordinated    [year]
   const unsigned char month,        //!< Universal Time Coordinated    [1-12 months] 
   unsigned char* days_in_month      //!< Days in the specified month   [1-28|29|30|31 days]
   )
{
   boolean is_a_leapyear;
   unsigned char utmp = 0;
   
   is_a_leapyear = TIMECONV_IsALeapYear( year );
   
   switch(month)
   {
   case  1: utmp = TIMECONV_DAYS_IN_JAN; break;
   case  2: if( is_a_leapyear ){ utmp = 29; }else{ utmp = 28; }break;    
   case  3: utmp = TIMECONV_DAYS_IN_MAR; break;
   case  4: utmp = TIMECONV_DAYS_IN_APR; break;
   case  5: utmp = TIMECONV_DAYS_IN_MAY; break;
   case  6: utmp = TIMECONV_DAYS_IN_JUN; break;
   case  7: utmp = TIMECONV_DAYS_IN_JUL; break;
   case  8: utmp = TIMECONV_DAYS_IN_AUG; break;
   case  9: utmp = TIMECONV_DAYS_IN_SEP; break;
   case 10: utmp = TIMECONV_DAYS_IN_OCT; break;
   case 11: utmp = TIMECONV_DAYS_IN_NOV; break;
   case 12: utmp = TIMECONV_DAYS_IN_DEC; break;
   default: return false; break;    
   }
   
   *days_in_month = utmp;
 
   return true;
 }

boolean TIMECONV_IsALeapYear(const unsigned short year )
{
   boolean is_a_leap_year = false;
 
   if( (year%4) == 0 )
   {
     is_a_leap_year = true;
     if( (year%100) == 0 )
     {
       if( (year%400) == 0 )
       {
         is_a_leap_year = true;
       }
       else
       {
         is_a_leap_year = false;
       }
     }
   }
   if( is_a_leap_year )
   {
     return true;
   }
   else
   {
     return false;
   }
}

String GetUTCTime (unsigned int num_week, unsigned long timeofweek){
  String UTC_Time, UTC_Date;
  unsigned short    utc_year;
  unsigned char     sec_, utc_month, utc_day, utc_hour, utc_minute;
  float   utc_seconds;
  double julian_date;
  sec_ = timeofweek % 100;
  timeofweek = timeofweek/100;
  TIMECONV_GetJulianDateFromGPSTime(num_week, timeofweek, 17, julian_date);
  TIMECONV_GetUTCTimeFromJulianDate(julian_date, &utc_year, &utc_month, &utc_day, &utc_hour, &utc_minute, &utc_seconds);
  UTC_Date = String(utc_year) + '-' + String(utc_month) + '-' + String(utc_day);
  UTC_Time = 'T' + null_add(utc_hour) + String(utc_hour) + ':' + null_add(utc_minute)  + String(utc_minute) + ':' + null_add(utc_seconds) +
  String(utc_seconds) + '.' + null_add(sec_) + String(sec_);
  return (UTC_Date + UTC_Time);
}

String null_add(int value){
  String null;
  if (value < 10)
     null = '0';
  else 
     null = "";
  return null;
}


