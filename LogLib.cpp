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

float CNF [17];
boolean TPV [21];
boolean ATT [18];
String CNF_name[17] = {"log_en", 
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
                       "max"};
String TPV_name[21] = {"device", 
                       "mode", 
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
                       "tdop",};
String ATT_name[18] = {"device", 
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
                       "gyro_y"};

struct DOF_DATA att;

static const int16_t dec_tbl[37][73] = \
{ \
    {150,145,140,135,130,125,120,115,110,105,100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5,0,-4,-9,-14,-19,-24,-29,-34,-39,-44,-49,-54,-59,-64,-69,-74,-79,-84,-89,-94,-99,104,109,114,119,124,129,134,139,144,149,154,159,164,169,174,179,175,170,165,160,155,150}, \
    {143,137,131,126,120,115,110,105,100,95,90,85,80,75,71,66,62,57,53,48,44,39,35,31,27,22,18,14,9,5,1,-3,-7,-11,-16,-20,-25,-29,-34,-38,-43,-47,-52,-57,-61,-66,-71,-76,-81,-86,-91,-96,101,107,112,117,123,128,134,140,146,151,157,163,169,175,178,172,166,160,154,148,143}, \
    {130,124,118,112,107,101,96,92,87,82,78,74,70,65,61,57,54,50,46,42,38,34,31,27,23,19,16,12,8,4,1,-2,-6,-10,-14,-18,-22,-26,-30,-34,-38,-43,-47,-51,-56,-61,-65,-70,-75,-79,-84,-89,-94,100,105,111,116,122,128,135,141,148,155,162,170,177,174,166,159,151,144,137,130}, \
    {111,104,99,94,89,85,81,77,73,70,66,63,60,56,53,50,46,43,40,36,33,30,26,23,20,16,13,10,6,3,0,-3,-6,-9,-13,-16,-20,-24,-28,-32,-36,-40,-44,-48,-52,-57,-61,-65,-70,-74,-79,-84,-88,-93,-98,103,109,115,121,128,135,143,152,162,172,176,165,154,144,134,125,118,111}, \
    {85,81,77,74,71,68,65,63,60,58,56,53,51,49,46,43,41,38,35,32,29,26,23,19,16,13,10,7,4,1,-1,-3,-6,-9,-13,-16,-19,-23,-26,-30,-34,-38,-42,-46,-50,-54,-58,-62,-66,-70,-74,-78,-83,-87,-91,-95,100,105,110,117,124,133,144,159,178,160,141,125,112,103,96,90,85 }, \
    {62,60,58,57,55,54,52,51,50,48,47,46,44,42,41,39,36,34,31,28,25,22,19,16,13,10,7,4,2,0,-3,-5,-8,-10,-13,-16,-19,-22,-26,-29,-33,-37,-41,-45,-49,-53,-56,-60,-64,-67,-70,-74,-77,-80,-83,-86,-89,-91,-94,-97,101,105,111,130,109,84,77,74,71,68,66,64,62 }, \
    {46,46,45,44,44,43,42,42,41,41,40,39,38,37,36,35,33,31,28,26,23,20,16,13,10,7,4,1,-1,-3,-5,-7,-9,-12,-14,-16,-19,-22,-26,-29,-33,-36,-40,-44,-48,-51,-55,-58,-61,-64,-66,-68,-71,-72,-74,-74,-75,-74,-72,-68,-61,-48,-25,2,22,33,40,43,45,46,47,46,46 }, \
    {36,36,36,36,36,35,35,35,35,34,34,34,34,33,32,31,30,28,26,23,20,17,14,10,6,3,0,-2,-4,-7,-9,-10,-12,-14,-15,-17,-20,-23,-26,-29,-32,-36,-40,-43,-47,-50,-53,-56,-58,-60,-62,-63,-64,-64,-63,-62,-59,-55,-49,-41,-30,-17,-4,6,15,22,27,31,33,34,35,36,36 }, \
    {30,30,30,30,30,30,30,29,29,29,29,29,29,29,29,28,27,26,24,21,18,15,11,7,3,0,-3,-6,-9,-11,-12,-14,-15,-16,-17,-19,-21,-23,-26,-29,-32,-35,-39,-42,-45,-48,-51,-53,-55,-56,-57,-57,-56,-55,-53,-49,-44,-38,-31,-23,-14,-6,0,7,13,17,21,24,26,27,29,29,30 }, \
    {25,25,26,26,26,25,25,25,25,25,25,25,25,26,25,25,24,23,21,19,16,12,8,4,0,-3,-7,-10,-13,-15,-16,-17,-18,-19,-20,-21,-22,-23,-25,-28,-31,-34,-37,-40,-43,-46,-48,-49,-50,-51,-51,-50,-48,-45,-42,-37,-32,-26,-19,-13,-7,-1,3,7,11,14,17,19,21,23,24,25,25 }, \
    {21,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,21,20,18,16,13,9,5,1,-3,-7,-11,-14,-17,-18,-20,-21,-21,-22,-22,-22,-23,-23,-25,-27,-29,-32,-35,-37,-40,-42,-44,-45,-45,-45,-44,-42,-40,-36,-32,-27,-22,-17,-12,-7,-3,0,3,7,9,12,14,16,18,19,20,21,21 }, \
    {18,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,18,17,16,14,10,7,2,-1,-6,-10,-14,-17,-19,-21,-22,-23,-24,-24,-24,-24,-23,-23,-23,-24,-26,-28,-30,-33,-35,-37,-38,-39,-39,-38,-36,-34,-31,-28,-24,-19,-15,-10,-6,-3,0,1,4,6,8,10,12,14,15,16,17,18,18 }, \
    {16,16,17,17,17,17,17,17,17,17,17,16,16,16,16,16,16,15,13,11,8,4,0,-4,-9,-13,-16,-19,-21,-23,-24,-25,-25,-25,-25,-24,-23,-21,-20,-20,-21,-22,-24,-26,-28,-30,-31,-32,-31,-30,-29,-27,-24,-21,-17,-13,-9,-6,-3,-1,0,2,4,5,7,9,10,12,13,14,15,16,16 }, \
    {14,14,14,15,15,15,15,15,15,15,14,14,14,14,14,14,13,12,11,9,5,2,-2,-6,-11,-15,-18,-21,-23,-24,-25,-25,-25,-25,-24,-22,-21,-18,-16,-15,-15,-15,-17,-19,-21,-22,-24,-24,-24,-23,-22,-20,-18,-15,-12,-9,-5,-3,-1,0,1,2,4,5,6,8,9,10,11,12,13,14,14 }, \
    {12,13,13,13,13,13,13,13,13,13,13,13,12,12,12,12,11,10,9,6,3,0,-4,-8,-12,-16,-19,-21,-23,-24,-24,-24,-24,-23,-22,-20,-17,-15,-12,-10,-9,-9,-10,-12,-13,-15,-17,-17,-18,-17,-16,-15,-13,-11,-8,-5,-3,-1,0,1,1,2,3,4,6,7,8,9,10,11,12,12,12 }, \
    {11,11,11,11,11,12,12,12,12,12,11,11,11,11,11,10,10,9,7,5,2,-1,-5,-9,-13,-17,-20,-22,-23,-23,-23,-23,-22,-20,-18,-16,-14,-11,-9,-6,-5,-4,-5,-6,-8,-9,-11,-12,-12,-12,-12,-11,-9,-8,-6,-3,-1,0,0,1,1,2,3,4,5,6,7,8,9,10,11,11,11 }, \
    {10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,9,7,6,3,0,-3,-6,-10,-14,-17,-20,-21,-22,-22,-22,-21,-19,-17,-15,-13,-10,-8,-6,-4,-2,-2,-2,-2,-4,-5,-7,-8,-8,-9,-8,-8,-7,-5,-4,-2,0,0,1,1,1,2,2,3,4,5,6,7,8,9,10,10,10 }, \
    {9,9,9,9,9,9,9,10,10,9,9,9,9,9,9,8,8,6,5,2,0,-4,-7,-11,-15,-17,-19,-21,-21,-21,-20,-18,-16,-14,-12,-10,-8,-6,-4,-2,-1,0,0,0,-1,-2,-4,-5,-5,-6,-6,-5,-5,-4,-3,-1,0,0,1,1,1,1,2,3,3,5,6,7,8,8,9,9,9  }, \
    {9,9,9,9,9,9,9,9,9,9,9,9,8,8,8,8,7,5,4,1,-1,-5,-8,-12,-15,-17,-19,-20,-20,-19,-18,-16,-14,-11,-9,-7,-5,-4,-2,-1,0,0,1,1,0,0,-2,-3,-3,-4,-4,-4,-3,-3,-2,-1,0,0,0,0,0,1,1,2,3,4,5,6,7,8,8,9,9  }, \
    {9,9,9,8,8,8,9,9,9,9,9,8,8,8,8,7,6,5,3,0,-2,-5,-9,-12,-15,-17,-18,-19,-19,-18,-16,-14,-12,-9,-7,-5,-4,-2,-1,0,0,1,1,1,1,0,0,-1,-2,-2,-3,-3,-2,-2,-1,-1,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8,8,9  }, \
    {8,8,8,8,8,8,9,9,9,9,9,9,8,8,8,7,6,4,2,0,-3,-6,-9,-12,-15,-17,-18,-18,-17,-16,-14,-12,-10,-8,-6,-4,-2,-1,0,0,1,2,2,2,2,1,0,0,-1,-1,-1,-2,-2,-1,-1,0,0,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8,8  }, \
    {8,8,8,8,9,9,9,9,9,9,9,9,9,8,8,7,5,3,1,-1,-4,-7,-10,-13,-15,-16,-17,-17,-16,-15,-13,-11,-9,-6,-5,-3,-2,0,0,0,1,2,2,2,2,1,1,0,0,0,-1,-1,-1,-1,-1,0,0,0,0,-1,-1,-1,-1,-1,0,0,1,3,4,5,7,7,8  }, \
    {8,8,9,9,9,9,10,10,10,10,10,10,10,9,8,7,5,3,0,-2,-5,-8,-11,-13,-15,-16,-16,-16,-15,-13,-12,-10,-8,-6,-4,-2,-1,0,0,1,2,2,3,3,2,2,1,0,0,0,0,0,0,0,0,0,0,-1,-1,-2,-2,-2,-2,-2,-1,0,0,1,3,4,6,7,8  }, \
    {7,8,9,9,9,10,10,11,11,11,11,11,10,10,9,7,5,3,0,-2,-6,-9,-11,-13,-15,-16,-16,-15,-14,-13,-11,-9,-7,-5,-3,-2,0,0,1,1,2,3,3,3,3,2,2,1,1,0,0,0,0,0,0,0,-1,-1,-2,-3,-3,-4,-4,-4,-3,-2,-1,0,1,3,5,6,7  }, \
    {6,8,9,9,10,11,11,12,12,12,12,12,11,11,9,7,5,2,0,-3,-7,-10,-12,-14,-15,-16,-15,-15,-13,-12,-10,-8,-7,-5,-3,-1,0,0,1,2,2,3,3,4,3,3,3,2,2,1,1,1,0,0,0,0,-1,-2,-3,-4,-4,-5,-5,-5,-5,-4,-2,-1,0,2,3,5,6  }, \
    {6,7,8,10,11,12,12,13,13,14,14,13,13,11,10,8,5,2,0,-4,-8,-11,-13,-15,-16,-16,-16,-15,-13,-12,-10,-8,-6,-5,-3,-1,0,0,1,2,3,3,4,4,4,4,4,3,3,3,2,2,1,1,0,0,-1,-2,-3,-5,-6,-7,-7,-7,-6,-5,-4,-3,-1,0,2,4,6  }, \
    {5,7,8,10,11,12,13,14,15,15,15,14,14,12,11,8,5,2,-1,-5,-9,-12,-14,-16,-17,-17,-16,-15,-14,-12,-11,-9,-7,-5,-3,-1,0,0,1,2,3,4,4,5,5,5,5,5,5,4,4,3,3,2,1,0,-1,-2,-4,-6,-7,-8,-8,-8,-8,-7,-6,-4,-2,0,1,3,5  }, \
    {4,6,8,10,12,13,14,15,16,16,16,16,15,13,11,9,5,2,-2,-6,-10,-13,-16,-17,-18,-18,-17,-16,-15,-13,-11,-9,-7,-5,-4,-2,0,0,1,3,3,4,5,6,6,7,7,7,7,7,6,5,4,3,2,0,-1,-3,-5,-7,-8,-9,-10,-10,-10,-9,-7,-5,-4,-1,0,2,4  }, \
    {4,6,8,10,12,14,15,16,17,18,18,17,16,15,12,9,5,1,-3,-8,-12,-15,-18,-19,-20,-20,-19,-18,-16,-15,-13,-11,-8,-6,-4,-2,-1,0,1,3,4,5,6,7,8,9,9,9,9,9,9,8,7,5,3,1,-1,-3,-6,-8,-10,-11,-12,-12,-11,-10,-9,-7,-5,-2,0,1,4  }, \
    {4,6,8,11,13,15,16,18,19,19,19,19,18,16,13,10,5,0,-5,-10,-15,-18,-21,-22,-23,-22,-22,-20,-18,-17,-14,-12,-10,-8,-5,-3,-1,0,1,3,5,6,8,9,10,11,12,12,13,12,12,11,9,7,5,2,0,-3,-6,-9,-11,-12,-13,-13,-12,-11,-10,-8,-6,-3,-1,1,4  }, \
    {3,6,9,11,14,16,17,19,20,21,21,21,19,17,14,10,4,-1,-8,-14,-19,-22,-25,-26,-26,-26,-25,-23,-21,-19,-17,-14,-12,-9,-7,-4,-2,0,1,3,5,7,9,11,13,14,15,16,16,16,16,15,13,10,7,4,0,-3,-7,-10,-12,-14,-15,-14,-14,-12,-11,-9,-6,-4,-1,1,3  }, \
    {4,6,9,12,14,17,19,21,22,23,23,23,21,19,15,9,2,-5,-13,-20,-25,-28,-30,-31,-31,-30,-29,-27,-25,-22,-20,-17,-14,-11,-9,-6,-3,0,1,4,6,9,11,13,15,17,19,20,21,21,21,20,18,15,11,6,2,-2,-7,-11,-13,-15,-16,-16,-15,-13,-11,-9,-7,-4,-1,1,4  }, \
    {4,7,10,13,15,18,20,22,24,25,25,25,23,20,15,7,-2,-12,-22,-29,-34,-37,-38,-38,-37,-36,-34,-31,-29,-26,-23,-20,-17,-13,-10,-7,-4,-1,2,5,8,11,13,16,18,21,23,24,26,26,26,26,24,21,17,12,5,0,-6,-10,-14,-16,-16,-16,-15,-14,-12,-10,-7,-4,-1,1,4  }, \
    {4,7,10,13,16,19,22,24,26,27,27,26,24,19,11,-1,-15,-28,-37,-43,-46,-47,-47,-45,-44,-41,-39,-36,-32,-29,-26,-22,-19,-15,-11,-8,-4,-1,2,5,9,12,15,19,22,24,27,29,31,33,33,33,32,30,26,21,14,6,0,-6,-11,-14,-15,-16,-15,-14,-12,-9,-7,-4,-1,1,4  }, \
    {6,9,12,15,18,21,23,25,27,28,27,24,17,4,-14,-34,-49,-56,-60,-60,-60,-58,-56,-53,-50,-47,-43,-40,-36,-32,-28,-25,-21,-17,-13,-9,-5,-1,2,6,10,14,17,21,24,28,31,34,37,39,41,42,43,43,41,38,33,25,17,8,0,-4,-8,-10,-10,-10,-8,-7,-4,-2,0,3,6  }, \
    {22,24,26,28,30,32,33,31,23,-18,-81,-96,-99,-98,-95,-93,-89,-86,-82,-78,-74,-70,-66,-62,-57,-53,-49,-44,-40,-36,-32,-27,-23,-19,-14,-10,-6,-1,2,6,10,15,19,23,27,31,35,38,42,45,49,52,55,57,60,61,63,63,62,61,57,53,47,40,33,28,23,21,19,19,19,20,22 }, \
    {168,173,178,176,171,166,161,156,151,146,141,136,131,126,121,116,111,106,101,-96,-91,-86,-81,-76,-71,-66,-61,-56,-51,-46,-41,-36,-31,-26,-21,-16,-11,-6,-1,3,8,13,18,23,28,33,38,43,48,53,58,63,68,73,78,83,88,93,98,103,108,113,118,123,128,133,138,143,148,153,158,163,168}, \
};

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
      for (i = 0; i < 16; i++){
          CNF[i] = config1 [CNF_name[i]];
      };
      classConfig[1] = config2["class"];
      for (i = 0; i < 20; i++){
          TPV[i] = config2 [TPV_name[i]];
      };
      classConfig[2] = config3["class"];
      for (i = 0; i < 17; i++){
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
    for (int i = 0; i < 15; i++){
        Serial.print(CNF[i]);
        Serial.print(" ");
    };
    Serial.println();
    Serial.print("TPV");
    Serial.print(" ");
    for (int i = 0; i < 20; i++){
        Serial.print(TPV[i]);
        Serial.print(" ");
    };
    Serial.println();
    Serial.print("ATT");
    Serial.print(" ");
    for (int i = 0; i < 17; i++){
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
        for (int i = 0; i < 20; i++){
           header = header + check_TPV(i);
        }
        header = "class," + header + "class,";
        for (int i = 0; i < 17; i++){
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
          dataFile.print(UTC_Time);
          dataFile.print(",");
      };
      if(TPV[3])
          dataFloat(gps.venus838data_raw.Latitude, 1);
      if(TPV[4])
          dataFloat(gps.venus838data_raw.Longitude, 1);
      if(TPV[5])
          dataFloat(gps.venus838data_raw.SealevelAltitude, 2);
      if(TPV[6])
          dataFloat(gps.venus838data_filter.Latitude, 1);
      if(TPV[7])
          dataFloat(gps.venus838data_filter.Longitude, 1);
      if(TPV[8])
          dataFloat(gps.venus838data_filter.SealevelAltitude, 2);
      if(TPV[9])
          dataFloat(course_angle, 0);
      if(TPV[10])
          dataFloat(gps.venus838data_raw.velocity, 0);
      if(TPV[11])
          dataFloat(gps.venus838data_filter.velocity, 0);
      if(TPV[12])
          dataFloat(gps.venus838data_raw.gdop, 0);
      if(TPV[13])
          dataFloat(gps.venus838data_raw.pdop, 0);
      if(TPV[14])
          dataFloat(gps.venus838data_raw.hdop, 0);
      if(TPV[15])
          dataFloat(gps.venus838data_raw.vdop, 0);      
      if(TPV[16])
          dataFloat(gps.venus838data_raw.tdop, 0);
}

void LogATT(){
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

float get_declination(float lat, float lon)
{
    int16_t decSW, decSE, decNW, decNE, lonmin, latmin;
    float decmin, decmax;
    uint8_t latmin_index, lonmin_index;

    latmin = floor(lat/5)*5;
    lonmin = floor(lon/5)*5;

    latmin_index= (90+latmin)/5;
    lonmin_index= (180+lonmin)/5;

    decSW = dec_tbl[latmin_index][lonmin_index];
    decSE = dec_tbl[latmin_index][lonmin_index+1];
    decNE = dec_tbl[latmin_index+1][lonmin_index+1];
    decNW = dec_tbl[latmin_index+1][lonmin_index];

    decmin = (lon - lonmin) / 5 * (decSE - decSW) + decSW;
    decmax = (lon - lonmin) / 5 * (decNE - decNW) + decNW;
    return (lat - latmin) / 5 * (decmax - decmin) + decmin;
}

