#include "GPSSerialMessageCom.h"

/*print softwareversion via pc terminal*/
void printSoftversion(GPSSoftVersiondata* versioninfo){
  String Printstring;
  Printstring="Softwareversion:";
  if(versioninfo->type)
      Printstring+="System code\r\n";
  else
      Printstring+="Reserved code\r\n";
  
      Printstring+="Kernel Version:";
      Printstring+=(versioninfo->Kversion>>16);
      Printstring+=".";
      Printstring+=((versioninfo->Kversion<<16)>>24);
      Printstring+=".";
      Printstring+=((versioninfo->Kversion<<24)>>24);
      Printstring+="\r\n";
      
      Printstring+="ODM Version:";
      Printstring+=(versioninfo->ODMversion>>16);
      Printstring+=".";
      Printstring+=((versioninfo->ODMversion<<16)>>24);
      Printstring+=".";
      Printstring+=((versioninfo->ODMversion<<24)>>24);
      Printstring+="\r\n";
      
      Printstring+="Revision:";
      Printstring+=(versioninfo->revision>>16);
      Printstring+=".";
      Printstring+=((versioninfo->revision<<16)>>24);
      Printstring+=".";
      Printstring+=((versioninfo->revision<<24)>>24);
      Printstring+="\r\n";
      Serial.println(Printstring);    
}

/*Print GPS data with some format*/
void printpositionfloatformat(float a, int decimalscale, const char * mark, const char* unit){
        int d = (int)(a*(float)decimalscale);
        int b=(d/decimalscale);
        int c=d%decimalscale;
        //Serial.print();
        Serial.print(mark); 
        Serial.print(b); 
        Serial.print(".");
        Serial.print(c);  
        Serial.print(unit);
       // Serial.print();
}

//Receive binary message via UART2 from GPS receiver
//buf: receive buf
//messageid: id of message that is requred to receive. if it is 0, ignore
//return: byte size of data  received 
int waitingRespondandReceive(byte *buf, byte messageid, unsigned int timeout)
{
  unsigned long start = millis();
  int payloadlen=0;
  int pos=0;
  byte c;
const byte BinaryMsgHeader[]={0xA0,0xA1};
  
  while (millis() - start < timeout) {
        if(pos==0)
            memset(buf,0,128);
            
        while(!Serial2.available()){
            delay(10);
        }
        c=Serial2.read();
        if(pos<2){
          if(c==BinaryMsgHeader[pos]){
              buf[pos++]=c;
          }
          else{
              pos=0;
          }
        }
        else if((pos>1)&&(pos<4)){
          buf[pos++]=c;
          if(pos==4)
              payloadlen=(buf[2]<<8)|buf[3];
        }
        else if(pos==4){
              buf[pos++]=c;
               if(messageid)
                   if(buf[4]!=messageid)
                       pos=0;
        }
         else{
            if(pos<payloadlen+7){
              buf[pos++]=c;
              if((pos==payloadlen+7)||((buf[pos-2]==0x0D)&&(buf[pos-1]==0x0A)))
                    return pos;
            }
             
        }
   }          
    return pos;  
}

int BinaryRecvMsgtype(const char *MsgInfo,byte* receivebuf){
	if((receivebuf[0]!=0xA0)|(receivebuf[1]!=0xA1)){
		MsgInfo="BinaryMessage unlecognized\n";
		return -1;
	}
	
	int recvlen=(receivebuf[2]<<8)|(receivebuf[3]);
	if((recvlen<0)|!recvlen){
		MsgInfo="BinaryMessage receive failed\n";
		return -1;
	}
	if((receivebuf[4]==0x83)&&(receivebuf[5]==0x02)){
		MsgInfo="BinaryAck received\n";
		return 2;
	}
	else if((receivebuf[4]==0x84)&&(receivebuf[5]==0x01)){
		MsgInfo="BinaryNAck received\n";
		return 1;
	}else if(receivebuf[4]==0xA8){
		MsgInfo="Navigation Binary message\n";
		return 59;
	}
	else{
		MsgInfo="Response message\n";
		return 3;
	}
	
}		
//Send configure binary message to Receiver and receive ACK or NACK
//streamsize : send stream size
//sendbuf    : byte array to send to receiver.
//messageid  : respond type
//timeout    :response waiting time in milisecond
//return      :respondmessage size
int SendBinaryMessagetoGPSreceiver(int streamsize, byte* sendbuf, byte* receivebuf, byte messageid, int timeout){
  char messagetype[64];
  memset(messagetype,0,64);
  int ret=0;
    if(streamsize>7){
      for(int cnt=0;cnt<streamsize;cnt++)
          Serial2.write(sendbuf[cnt]);
      if(waitingRespondandReceive(receivebuf,messageid,timeout)>7){
        ret = BinaryRecvMsgtype(messagetype,receivebuf);
              Serial.println(messagetype); 
        }
       return ret; 
  }
  else
    Serial.println("Making binary message was failed.\n");
  return ret;
}

/*Receive GPS Navigation binary messat and input into parameter1 and after kalman filtering,
    Insert filtered data into parmeter2 
*/
int GPSNavigationMsgProcessing(NavGPSdata *filterbeforedata, NavGPSdata *filterafterdata,BMsg838 gps,KalmanFilter *filter,KalmanFilterVA *filterVA){
      int ret;
      //GPS interface variabes
      uint8_t modenum[2];
      int32_t array_position[2];
      uint32_t array_altitude[2];
      uint16_t array_dilution[5];
      int32_t array_coordinate[3];
      int32_t array_veolcity[3];
      int64_t long_velocity;
      int64_t *filterdata;
      
      //Receive Navigate binary message and scale with data format 
      //and input into Parameters "
      ret=gps.ReceiveNavigationData(modenum,array_position, array_altitude, array_dilution, array_coordinate,array_veolcity);
      if(ret==1){
       	  
      		  filterbeforedata->fixmode=modenum[0];
            filterbeforedata->NumSV=modenum[1];
            filterbeforedata->Latitude=(float)array_position[0]/10000000.0;
            filterbeforedata->Longitude=(float)array_position[1]/10000000.0;
            filterbeforedata->SealevelAltitude=(float)array_altitude[1]/100.0;
              
            filterbeforedata->GDOP=(float)array_dilution[0]/100.0;
      		  filterbeforedata->PDOP=(float)array_dilution[1]/100.0;
            filterbeforedata->HDOP=(float)array_dilution[2]/100.0;
            filterbeforedata->VDOP=(float)array_dilution[3]/100.0;
            filterbeforedata->TDOP=(float)array_dilution[4]/100.0;

            if ((array_position[0]!=0)&&(array_position[1]!=0))  {
            filterdata=filter->KalmanProcessing((int64_t)array_position[0], (int64_t)array_position[1]);
            } else if (filter->i == 100) {filterdata=filter->KalmanNoData();}
            filterafterdata->Latitude=(float)filterdata[0]/10000000.0;
            filterafterdata->Longitude=(float)filterdata[1]/10000000.0;
            filterafterdata->receivedtime=(float)filterdata[2];
            long_velocity=0;
            for(int k=0;k<3;k++)
                long_velocity+=(array_veolcity[k]*array_veolcity[k]);
            long_velocity=sqrt(long_velocity);
            filterbeforedata->velocity=(float)long_velocity/100.0;
            filterdata = filterVA->KalmanProcessing(array_altitude[1], long_velocity);
            filterafterdata->SealevelAltitude=(float)filterdata[0]/100.0;
            filterafterdata->velocity=(float)filterdata[1]/100.0;
            //filterafterdata->velocity=(float)filterdata[1]/100.0;
            return 1;        
            
        }
        else return ret;
}
