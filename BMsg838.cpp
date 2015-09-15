
#include "BMsg838.h"

#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"
#define  ACK		0x02
#define  NACK		0x01
fix_cb_t fix_callback = NULL; // TJS:

void BMsg838::add_callback(fix_cb_t fct_ptr){
  fix_callback = fct_ptr;
}

BMsg838::BMsg838()
:  _time(GPS_INVALID_TIME)
,  _date(GPS_INVALID_DATE)
,  _latitude(GPS_INVALID_ANGLE)
,  _longitude(GPS_INVALID_ANGLE)
,  _altitude(GPS_INVALID_ALTITUDE)
,  _speed(GPS_INVALID_SPEED)
,  _course(GPS_INVALID_ANGLE)
,  _last_time_fix(GPS_INVALID_FIX_TIME)
,  _last_position_fix(GPS_INVALID_FIX_TIME)
,  _parity(0)
,  _is_checksum_term(false)
,  _sentence_type(_GPS_SENTENCE_OTHER)
,  _term_number(0)
,  _term_offset(0)
,  _gps_data_good(false)
#ifndef _GPS_NO_STATS
,  _encoded_characters(0)
,  _good_sentences(0)
,  _failed_checksum(0)
#endif
{
  _term[0] = '\0';
 
}
BMsg838::~BMsg838()
{
	 
}
//
// public methods
//
/*---------------------------------------------------------------------*/
/*================= Binary message checksum function==================*/
/*						CS = 0, N=PL;
						For n = 0 to N
					CS = CS ^ <Payload Byte # n>*/	
/*---------------------------------------------------------------------*/
uint8_t BMsg838::checksum(byte* buffer, int len) 
{
    uint8_t cs = 0;
    while (len) {
      cs ^= *buffer;
      buffer++;
      len--;
    }
    return cs;
}
//get message type among Ack, response, NAck an error


/*Make binary message of venus 838LPx GPS receiver
	The sytax of the Message is shown below.
	<0xA0,0xA1><PL><Message ID><Message Body><CS><0x0D,0x0A>
	Payload : Binary Message String
	id :      message id
	len:      Message Payload length
	subflag:  message type
	
	return total length of Binary message made
*/
uint8_t BMsg838::MakeBinaryMessage(int len){
	char checkingsum=0;
	char temp[128];
	memcpy(temp,SendStream,128);
	checkingsum=checksum(SendStream, len);
	
	SendStream[0]=0xA0;
	SendStream[1]=0xA1;
	SendStream[2]=(len>>8)&0xFF;
	SendStream[3]=len;
	for(int j=0;j<len;j++){
		SendStream[j+4]=temp[j];	
	}
	
	SendStream[len+4]=checkingsum;
	SendStream[len+5]=0x0D;
	SendStream[len+6]=0x0A;
	return len+7;
}

/*This is a request which will reset and restart the GNSS receiver. */
uint8_t BMsg838::ResetGNSS(uint8_t startmode, uint16_t year, uint8_t month, uint8_t day,uint8_t hour, uint8_t minute, uint8_t second, int16_t latitude, int16_t longitude, int16_t altitude)
{
	memset(SendStream,0,128);
	SendStream[0]=1;
	SendStream[1]=startmode;
	SendStream[2]=year>>8; 
	SendStream[3]=year;
	SendStream[4]=month;
	SendStream[5]=day; 
	SendStream[6]=hour;
	SendStream[7]=minute;
	SendStream[8]=second;
	SendStream[9]=(latitude*100)>>8;
	SendStream[10]=latitude*100;
	SendStream[11]=(longitude*100)>>8;
	SendStream[12]=longitude*100;
	SendStream[13]=altitude>>8;
	SendStream[14]=altitude; 
	return MakeBinaryMessage(15);
}
/*Make Stream to request message  of Softversion*/
uint8_t BMsg838::GetSoftVersion(){
	
	memset(SendStream,0,128);
	SendStream[0]=0x02;
	SendStream[1]=0x00;
	return MakeBinaryMessage(2);
}

/*Get version information of GPS receiver
	
	return : 1->get success, 0-> checksum error, -1->timeout, 3->
*/
GPSSoftVersiondata* BMsg838::ResponseSoftVersion()
{
	//return SendMessage(SendStream, 1000);
	if(RecVBinarybuf[4]==0x80){
		softversion.type=RecVBinarybuf[5];
		softversion.Kversion=(RecVBinarybuf[6]<<24)|(RecVBinarybuf[7]<<16)|(RecVBinarybuf[8]<<8)|RecVBinarybuf[9];
		softversion.ODMversion=(RecVBinarybuf[10]<<24)|(RecVBinarybuf[11]<<16)|(RecVBinarybuf[12]<<8)|RecVBinarybuf[13];
		softversion.revision=(RecVBinarybuf[14]<<24)|(RecVBinarybuf[15]<<16)|(RecVBinarybuf[16]<<8)|RecVBinarybuf[17];
		softversion.ckecksumOK =(checksum(RecVBinarybuf+4, 14)==RecVBinarybuf[18]);
		return &softversion;
		
	}
	else 
		return NULL;
	
}

/*Get SoftwareCRC information of GPS receiver
	return binaysteam size in byte unit.	
*/
uint8_t BMsg838::GetSoftCRC()
{
	memset(SendStream,0,128);
	SendStream[0]=0x03;
	SendStream[1]=0x00;
	return MakeBinaryMessage(2);
}
/*Response message process of SoftwareCRC information of GPS receiver
	type: Software type: 0?Reserved:(1)System code	
	CRC:CRC value
	timeout: ack waiting time
	return : 1->get success, 0-> checksum error, -1->timeout, 3->NACK
	
	Two Binary messages are used in this function 
	of which id are is 0x03,x81
*/
uint8_t BMsg838::ResponseSoftCRC(uint8_t*Type, uint16_t*CRCinfo){
	//SendMessage(SendStream, 1000);
	
	if(RecVBinarybuf[4]==0x81){
		*Type=RecVBinarybuf[5];
		*CRCinfo=(RecVBinarybuf[6]<<8)|RecVBinarybuf[7];
					
		return (checksum(RecVBinarybuf+4, 4)==RecVBinarybuf[8]);
		
	}
	else 
		return -1;

}
/*Set Factory Default
	timeout : response wating time
	return  : ack(0x02) or 0x03(Nack) or -1(timeput) , 0(checksum error)
*/

uint8_t BMsg838::SetFactoryDefalt(){
	memset(SendStream,0,128);
	SendStream[0]=0x04;
	SendStream[1]=0x00;
	return MakeBinaryMessage(2);
}
/*=================================================================/
	Set SerialPort Baudrate
	baudrate: com1 baudrae of GPS to set
	atribute: pdatetype: 0(RAM) or 1(both fo ram and flash)
	timeout : response wating time
	return  : ack(0x02) or 0x03(Nack) or -1(timeput) , 0(checksum error)
/====================================================================*/

uint8_t BMsg838::SetSerialPort(int Baudrate, uint8_t Atribute){
	memset(SendStream,0,128);
	int baudR[9]={4800,9600,19200,38400,57600,115200,230400,460800,921600};
	SendStream[0]=0x05;
	SendStream[1]=0x00;
	for(int i=0;i<9;i++)
		if(baudR[i]==Baudrate){
			SendStream[2]=i;
			break;
		}				
	SendStream[3]=Atribute;						
	return MakeBinaryMessage(4);
}


/*=================================================================/
	Set BinaryMessagetype
	
	timeout : response wating time
	return  : ack(0x02) or 0x03(Nack) or -1(timeput) , 0(checksum error)
/=================================================================*/
uint8_t BMsg838::SetBinaryMessagetype(){
	SendStream[0]=0x09;
	SendStream[1]=0x02;				//Message type : Binary message Mode
	SendStream[2]=0x01;				//atribute: pdatetype: 0(RAM) or 1(both fo ram and flash)		
	return MakeBinaryMessage(3);
	
}
/*Set powermode binary message steeam make
		
*/
uint8_t BMsg838::SetPowerMode(uint8_t mode, uint8_t Atribute)
{
	memset(SendStream,0,128);
	SendStream[0]=12;
	SendStream[1]=mode;
	SendStream[2]=Atribute;						
	return MakeBinaryMessage(3);
}
/*making binary message stream setting of position update rate */
uint8_t BMsg838::SetPositionRate(uint8_t Rate)
{
	memset(SendStream,0,128);
	SendStream[0]=0x0E;
	SendStream[1]=Rate;			//update rate
	SendStream[2]=1;			//0:Ram 1:both of RAM amd Flush						
	return MakeBinaryMessage(3);
}
/*Get Position update rate
	
	return : stream length
*/
uint8_t BMsg838::GetPositionRate()
{
	
	memset(SendStream,0,128);
	SendStream[0]=0x10;
	return MakeBinaryMessage(1);
}
/*Response Message process to query of Position update rate
	rate: positon update rate is one of {1,2,4,5,8,10,20,25,40,50}
	timout wating time of ACK
	return : 1->get success, 0-> checksum error, -1->timeout, 3->nack
*/	
uint8_t BMsg838::ResponsePositionRate(uint8_t* rate)
{	
	if(RecVBinarybuf[4]==0x86){
		*rate=RecVBinarybuf[5];
						
		return (checksum(RecVBinarybuf+4, 2)==RecVBinarybuf[6]);
	}
	else 
		return -1;
	
}


/*=================================================================/
	Set Navigation Interval
	interval: value of Navigation Interval of GPS to set
	return  : ack(0x02) or 0x03(Nack) or -1(timeput) , 0(checksum error)
/====================================================================*/
uint8_t BMsg838::SetNavigationInterval(uint8_t interval)  //interval[7]
{
	memset(SendStream,0,128);
	SendStream[0]=17;
	SendStream[1]=interval;
	SendStream[2]=1;							
	return MakeBinaryMessage(3);
	
}


/*=================================================================/
	Set Position Datum
	Set parameter:
			uint16_t index, 
			uint8_t EllipIdx, 
			uint16_t DeltaX, 
			uint16_t DeltaY, 
			uint16_t DeltaZ,
			uint32_t Semiaxis, 
			uint8_t InversedFlatten
		
	return  : ack(0x02) or 0x03(Nack) or -1(timeput) , 0(checksum error)
/====================================================================*/
uint8_t BMsg838::SetPositionDatum(uint16_t index, uint8_t EllipIdx, uint16_t DeltaX, uint16_t DeltaY, uint16_t DeltaZ,uint32_t Semiaxis, uint8_t InversedFlatten)
{
	memset(SendStream,0,128);
	SendStream[0]=0x29;
	SendStream[1]=(index>>8)&0xFF;
	SendStream[2]=index&0xFF;
	SendStream[3]=EllipIdx;
	SendStream[4]=(DeltaX>>8)&0xFF;
	SendStream[5]=DeltaX&0xFF;
	SendStream[6]=(DeltaY>>8)&0xFF;
	SendStream[7]=(DeltaY)&0xFF;
	SendStream[8]=(DeltaZ>>8)&0xFF;
	SendStream[9]=(DeltaZ)&0xFF;
	SendStream[10]=(Semiaxis>>24)&0xFF;
	SendStream[11]=(Semiaxis>>16)&0xFF;
	SendStream[12]=(Semiaxis>>8)&0xFF;
	SendStream[13]=(Semiaxis)&0xFF;
	SendStream[14]=(InversedFlatten>>24)&0xFF;
	SendStream[15]=(InversedFlatten>>16)&0xFF;
	SendStream[16]=(InversedFlatten>>8)&0xFF;
	SendStream[17]=InversedFlatten&0xFF;
	SendStream[18]=1;							
	return MakeBinaryMessage(19);
	
}

	
/*return : 1->get success, 0-> checksum error, -1->timeout, 3->*/
uint8_t BMsg838::Getdatum(){
	
	memset(SendStream,0,128);
	SendStream[0]=0x2D;
	return MakeBinaryMessage(1);
	//SendMessage(SendStream, 1000);
}
/*return : 1->get success, 0-> checksum error, -1->timeout, 3->*/
uint8_t BMsg838::Responsedatum(uint16_t* Datum){	
	if(RecVBinarybuf[4]==0xAE){
		
		*Datum=(RecVBinarybuf[5]<<8)|RecVBinarybuf[6];
					
		return (checksum(RecVBinarybuf+4, 3)==RecVBinarybuf[7]);
	}
	else 
		return -1;
	
}



/*Enable or disable position pinning of
GNSS receiver
return : binary message length*/
uint8_t BMsg838::SetPositionPinning(uint8_t positionpinning)
{
	memset(SendStream,0,128);
	SendStream[0]=0x39;
	SendStream[1]=positionpinning;
	SendStream[2]=1;
	return MakeBinaryMessage(3);
	
}
/*Query position pinning status of the
GNSS receiver
return : binary message length*/
uint8_t BMsg838::GetPositionPinning(){
	
	memset(SendStream,0,128);
	SendStream[0]=0x3A;
	return MakeBinaryMessage(1);
}
/*return : 1->get success, 0-> checksum error, -1->timeout, 3->*/

uint8_t BMsg838::ResponsePositionPinning(uint8_t*status, uint16_t* speed, uint16_t* cnt,uint16_t* upspeed,uint16_t* upcnt,uint16_t* updsit){
	//return SendMessage(SendStream, 1000);
	if(RecVBinarybuf[4]==0xB4){
		
		*status=RecVBinarybuf[5];
		*speed=(RecVBinarybuf[6]<<8)|RecVBinarybuf[7];
		*cnt=(RecVBinarybuf[8]<<8)|RecVBinarybuf[9];
		*upspeed=(RecVBinarybuf[10]<<8)|RecVBinarybuf[11];
		*upcnt=(RecVBinarybuf[12]<<8)|RecVBinarybuf[13];
		*updsit=(RecVBinarybuf[14]<<8)|RecVBinarybuf[15];
		return (checksum(RecVBinarybuf+4, 12)==RecVBinarybuf[16]);;
	}
	else 
		return -1;
}
/*Set position pinning parameters of GNSS
receiver
return: binary message strem length*/
uint8_t BMsg838::SetPositionPinningParam(uint16_t speed,uint16_t cnt,uint16_t upspeed,uint16_t upcnt,uint16_t updistance){
	memset(SendStream,0,128);
	SendStream[0]=0x3B;
	SendStream[1]=(speed>>8)&0xFF;
	SendStream[2]=(speed)&0xFF;
	SendStream[3]=(cnt>>8)&0xFF;
	SendStream[4]=(cnt)&0xFF;
	SendStream[5]=(upspeed>>8)&0xFF;
	SendStream[6]=(upspeed)&0xFF;
	SendStream[7]=(upcnt>>8)&0xFF;
	SendStream[8]=(upcnt)&0xFF;
	SendStream[9]=(updistance>>8)&0xFF;
	SendStream[10]=(updistance)&0xFF;
	SendStream[11]=1;
	return MakeBinaryMessage(12);
	
}

/*Query 1PPS timing of the GNSS receiver
	return : binary message stream length in byte unit*/
uint8_t BMsg838::Get1PPSTiming( ){
	char SendStream[128];
	memset(SendStream,0,128);
	SendStream[0]=0x44;
	return MakeBinaryMessage(1);
	
}
/*	processing Response message to Query 1PPS timing of the GNSS receiver
	return : 1->get success, 0-> checksum error, -1->timeout, NACK->*/

uint8_t BMsg838::Response1PPSTiming(uint8_t* mode,uint32_t* len,uint32_t* standev,DPFP* savelati,DPFP* savelong,SPFP* savealti,uint8_t* runtimemode,uint32_t* runtimelen){
	
	if(RecVBinarybuf[4]==0xC2){
		
		*mode=RecVBinarybuf[5];
		*len=(RecVBinarybuf[6]<<24)|(RecVBinarybuf[7]<<16)|(RecVBinarybuf[8]<<8)|RecVBinarybuf[9];
		*standev=(RecVBinarybuf[10]<<24)|(RecVBinarybuf[11]<<16)|(RecVBinarybuf[12]<<8)|RecVBinarybuf[13];
		*savelati=(RecVBinarybuf[14]<<56)|(RecVBinarybuf[15]<<48)|(RecVBinarybuf[16]<<40)|(RecVBinarybuf[17]<<32)|(RecVBinarybuf[18]<<24)|(RecVBinarybuf[19]<<16)|(RecVBinarybuf[20]<<8)|RecVBinarybuf[21];
		*savelong=(RecVBinarybuf[22]<<56)|(RecVBinarybuf[23]<<48)|(RecVBinarybuf[24]<<40)|(RecVBinarybuf[25]<<32)|(RecVBinarybuf[25]<<26)|(RecVBinarybuf[27]<<16)|(RecVBinarybuf[28]<<8)|RecVBinarybuf[29];
		*savealti=(RecVBinarybuf[30]<<24)|(RecVBinarybuf[31]<<16)|(RecVBinarybuf[32]<<8)|RecVBinarybuf[33];
		*runtimemode=RecVBinarybuf[34];
		*runtimelen=(RecVBinarybuf[35]<<24)|(RecVBinarybuf[36]<<16)|(RecVBinarybuf[37]<<8)|RecVBinarybuf[38];
		return (checksum(RecVBinarybuf+4, 35)==RecVBinarybuf[39]);
	}
	else 
		return -1;
	
}
/*Configure cable delay of 1PPS timing*/
uint8_t BMsg838::Set1PPSCabledelay(uint32_t Cabledelay){

	char SendStream[128];
	memset(SendStream,0,128);
	SendStream[0]=0x45;
	SendStream[1]=Cabledelay>>24;
	SendStream[2]=Cabledelay>>16;
	SendStream[3]=Cabledelay>>8;
	SendStream[4]=Cabledelay;
	SendStream[5]=1;
	return MakeBinaryMessage(6);
	
}
/*Cable delay of 1PPS timing mode . Get value in unit of 1/100 ns. edelayThe 
	value is stored in Cabledelay [-500000,500000] 
	return : 1->get success, 0-> checksum error, -1->timeout, NACK->
	*/
	
uint8_t BMsg838::Get1PPSCabledelay(){

	memset(SendStream,0,128);
	SendStream[0]=0x46;
	return MakeBinaryMessage(1);
	//SendMessage(SendStream, 1000);
}
uint8_t BMsg838::Response1PPSCabledelay(uint32_t *Cabledelay){
	
	if(RecVBinarybuf[4]==0xBB){
		
		*Cabledelay=(RecVBinarybuf[5]<<24)|(RecVBinarybuf[6]<<16)|(RecVBinarybuf[7]<<8)|RecVBinarybuf[8];
		return (checksum(RecVBinarybuf+4, 5)==RecVBinarybuf[9]);
	}
	else 
		return -1;

}
/*Configure the navigation mode of GNSS
receiver
return: Binary Message stream length*/
uint8_t BMsg838::SetGNSSNavigationMode(uint8_t mode){
	
	memset(SendStream,0,128);
	SendStream[0]=0x64;
	SendStream[1]=0x17;
	SendStream[2]=mode;
	SendStream[3]=1;
	return MakeBinaryMessage(4);
	
}
/*Query the navigation mode of GNSS
receiver
return: Binary Message stream length*/
int8_t BMsg838::GetGNSSNavigationMode(){
	
	memset(SendStream,0,128);
	SendStream[0]=0x64;
	SendStream[1]=0x18;
	return MakeBinaryMessage(2);
}
/*Processing response message to query the navigation mode
	return : 1->get success, 0-> checksum error, -1->timeout, NACK->*/

int8_t BMsg838::ResponseGNSSNavigationMode(char* mode){
	
	if(RecVBinarybuf[5]==0x8B){
		
		switch (RecVBinarybuf[6])
					{
						case		0:
							sprintf(mode, "%s","auto");//mode="auto";
							break;
						case		1:
							//mode="prdestrain";
							sprintf(mode, "%s","prdestrain");
							break;
						case		2:
							//mode="car";
							sprintf(mode, "%s","car");
							break;
						case		3:
							//mode="marine";
							sprintf(mode, "%s","marine");
							break;
						case		4:
							//mode="ballon";
							sprintf(mode, "%s","ballon");
							break;
						case		5:
							//mode="airborne";
							sprintf(mode, "%s","airborne");
							break;
						
					}
					
					return (checksum(RecVBinarybuf+4, 3)==RecVBinarybuf[7]);
	}
	else 
		return -1;
	
}
/*binary message stream  make for Configure the GNSS constellation type
used for navigation solution

return stream length*/
uint8_t BMsg838::SetGNSSConstelGPStype(){
	memset(SendStream,0,128);
	SendStream[0]=0x64;
	SendStream[1]=0x19;
	SendStream[2]=0;
	SendStream[3]=0;
	SendStream[4]=1;
	return MakeBinaryMessage(5);
	

}
/*make binary message stream to Query the GNSS constellation type used
for navigation solution
return : binay message stream size in byte*/
uint8_t BMsg838::GetGNSSConstellationtype(){
	
	memset(SendStream,0,128);
	SendStream[0]=0x64;
	SendStream[1]=0x1A;
	return MakeBinaryMessage(2);
}
/*Response message processing to Query the GNSS constellation type used
for navigation solution
return : 1->get success, 0-> checksum error, -1->timeout, NACK->*/

uint8_t BMsg838::ResponseGNSSConstellationtype(char *mode){
	//SendMessage(SendStream, 1000);
	char Navimode[4][10]={"GPS","Glonass","Galileo","Beido"};	
	if(RecVBinarybuf[5]==0x8C){
		
		if(RecVBinarybuf[7]&1){
				sprintf(mode, "%s",Navimode[0]);
				//mode="GPS";
				
			}if(RecVBinarybuf[7]&2){
				if(mode)
					strcat(mode,Navimode[1]);
				else
					sprintf(mode, "%s",Navimode[1]);
				
			}if(RecVBinarybuf[7]&4){
				if(mode)
					strcat(mode,Navimode[2]);
				else
					sprintf(mode, "%s",Navimode[2]);
			}if(RecVBinarybuf[7]&8){
				if(mode)
					strcat(mode,Navimode[3]);
				else
					sprintf(mode, "%s",Navimode[3]);
			}	
			return (checksum(RecVBinarybuf+4, 4)==RecVBinarybuf[8]);
	}
	else 
		return -1;
	
}
/*Make binary message stream for Configure GPS/UTC leap seconds of
GNSS receiver
return binary message stream size*/
uint8_t BMsg838::SetGPSUTCSecond(uint8_t leapsecond)
{
	memset(SendStream,0,128);
	SendStream[0]=0x64;
	SendStream[1]=0x1F;
	SendStream[2]=leapsecond;
	SendStream[3]=1;
	return MakeBinaryMessage(4);
}
/*Make binary message stream for Query GPS time of GNSS receiver
return binary message stream size*/
uint8_t BMsg838::GetGPSTime(){
	
	memset(SendStream,0,128);
	SendStream[0]=0x64;
	SendStream[1]=0x20;
	return  MakeBinaryMessage(2);
	//SendMessage(SendStream, timeout);
}
/*response message processing to Query GPS time of GNSS receiver
return message length*/
uint8_t BMsg838::respondGPSTime(uint32_t *Timeofweek, uint32_t *SubTimeofweek, uint16_t *weeknumber, uint8_t *Defleapsecond,int8_t *curleapsecond, uint8_t *Valid){
	
	if(RecVBinarybuf[5]==0x8E){
		
			*Timeofweek=(RecVBinarybuf[6]<<24)|(RecVBinarybuf[7]<<16)|(RecVBinarybuf[8]<<8)|RecVBinarybuf[9];
			*SubTimeofweek=(RecVBinarybuf[10]<<24)|(RecVBinarybuf[11]<<16)|(RecVBinarybuf[12]<<8)|RecVBinarybuf[13];
			*weeknumber=(RecVBinarybuf[14]<<8)|RecVBinarybuf[15];
			*Defleapsecond=RecVBinarybuf[16];
			*curleapsecond=RecVBinarybuf[17];
			*Valid=RecVBinarybuf[18];
			
			return (checksum(RecVBinarybuf+4, 15)==RecVBinarybuf[19]);
		
	}
	else 
		return -1;
	
}
/*Make binary message stream for Configure GNSS datum index of GNSS
receiver
return binary message stream size*/
uint8_t BMsg838::SetDatumIndex(uint8_t index){
	memset(SendStream,0,128);
	SendStream[0]=0x64;
	SendStream[1]=0x27;
	SendStream[2]=(index>>8)&0xFF;
	SendStream[3]=index&0xFF;
	SendStream[4]=1;
	return MakeBinaryMessage(5);
	
}
/*Make binary message stream for Query GNSS datum index of GNSS
receiver
return binary message stream size*/
uint8_t BMsg838::GetDatumIndex(){
	
	memset(SendStream,0,128);
	SendStream[0]=0x64;
	SendStream[1]=0x28;
	return MakeBinaryMessage(2);
}	//SendMessage(SendStream, 1000);
/*response message processing to Configure GNSS datum index of GNSS
receiver
return 1:receive ok 0:checksum error  -1: Invalid message*/
uint8_t BMsg838::ResponseDatumIndex(uint16_t *datumindex)
{
	
	if(RecVBinarybuf[5]==0x92){
		
			*datumindex=(RecVBinarybuf[6]<<8)|RecVBinarybuf[7];
			return (checksum(RecVBinarybuf+4, 4)==RecVBinarybuf[8]);
	}
	else 
		return -1;

}
/*Make binary message stream for Configure 1PPS pulse width of GNSS
receiver
return binary message stream size*/
uint8_t BMsg838::Set1PPSPulseWidth(uint32_t width ){
	memset(SendStream,0,128);
	SendStream[0]=0x65;
	SendStream[1]=0x01;
	SendStream[2]=(width>>24)&0xFF;
	SendStream[3]=(width>>16)&0xFF;
	SendStream[4]=(width>>8)&0xFF;
	SendStream[5]=(width)&0xFF;
	SendStream[6]=1;
	return MakeBinaryMessage(7);
	
}
/*Make binary message stream for Query 1PPS pulse width of GNSS
receiver
return binary message stream size*/

uint8_t BMsg838::Get1PPSPulseWidth(){
	
	memset(SendStream,0,128);
	SendStream[128]=0x65;
	SendStream[128]=0x02;
	return MakeBinaryMessage(2);
}
/*
	response message processing to Query 1PPS pulse width of GNSS
receiver
return : 1->get success, 0-> checksum error, -1->timeout, NACK->*/

uint8_t BMsg838::Response1PPSPulseWidth(uint32_t* Width){
	
	if(RecVBinarybuf[5]==0x80){
		
			*Width=(RecVBinarybuf[6]<<24)|(RecVBinarybuf[7]<<16)|(SendStream[8]<<8)|(SendStream[9]<<0);
			return (checksum(RecVBinarybuf+4, 6)==RecVBinarybuf[10]);
			
	}
	else 
		return -1;

}

/*Make binary message stream for Configure 1PPS frequency output of
GNSS receiver
return binary message stream size*/
uint8_t BMsg838::Set1PPSFrequency(uint32_t frequency){
	
	memset(SendStream,0,128);
	SendStream[0]=0x65;
	SendStream[1]=3;
	SendStream[2]=(frequency>>24)&0xFF;
	SendStream[3]=(frequency>>16)&0xFF;
	SendStream[4]=(frequency>>8)&0xFF;
	SendStream[5]=(frequency)&0xFF;
	SendStream[6]=1;
	return MakeBinaryMessage(7);
	
	
}

/*Make binary message stream for query 1PPS frequency output of
GNSS receiver
return binary message stream size*/
uint8_t BMsg838::Get1PPSFrequency()
{
	
	memset(SendStream,0,128);
	SendStream[0]=0x65;
	SendStream[0]=0x04;
	return MakeBinaryMessage(2);
}
/*
	response message processing to query 1PPS frequency output of
GNSS receiver
return : 1->get success, 0-> checksum error, -1->timeout, NACK->*/

uint8_t BMsg838::Response1PPSFrequency(uint32_t *frequency)
{	
	if(RecVBinarybuf[5]==0x81){
		
		*frequency=(RecVBinarybuf[6]<<24)|(RecVBinarybuf[7]<<16)|(RecVBinarybuf[8]<<8)|(RecVBinarybuf[9]<<0);
		return (checksum(RecVBinarybuf+4, 6)==RecVBinarybuf[10]);
			
	}
	else 
		return -1;
	
}
/*This is a function which receive Message of user navigation date in binary format
	(ID:0xA8)from the GNSS receiver .
	Paramer:
		position: scale 1e-7 , degree   [0]-Latitude; >0 :North Hemisphere
										 			<0:South Hemisphere
							   			[1]-Longitude;>0 :North Hemisphere
													<0:South Hemisphere
		altitude: scale 0.01 , meter   	[0]-ellipsoid altitu;
							            [1]-mean sea level altitude			
		dilution: scale 0.01 , 			[0]-Geometric dilution of precision
							   			[1]-Position dilution of precision
							   			[2]-Horizontal dilution of precision
							   			[3]-Vertical dilution of precision
							   			[4]-Time dilution of precision			
		coordinate: scale 0.01 , meter  [0]-ECEF X coordinate
							            [1]-ECEF Y coordinate
							            [2]-ECEF Z coordinate	
		altitude: scale 0.01 , meter/s 	[0]-ECEF X Veolcity
							           	[1]-ECEF Y Veolcity	
							           	[2]-ECEF Z Veolcity	
	return : 1->receive success, 0-> checksum error, -1->timeout, 3->NACK
												
*/
uint8_t BMsg838::ReceiveNavigationData(uint8_t* modenum,int32_t *position, uint32_t* altitude, uint16_t *dilution, int32_t* coordinate,int32_t* veolcity)
{
	
	if(RecVBinarybuf[4]==0xA8){
		modenum[0]=RecVBinarybuf[5];
		modenum[1]=RecVBinarybuf[6];
		position[0]=(RecVBinarybuf[13]<<24)|(RecVBinarybuf[14]<<16)|(RecVBinarybuf[15]<<8)|(RecVBinarybuf[16]<<0);
		position[1]=(RecVBinarybuf[17]<<24)|(RecVBinarybuf[18]<<16)|(RecVBinarybuf[19]<<8)|(RecVBinarybuf[20]<<0);
		altitude[0]=(RecVBinarybuf[21]<<24)|(RecVBinarybuf[22]<<16)|(RecVBinarybuf[23]<<8)|(RecVBinarybuf[24]<<0);
		altitude[1]=(RecVBinarybuf[25]<<24)|(RecVBinarybuf[26]<<16)|(RecVBinarybuf[27]<<8)|(RecVBinarybuf[28]<<0);
		dilution[0]=(RecVBinarybuf[29]<<8)|(RecVBinarybuf[30]<<0);
		dilution[1]=(RecVBinarybuf[31]<<8)|(RecVBinarybuf[32]<<0);
		dilution[2]=(RecVBinarybuf[33]<<8)|(RecVBinarybuf[34]<<0);
		dilution[3]=(RecVBinarybuf[35]<<8)|(RecVBinarybuf[36]<<0);
		dilution[4]=(RecVBinarybuf[37]<<8)|(RecVBinarybuf[38]<<0);
		coordinate[0]=(RecVBinarybuf[39]<<24)|(RecVBinarybuf[40]<<16)|(RecVBinarybuf[41]<<8)|(RecVBinarybuf[42]<<0);
		coordinate[1]=(RecVBinarybuf[43]<<24)|(RecVBinarybuf[44]<<16)|(RecVBinarybuf[45]<<8)|(RecVBinarybuf[46]<<0);
		coordinate[2]=(RecVBinarybuf[47]<<24)|(RecVBinarybuf[48]<<16)|(RecVBinarybuf[49]<<8)|(RecVBinarybuf[50]<<0);
		veolcity[0]=(RecVBinarybuf[51]<<24)|(RecVBinarybuf[52]<<16)|(RecVBinarybuf[53]<<8)|(RecVBinarybuf[54]<<0);
		veolcity[1]=(RecVBinarybuf[55]<<24)|(RecVBinarybuf[56]<<16)|(RecVBinarybuf[57]<<8)|(RecVBinarybuf[58]<<0);
		veolcity[2]=(RecVBinarybuf[59]<<24)|(RecVBinarybuf[60]<<16)|(RecVBinarybuf[61]<<8)|(RecVBinarybuf[62]<<0);
		return (checksum(RecVBinarybuf+4, 59)==RecVBinarybuf[63]);
	}
	else 
		return -1;
}

/*======================Binary message to make ============*/
//uint8_t BMsg838::SetNMEATalkId(uint8_t idtype, uint8_t Attribute);
//uint8_t BMsg838::GetNMEATalkId();
//93			char*	respondNMEAtalkID(uint8_t id);

//uint8_t BMsg838::Set1PPSTiming(uint8_t timemode,uint32_t len,uint32_t standdeciation,DFDP Latitude,DFDP Longtude,DFDP Altitude,uint8_t attribute);
//uint8_t BMsg838::SetSBASParam(uint8_t Enable,uint8_t ranging,uint8_t URAmask,uint8_t correction,uint8_t channel,uint8_t subsystemmask,uint8_t attribute);
//uint8_t BMsg838::GetSBASStatus();
//6280		char*	RespondSBASStatus(uint8_t Enable,uint8_t ranging,uint8_t URAmask,uint8_t correction,uint8_t channel,uint8_t subsystemmask,uint8_t *respond);

//uint8_t BMsg838::SetQZSS(uint8_t Enable,uint8_t channel,uint8_t attribute);
//uint8_t BMsg838::GetQZSS();
//6281		char*	RespondQZSS(uint8_t Enable,uint8_t channel,uint8_t *respond);

//uint8_t BMsg838::SetSAEE(uint8_t Enable,uint8_t attribute);
//uint8_t BMsg838::GetSAEE();
//6380		char*	RespondSAEE(uint8_t Status,uint8_t *respond);


//uint8_t BMsg838::Getbootstatus(uint8_t *respond);
//6480:4		char*	respondbootstatus(uint8_t status,uint8_t type,uint8_t *respond);

//uint8_t BMsg838::SetExtendedNMEA(uint8_t interval[],uint8_t attribute);
//uint8_t BMsg838::GetExtendedNMEA();
//6481:14			char*	respondExtendedNMEA(uint8_t interval[],uint8_t *respond);

//uint8_t BMsg838::SetInferenceDetection(uint8_t detectcontrol,uint8_t attribute);
//uint8_t BMsg838::GetInferenceDetection();
//6483:4			char*	respondInferenceDetection(uint8_t detectcontrol,uint8_t status,uint8_t *respond);

//uint8_t BMsg838::SetParamseachNum(uint8_t num,uint8_t attributed);
//uint8_t BMsg838::GetParamseachNum();
//640A:3			char*	respondParamseachNum(uint8_t num,uint8_t *respond);
//uint8_t BMsg838::Setephemeris(uint8_t **subframe);
//2E       uint8_t BMsg838::GetDOPMask();
//AF       uint8_t BMsg838::respondDOPMask();
//uint8_t BMsg838::GetElevaCNRMask();
//	B0		char*	respondElevationCNRmask(uint16_t len, uint8_t *respond);
//uint8_t BMsg838::SetDopMask(uint8_t mode,uint16_t pdopval,uint16_t hdopval,uint16_t gdopval,uint8_t Atribute);
//uint8_t BMsg838::SetElevationCnrMask(uint8_t mode,uint8_t elevamask,uint8_t cnrmask,uint8_t Atribute);
//uint8_t BMsg838::SetNMEAmessage(uint8_t *interval,uint8_t attribute){}
//uint8_t BMsg838::DownloadImage(uint8_t Baudrate, uint8_t type,uint16_t Flushid,uint8_t Index){}
//15        uint8_t BMsg838::GetPowerMode(uint8_t Atribute);
//B9		char*	respondPWRModeStatus(uint16_t len, uint8_t *respond)
//uint8_t BMsg838::Getephemeris(uint8_t sv);
// B1			char*	respondGPSEphemeris(uint16_t len, uint8_t *respond);
//



// public methods
//
/*====================NMEA Message Interface function=================*/
bool BMsg838::encode(char c)
{
  bool valid_sentence = false;
  // Serial.print(c); // TJS:
  ++_encoded_characters;
  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term))
    {
      _term[_term_offset] = 0;
      valid_sentence = term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _is_checksum_term = false;
    _gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1)
    _term[_term_offset++] = c;
  if (!_is_checksum_term)
    _parity ^= c;

  return valid_sentence;
}

#ifndef _GPS_NO_STATS
void BMsg838::stats(unsigned long *chars, unsigned short *sentences, unsigned short *failed_cs)
{
  if (chars) *chars = _encoded_characters;
  if (sentences) *sentences = _good_sentences;
  if (failed_cs) *failed_cs = _failed_checksum;
}
#endif

//
// internal utilities
//
int BMsg838::from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

unsigned long BMsg838::parse_decimal()
{
  char *p = _term;
  bool isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * gpsatol(p);
  while (gpsisdigit(*p)) ++p;
  if (*p == '.')
  {
    if (gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}

unsigned long BMsg838::parse_degrees()
{
  char *p;
  unsigned long left = gpsatol(_term);
  unsigned long tenk_minutes = (left % 100UL) * 10000UL;
  for (p=_term; gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 1000;
    while (gpsisdigit(*++p))
    {
      tenk_minutes += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) * 100000 + tenk_minutes / 6;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool BMsg838::term_complete()
{
  // Serial.println(_term); // TJS:
  if (_is_checksum_term)
  {
    byte checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
    if (checksum == _parity)
    {
      if (_gps_data_good)
      {
#ifndef _GPS_NO_STATS
        ++_good_sentences;
#endif
        _last_position_fix = _new_position_fix;
	_last_time_fix = _new_time_fix;

        switch(_sentence_type)
        {
        case _GPS_SENTENCE_GPRMC:
          _time      = _new_time;
          _date      = _new_date;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _speed     = _new_speed;
          _course    = _new_course;
	  // Serial.println("GPRMC"); // TJS:
	  if(fix_callback != NULL){ // TJS: 
	    fix_callback(_date, _time, _latitude, _longitude, 0, _speed, _course); // TJS:
	  } // TJS:
          break;
        case _GPS_SENTENCE_GPGGA:
          _altitude  = _new_altitude;
          _time      = _new_time;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
	  // Serial.println("GPGGA"); // TJS:
	  if(fix_callback != NULL){ // TJS: 
	    fix_callback(0, _time, _latitude, _longitude, _altitude, 0, 0); // TJS:
	  } // TJS:
          break;
        }

        return true;
      }
    }

#ifndef _GPS_NO_STATS
    else
      ++_failed_checksum;
#endif
    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!gpsstrcmp(_term, _GPRMC_TERM))
      _sentence_type = _GPS_SENTENCE_GPRMC;
    else if (!gpsstrcmp(_term, _GPGGA_TERM))
      _sentence_type = _GPS_SENTENCE_GPGGA;
    else
      _sentence_type = _GPS_SENTENCE_OTHER;
    return false;
  }

  if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
  switch((_sentence_type == _GPS_SENTENCE_GPGGA ? 200 : 100) + _term_number)
  {
    case 101: // Time in both sentences
    case 201:
      _new_time = parse_decimal();
      _new_time_fix = millis();
      break;
    case 102: // GPRMC validity
      _gps_data_good = _term[0] == 'A';
      break;
    case 103: // Latitude
    case 202:
      _new_latitude = parse_degrees();
      _new_position_fix = millis();
      break;
    case 104: // N/S
    case 203:
      if (_term[0] == 'S')
        _new_latitude = -_new_latitude;
      break;
    case 105: // Longitude
    case 204:
      _new_longitude = parse_degrees();
      break;
    case 106: // E/W
    case 205:
      if (_term[0] == 'W')
        _new_longitude = -_new_longitude;
      break;
    case 107: // Speed (GPRMC)
      _new_speed = parse_decimal();
      break;
    case 108: // Course (GPRMC)
      _new_course = parse_decimal();
      break;
    case 109: // Date (GPRMC)
      _new_date = gpsatol(_term);
      break;
    case 206: // Fix data (GPGGA)
      _gps_data_good = _term[0] > '0';
      break;
    case 209: // Altitude (GPGGA)
      _new_altitude = parse_decimal();
      break;
  }

  return false;
}

long BMsg838::gpsatol(const char *str)
{
  long ret = 0;
  while (gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}

int BMsg838::gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}
void BMsg838::crack_datetime(int *year, byte *month, byte *day, 
  byte *hour, byte *minute, byte *second, byte *hundredths, unsigned long *age)
{
  unsigned long date, time;
  get_datetime(&date, &time, age);
  if (year) 
  {
    *year = date % 100;
    *year += *year > 80 ? 1900 : 2000;
  }
  if (month) *month = (date / 100) % 100;
  if (day) *day = date / 10000;
  if (hour) *hour = time / 1000000;
  if (minute) *minute = (time / 10000) % 100;
  if (second) *second = (time / 100) % 100;
  if (hundredths) *hundredths = time % 100;
}
float BMsg838::distance_between (float lat1, float long1, float lat2, float long2) 
{
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  return delta * 6372795; 
}

float BMsg838::course_to (float lat1, float long1, float lat2, float long2) 
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

const char *BMsg838::cardinal (float course)
{
  static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}
void BMsg838::f_get_position(float *latitude, float *longitude, unsigned long *fix_age)
{
  long lat, lon;
  get_position(&lat, &lon, fix_age);
  *latitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lat / 1000000.0);
  *longitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lon / 1000000.0);
}			

float BMsg838::f_altitude()    
{
  return _altitude == GPS_INVALID_ALTITUDE ? GPS_INVALID_F_ALTITUDE : _altitude / 100.0;
}

float BMsg838::f_course()
{
  return _course == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : _course / 100.0;
}

float BMsg838::f_speed_knots() 
{
  return _speed == GPS_INVALID_SPEED ? GPS_INVALID_F_SPEED : _speed / 100.0;
}

float BMsg838::f_speed_mph()   
{ 
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPH_PER_KNOT * sk; 
}

float BMsg838::f_speed_mps()   
{ 
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPS_PER_KNOT * sk; 
}

float BMsg838::f_speed_kmph()  
{ 
  float sk = f_speed_knots();
  return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_KMPH_PER_KNOT * sk; 
}

const float BMsg838::GPS_INVALID_F_ANGLE = 1000.0;
const float BMsg838::GPS_INVALID_F_ALTITUDE = 1000000.0;
const float BMsg838::GPS_INVALID_F_SPEED = -1.0;		
	
