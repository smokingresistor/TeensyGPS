/*BMsg838 - a small library for Teensy and Arduino, providing basic Binary Message and basic NMEA Message, which
	is sended from venus 838 GPS receiver. 
	This file is made two part: Binary message interface, NMEA message Interfacce.
		
*/
/*---------------------------------------------------------------------*/
/*==============================Header parts===========================*/
/*---------------------------------------------------------------------*/
#ifndef BMsg838_h
#define BMsg838_h
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#include "HardwareSerial.h"
/*---------------------------------------------------------------------*/
/*===================== Const for NMEA Interface======================*/
/*---------------------------------------------------------------------*/

#define _GPS_VERSION 1 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
// #define _GPS_NO_STATS
#define DPFP	double
#define SPFP    float
typedef struct GPSdata 
{
      int64_t  receivedtime;
      byte  fixmode;
      byte  NumSV;
      float Latitude;
      float Longitude;
      float SealevelAltitude;
      float velocity;
      float GDOP;
      float PDOP;
      float HDOP;
      float VDOP;
      float TDOP;
      
}NavGPSdata;

typedef struct SoftVersiondata 
{
      uint8_t type;   //Software type: 0?Reserved:(1)System code
      uint32_t Kversion;  //SkyTraq Kernel Version if 0ABC then version is A.B.C
      uint32_t ODMversion;	//SkyTraq Version 
      uint32_t revision;	//SkyTrsqRevision if 00YYMMDD
      bool ckecksumOK;
    
}GPSSoftVersiondata;


typedef void(*fix_cb_t)(unsigned long date, 
			unsigned long time,
			long lat,
			long lon,
			long alt,
			unsigned long speed,
			unsigned long course
			); // TJS:
class BMsg838
{
  public:
  	enum {
    GPS_INVALID_AGE = 0xFFFFFFFF,      GPS_INVALID_ANGLE = 999999999, 
    GPS_INVALID_ALTITUDE = 999999999,  GPS_INVALID_DATE = 0,
    GPS_INVALID_TIME = 0xFFFFFFFF,		 GPS_INVALID_SPEED = 999999999, 
    GPS_INVALID_FIX_TIME = 0xFFFFFFFF, GPS_INVALID_SATELLITES = 0xFF,
    GPS_INVALID_HDOP = 0xFFFFFFFF
  };

  static const float GPS_INVALID_F_ANGLE, GPS_INVALID_F_ALTITUDE, GPS_INVALID_F_SPEED;

    BMsg838();
    ~BMsg838();
 
 /*---------------------------------------------------------------------*/
/*============== Functions for Binary message passing==================*/
/*--Binary message syntax is 
	 <0xA0,0xA1><PL><Message ID><Message Body><CS><0x0D,0x0A>*/
/*---------------------------------------------------------------------*/   
    uint8_t   checksum(byte* buffer, int len);
 	  
    uint8_t   MakeBinaryMessage(int len);
    uint8_t   ReceiveNavigationData(uint8_t* modenum,int32_t *position, uint32_t* altitude, uint16_t *dilution, int32_t* coordinate,int32_t* veolcity);
    uint8_t   ResetGNSS(uint8_t startmode, uint16_t year, uint8_t month, uint8_t day,uint8_t hour, uint8_t minute, uint8_t second, int16_t latitude, int16_t longitude, int16_t altitude);
    uint8_t   GetSoftVersion();
    GPSSoftVersiondata* ResponseSoftVersion();
    uint8_t   GetSoftCRC();
    uint8_t   ResponseSoftCRC(uint8_t*Type, uint16_t*CRCinfo);
    uint8_t   SetFactoryDefalt();
    uint8_t   SetSerialPort(int Baudrate, uint8_t Atribute);
    uint8_t   SetBinaryMessagetype();
    uint8_t   SetPowerMode(uint8_t mode, uint8_t Atribute);

    uint8_t   SetPositionRate(uint8_t Rate);
    uint8_t   GetPositionRate();
    uint8_t   ResponsePositionRate(uint8_t* rate);
    uint8_t   SetNavigationInterval(uint8_t interval);
    uint8_t   SetPositionDatum(uint16_t index,uint8_t EllipIdx,uint16_t DeltaX,uint16_t DeltaY,uint16_t DeltaZ,uint32_t Semiaxis,uint8_t InversedFlatten);
    uint8_t   Getdatum();
    uint8_t   Responsedatum(uint16_t* Datum);	

    uint8_t   SetPositionPinning(uint8_t positionpinning);
    uint8_t   GetPositionPinning();
    uint8_t   ResponsePositionPinning(uint8_t*status, uint16_t* speed, uint16_t* cnt,uint16_t* upspeed,uint16_t* upcnt,uint16_t* updsit);
    uint8_t   SetPositionPinningParam(uint16_t speed,uint16_t cnt,uint16_t upspeed,uint16_t upcnt,uint16_t updistance);
    uint8_t   Get1PPSTiming();
    uint8_t   Response1PPSTiming(uint8_t* mode,uint32_t* len,uint32_t* standev,DPFP* savelati,DPFP* savelong,SPFP* savealti,uint8_t* runtimemode,uint32_t* runtimelen);
    uint8_t   Set1PPSCabledelay(uint32_t Cabledelay);
    uint8_t   Get1PPSCabledelay();
    uint8_t Response1PPSCabledelay(uint32_t *Cabledelay);
    uint8_t   SetGNSSNavigationMode(uint8_t mode);
    int8_t   GetGNSSNavigationMode();
	int8_t ResponseGNSSNavigationMode(char* mode);
    uint8_t   SetGNSSConstelGPStype();
    uint8_t   GetGNSSConstellationtype();
	uint8_t ResponseGNSSConstellationtype(char *mode);
    uint8_t   SetGPSUTCSecond(uint8_t leapsecond);
    uint8_t   GetGPSTime();
    uint8_t   respondGPSTime(uint32_t *Timeofweek, uint32_t *SubTimeofweek, uint16_t *weeknumber, uint8_t *Defleapsecond,int8_t *curleapsecond, uint8_t *Valid);

	uint8_t   SetDatumIndex(uint8_t index);
	uint8_t   ResponseDatumIndex(uint16_t *datumindex);
    uint8_t   GetDatumIndex();
    uint8_t   Set1PPSPulseWidth(uint32_t width );
    uint8_t   Get1PPSPulseWidth();
    uint8_t   Response1PPSPulseWidth(uint32_t* Width);
    uint8_t   Set1PPSFrequency(uint32_t frequency);
    uint8_t   Get1PPSFrequency();
    uint8_t   Response1PPSFrequency(uint32_t *frequency);
    
    byte SendStream[128];
    byte RecVBinarybuf[128];
    NavGPSdata venus838data_filter;
	NavGPSdata venus838data_raw;
	GPSSoftVersiondata softversion;
	
	
/*---------------------------------------------------------------------*/
/*===================== Function for NMEA Message======================*/
/*---------------------------------------------------------------------*/
    void add_callback(fix_cb_t fct_ptr); // TJS:
    bool encode(char c); // process one character received from GPS
	BMsg838 &operator << (char c) {encode(c); return *this;}
    
    // lat/long in hundred thousandths of a degree and age of fix in milliseconds
    inline void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0)
    {
      if (latitude) *latitude = _latitude;
      if (longitude) *longitude = _longitude;
      if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? 
        GPS_INVALID_AGE : millis() - _last_position_fix;
    }

    // date as ddmmyy, time as hhmmsscc, and age in milliseconds
    inline void get_datetime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
    {
      if (date) *date = _date;
      if (time) *time = _time;
      if (fix_age) *fix_age = _last_time_fix == GPS_INVALID_FIX_TIME ? 
        GPS_INVALID_AGE : millis() - _last_time_fix;
    }

    // signed altitude in centimeters (from GPGGA sentence)
    inline long altitude() { return _altitude; }

    // course in last full GPRMC sentence in 100th of a degree
    inline unsigned long course() { return _course; }
    
   // speed in last full GPRMC sentence in 100ths of a knot
  inline unsigned long speed() { return _speed; }

  // satellites used in last full GPGGA sentence
  inline unsigned short satellites() { return _numsats; }

  // horizontal dilution of precision in 100ths
  inline unsigned long hdop() { return _hdop; }

    void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0);
  	void crack_datetime(int *year, byte *month, byte *day, 
    byte *hour, byte *minute, byte *second, byte *hundredths = 0, unsigned long *fix_age = 0);
  	float f_altitude();
  	float f_course();
  	float f_speed_knots();
  	float f_speed_mph();
  	float f_speed_mps();
  	float f_speed_kmph();
  	
    static int library_version() { return _GPS_VERSION; }
    static float distance_between (float lat1, float long1, float lat2, float long2);
  	static float course_to (float lat1, float long1, float lat2, float long2);
  	static const char *cardinal(float course);
  	
  	#ifndef _GPS_NO_STATS
    void stats(unsigned long *chars, unsigned short *good_sentences, unsigned short *failed_cs);
#endif

private:
    enum {_GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_OTHER};
    
    // properties
    
    
    unsigned long _time, _new_time;
    unsigned long _date, _new_date;
    long _latitude, _new_latitude;
    long _longitude, _new_longitude;
    long _altitude, _new_altitude;
    unsigned long  _speed, _new_speed;
    unsigned long  _course, _new_course;
    unsigned long  _hdop, _new_hdop;
    unsigned short _numsats, _new_numsats;

    unsigned long _last_time_fix, _new_time_fix;
    unsigned long _last_position_fix, _new_position_fix;

    // parsing state variables
    byte _parity;
    bool _is_checksum_term;
    char _term[15];
    byte _sentence_type;
    byte _term_number;
    byte _term_offset;
    bool _gps_data_good;

#ifndef _GPS_NO_STATS
    // statistics
    unsigned long _encoded_characters;
    unsigned short _good_sentences;
    unsigned short _failed_checksum;
    unsigned short _passed_checksum;
#endif

    // internal utilities
    int from_hex(char a);
    unsigned long parse_decimal();
    unsigned long parse_degrees();
    bool term_complete();
    bool gpsisdigit(char c) { return c >= '0' && c <= '9'; }
    long gpsatol(const char *str);
    int gpsstrcmp(const char *str1, const char *str2);
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round 

#endif