
#ifndef GPSSerialMessageCom_h
#define GPSSerialMessageCom_h
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "BMsg838.h"
#include "KalmanFilter.h"
#include "KalmanFilterVA.h"


void printSoftversion(GPSSoftVersiondata* versioninfo);
void printpositionfloatformat(float a, int decimalscale, const char * mark, const char* unit);
int waitingRespondandReceive(byte *buf, byte messageid, unsigned int timeout);
int SendBinaryMessagetoGPSreceiver(int streamsize, byte* sendbuf, byte* receivebuf, byte messageid, int timeout);
int BinaryRecvMsgtype(const char *MsgInfo,byte* receivebuf);
int GPSNavigationMsgProcessing(NavGPSdata *filterbeforedata, NavGPSdata *filterafterdata,BMsg838 gps,KalmanFilter *filter,KalmanFilterVA *filterVA);
#endif
