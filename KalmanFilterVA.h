/*This is a Program h file for Kalman filtering*/ 

#ifndef KalmanFilterVA_h
#define KalmanFilterVA_h

#include "MatrixMath.h"
#include "math.h"

class KalmanFilterVA
{
  public:
    KalmanFilterVA();
    ~KalmanFilterVA();
    unsigned long time;
    int i;
    float filterVal = 0.05;
    float pi = 3.1415926;
    float cosLat ;  
    
    int64_t Rearth ;
float RcovarianceMatrix[2][2] = {
         {2.21017383364137, 3.51637078682249},
         {3.51637078682249, 13.8032720993553},
  };
    
    float QcovarianceMatrix[4][4]= {
           {0.01,    0,       0,      0    },
           {0,       0.01,    0,      0    },
           {0,       0,       0.001,  0    },
           {0,       0,       0,      0.001},
   };
    float Hmatrix[2][4] = {
        {1,0,0,0},
        {0,1,0,0},
  };
    float HmatrixTranspose[4][2]= {
              {1.0, 0.0},
              {0.0, 1.0},
              {0.0, 0.0},
              {0.0, 0.0},
  };    
    float PerrorCovariance[4][4]= {
           {0.001, 0,     0,    0   },
           {0,     0.001, 0,    0   },
           {0,     0,     0.02, 0   },
           {0,     0,     0,    0.02},
  };
    float delta_T = 0; //((float) (GPS_data[i][2] - GPS_data[i-1][2])) / 1000; //Time elapsed in seconds
    float data[2]={0,0};  //{(float) (GPS_data[i][0]-GPS_data[0][0])*pi/180*Rearth/10000000,(float) (GPS_data[i][1]-GPS_data[0][1])*pi/180*Rearth*cosLat/10000000};
    float Xstate[4][1] = {
          {data[0]},
          {data[1]},
          {0.0    },
          {0.0    },
  };
    int64_t GPS_data[1][3] = {
          {0,0,0},
  };
   
      int64_t prevGPS_data[1][3] = {
          {0,0,0},
  };
    int64_t firstGPS_data[1][3] = {
          {0,0,0},
  };  

  
  
  
    int64_t* KalmanProcessing(int64_t lat, int64_t lon);
    int64_t* KalmanNoData();
 private:
};
#endif
