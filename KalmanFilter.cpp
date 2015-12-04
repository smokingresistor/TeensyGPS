/*
	This is a code that process Kalman Filtering of position date from GPS.
	
*/

#include "KalmanFilter.h"

/*Kalman Filter class Structor*/
KalmanFilter::KalmanFilter()     

{
  //i = 1;
	cosLat = 0.0;  
	Rearth = 6378137;
	
}
/*Kalman Filter class Destroier*/
KalmanFilter::~KalmanFilter()
{

}
//
// public methods
//
/*This is a function to filter the GPS data from GPS
	Parameter: lat-> Position data1
			   lon-> Position data2
	filtered data->GPS_data:
*/
int64_t* KalmanFilter::KalmanProcessing(int64_t lat, int64_t lon){
	 time = millis();
         prevGPS_data[0][0] = GPS_data[0][0];
         prevGPS_data[0][1] = GPS_data[0][1];
         prevGPS_data[0][2] = GPS_data[0][2];
         GPS_data[0][0] = lat;
         GPS_data[0][1] = lon;
         GPS_data[0][2] = (uint64_t) time;
         
          //if (i != 100){ //Initialize cosLat      
          Serial.print("\r\n ==================FirstLat: "); Serial.print((float)firstGPS_data[0][0]); Serial.print(" FirstLon: "); Serial.println((float)firstGPS_data[0][1]);        
          if (((firstGPS_data[0][0] == GPS_data[0][0])&&(firstGPS_data[0][1] == GPS_data[0][1]))||((firstGPS_data[0][0] == 0)&&(firstGPS_data[0][1] == 0))){
            Serial.println("\r\n INIT FIRST!!! ===========================================");
            firstGPS_data[0][0] = GPS_data[0][0];
            firstGPS_data[0][1] = GPS_data[0][1];
            firstGPS_data[0][2] = GPS_data[0][2]; 
            cosLat = cos((float) firstGPS_data[0][0]/10000000*pi/180);   
              i = 100;
          }              
          delta_T = ((float) (GPS_data[0][2] - prevGPS_data[0][2])) / 1000; //Time elapsed in seconds
          float Amatrix[4][4] = {
                 {1, 0,  delta_T, 0      },
                 {0, 1,  0,       delta_T},
                 {0, 0,  1,       0      },
                 {0, 0,  0,       1      },
          };          
          float nextXstateEstimate[4][1] = {
                {0},
                {0},
                {0},
                {0},
          };    
          Matrix.Multiply((float*) Amatrix, (float*) Xstate, 4, 4, 1, (float*) nextXstateEstimate);           
          /*************Line 78 of Matlab code*************/
          float PerrorCovarianceEstimate[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float AmatrixTranspose[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateProductMatrix[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateSumMatrix[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };          
          Matrix.Transpose((float*) Amatrix, 4 ,4, (float *) AmatrixTranspose);
          Matrix.Multiply((float*) Amatrix, (float*) PerrorCovariance, 4,4,4, (float*) IntermediateProductMatrix);
          Matrix.Multiply((float*) IntermediateProductMatrix, (float*) AmatrixTranspose, 4,4,4, (float*) IntermediateSumMatrix);
          Matrix.Add((float*) IntermediateSumMatrix, (float*) QcovarianceMatrix, 4, 4, (float*) PerrorCovarianceEstimate);
          /*************Line 81 of Matlab code*************/
          float KalmanGain[4][2] = {
                {0.0, 0.0},
                {0.0, 0.0},
                {0.0, 0.0},
                {0.0, 0.0},
          };    
          float IntermediateProductMatrix2[2][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateProductMatrix3[2][2] = {
                {0.0, 0.0},
                {0.0, 0.0}, 
          };
          float IntermediateQuotientMatrix[4][2] = {
                {0.0, 0.0},
                {0.0, 0.0},
                {0.0, 0.0},
                {0.0, 0.0},
          };
          float IntermediateSumMatrix2[2][2] = {
                {0.0, 0.0}, 
                {0.0, 0.0},
          };
          Matrix.Multiply((float*) Hmatrix, (float*) PerrorCovarianceEstimate, 2,4,4, (float*) IntermediateProductMatrix2);
          Matrix.Multiply((float*) IntermediateProductMatrix2, (float*) HmatrixTranspose, 2,4,2, (float*) IntermediateProductMatrix3);
          Matrix.Add((float*) IntermediateProductMatrix3, (float*) RcovarianceMatrix, 2, 2, (float*) IntermediateSumMatrix2);
          int flag = Matrix.Invert((float*) IntermediateSumMatrix2, 2); //The argument matrix gets inverted; no need to create temp matrix.
          if (flag == 0){
            Serial.println("CANT INVERT");
          }
          Matrix.Multiply((float*) PerrorCovarianceEstimate, (float*) HmatrixTranspose, 4,4,2, (float*) IntermediateQuotientMatrix);
          Matrix.Multiply((float*) IntermediateQuotientMatrix, (float*) IntermediateSumMatrix2, 4,2,2, (float*) KalmanGain);
          data[0] = (float) (GPS_data[0][0]);///10000000.0;//-firstGPS_data[0][0]/10000000.0);//10000000.0;//*pi/180*Rearth/10000000.0;
          data[1] = (float) (GPS_data[0][1]);///10000000.0;//-firstGPS_data[0][1]/10000000.0);///10000000.0;//*pi/180*Rearth*cosLat/10000000.0; 
          Serial.print("\r\n i: "); Serial.print(i); 
          Serial.print("\r\n FirstLat: "); Serial.print((float)firstGPS_data[0][0]); Serial.print(" FirstLon: "); Serial.println((float)firstGPS_data[0][1]);  
          Serial.print("\r\n LstLat: "); Serial.print((float)GPS_data[0][0]); Serial.print(" LastLon: "); Serial.println((float)GPS_data[0][1]); 
          Serial.print("\r\n Lat: "); Serial.print(data[0]); Serial.print(" Lon: "); Serial.println(data[1]);
          float ZkTranspose[2][1] = { //We only need the transposed version of this, so we do it right here.
                {data[0]},
                {data[1]},
                }; 
          float IntermediateProductMatrix4[2][1] = {
                {0.0}, 
                {0.0},
          };
          float IntermediateProductMatrix5[4][1] = {
                {0.0}, 
                {0.0},
                {0.0}, 
                {0.0},
          };
          float IntermediateSubtractionMatrix[2][1] = {
                {0.0}, 
                {0.0},
          };
          Matrix.Multiply((float*) Hmatrix, (float*) nextXstateEstimate, 2,4,1, (float*) IntermediateProductMatrix4);
          Matrix.Subtract((float*) ZkTranspose, (float*) IntermediateProductMatrix4, 2, 1, (float*) IntermediateSubtractionMatrix);
          Matrix.Multiply((float*) KalmanGain, (float*) IntermediateSubtractionMatrix, 4,2,1, (float*) IntermediateProductMatrix5); //Reuse intermediate matrix because it has appropriate dimensions
          Matrix.Add((float*) nextXstateEstimate, (float*) IntermediateProductMatrix5, 4,1, (float*) Xstate); //NO NEED TO TRANSPOSE X STATE. WE DID THAT IN MATLAB FOR CONVENIENCE
          /*************Line 84 of Matlab code*************/
          Matrix.Multiply((float*) KalmanGain, (float*) Hmatrix, 4,2,4, (float*) IntermediateProductMatrix); //Reuse this intermediate matrix because it's 4x4 and we need 4x4
          float IdentityMatrix[4][4] = {
                {1,0,0,0},
                {0,1,0,0},
                {0,0,1,0},
                {0,0,0,1},
          };
          Matrix.Subtract((float*) IdentityMatrix, (float*) IntermediateProductMatrix, 4,4, (float*) IntermediateQuotientMatrix); //Reuse this intermediate matrix because it's 4x4 and we need 4x4.
          Matrix.Multiply((float*) IntermediateQuotientMatrix, (float*) PerrorCovarianceEstimate, 4,4,4, (float*) PerrorCovariance);              
          Matrix.Print((float*) Xstate, 4, 1, "Xstate Update: ");
          Matrix.Print((float*) ZkTranspose, 2, 1, "ZK transpose: ");
          Matrix.Print((float*) nextXstateEstimate, 4, 1, "Next X State Estimate: ");
          //i = i + 1; 
          GPS_data[0][0] = Xstate[0][0];
          GPS_data[0][1] = Xstate[1][0];
          return GPS_data[0];
        
}

int64_t* KalmanFilter::KalmanNoData(){
          uint64_t currentTime = (int32_t) millis();  
          delta_T = (float) (currentTime - time) / 1000; //Time elapsed in seconds
          float Amatrix[4][4] = {
                 {1, 0,  delta_T, 0      },
                 {0, 1,  0,       delta_T},
                 {0, 0,  1,       0      },
                 {0, 0,  0,       1      },
          };          
          float nextXstateEstimate[4][1] = {
                {0},
                {0},
                {0},
                {0},
          };    
          Matrix.Multiply((float*) Amatrix, (float*) Xstate, 4, 4, 1, (float*) nextXstateEstimate);
          Xstate[0][0] = nextXstateEstimate[0][0];
          Xstate[1][0] = nextXstateEstimate[1][0];
          Xstate[2][0] = nextXstateEstimate[2][0];
          Xstate[3][0] = nextXstateEstimate[3][0];
                    
          /*************Line 78 of Matlab code*************/
          float AmatrixTranspose[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateProductMatrix[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateSumMatrix[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          }; 
          float PerrorCovarianceEstimate[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };         
          Matrix.Transpose((float*) Amatrix, 4 ,4, (float *) AmatrixTranspose);
          Matrix.Multiply((float*) Amatrix, (float*) PerrorCovariance, 4,4,4, (float*) IntermediateProductMatrix);
          Matrix.Multiply((float*) IntermediateProductMatrix, (float*) AmatrixTranspose, 4,4,4, (float*) IntermediateSumMatrix);
          Matrix.Add((float*) IntermediateSumMatrix, (float*) QcovarianceMatrix, 4, 4, (float*) PerrorCovarianceEstimate);
          for (int j = 0; j < 4; j++){
            for (int k = 0; k < 4; k++){
              PerrorCovariance[j][k] = PerrorCovarianceEstimate[j][k];
            }
          }          
          GPS_data[0][0] = Xstate[0][0];
          GPS_data[0][1] = Xstate[1][0];
          time = millis();
          return GPS_data[0];                
}

float KalmanFilter::Smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return smoothedVal;
}

