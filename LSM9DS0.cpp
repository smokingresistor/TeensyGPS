#include "LSM9DS0.h"

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

int INT1XM = 16; // INT1XM tells us when accel data is ready
int INT2XM = 17; // INT2XM tells us when mag data is ready
int DRDYG  = 12; // DRDYG  tells us when gyro data is ready

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.

// Offsets applied to raw x/y/z values
float mag_offsets[3]            = { 0.078835, -0.097777, 0.049886 };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.147015, -0.017120, 0.009420 },
                                    { -0.017120, 1.147780, -0.011485},
                                    { 0.009420, -0.011485, 1.199737} }; 

float mag_field_strength        = 0.48;

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
// System constants
#define deltat 0.02f // sampling period in seconds (shown as 20 ms)
#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta 1 //sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float m_x, m_y, m_z; // magnetometer measurements
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error
uint32_t lastUpdate = 0, sumCount = 0;  // used to calculate integration interval
uint32_t Now = 0;                       // used to calculate integration interval
float abias[3] = {0, 0, 0}, gbias[3] = {-0.7, -6, -4}, gzero[3] = {1, 1, 1};
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
NXPSensorFusion sensorFilter;

/// \fn get_declination
/// \brief Return declination from gps coordinates \n
/// 
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

/// \fn sensor_9dof_configure
/// \brief Configure 9dof sensor \n
/// \detailed
/// 1. Set 4G diapason for accels \n
/// 2. Set 245 degree per second diapason for gyros \n
/// 3. Set 2 Gauss diapason for magnetometers \n
void sensor_9dof_configure()
{
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG,  INPUT);
  dof.begin();
  dof.setAccelScale(dof.A_SCALE_4G);
  dof.setGyroScale(dof.G_SCALE_245DPS);
  dof.setMagScale(dof.M_SCALE_2GS);
  // Set output data rates                   
  dof.setAccelODR(dof.A_ODR_200); // Set accelerometer update rate at 100 Hz
  dof.setAccelABW(dof.A_ABW_50); // Choose lowest filter setting for low noise                            380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
  dof.setGyroODR(dof.G_ODR_190_BW_125);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise
  dof.setMagODR(dof.M_ODR_125); // Set magnetometer to update every 80 ms
  dof.calLSM9DS0(gbias, abias);
  // sensor filter start
  sensorFilter.begin(50);
}

/// \fn
/// \brief Scale accels data by factor 1.5 (see datasheet) \n
///
void scale_accel_16g(){  //need to set abias
  dof.ax = 1.5 * dof.ax;
  dof.ay = 1.5 * dof.ay;
  dof.az = 1.5 * dof.az;
}

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z)
{
    // local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
    float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
    J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
    float h_x, h_y, h_z; // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;
    float twoSEq_4 = 2.0f * SEq_4;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xSEq_1 = 2.0f * b_x * SEq_1;
    float twob_xSEq_2 = 2.0f * b_x * SEq_2;
    float twob_xSEq_3 = 2.0f * b_x * SEq_3;
    float twob_xSEq_4 = 2.0f * b_x * SEq_4;
    float twob_zSEq_1 = 2.0f * b_z * SEq_1;
    float twob_zSEq_2 = 2.0f * b_z * SEq_2;
    float twob_zSEq_3 = 2.0f * b_z * SEq_3;
    float twob_zSEq_4 = 2.0f * b_z * SEq_4;
    float SEq_1SEq_2;
    float SEq_1SEq_3 = SEq_1 * SEq_3;
    float SEq_1SEq_4;
    float SEq_2SEq_3;
    float SEq_2SEq_4 = SEq_2 * SEq_4;
    float SEq_3SEq_4;
    float twom_x = 2.0f * m_x;
    float twom_y = 2.0f * m_y;
    float twom_z = 2.0f * m_z;
    // normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // normalise the magnetometer measurement
    norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    m_x /= norm;
    m_y /= norm;
    m_z /= norm;
    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
    f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    J_41 = twob_zSEq_3; // negated in matrix multiplication
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // compute then integrate the estimated quaternion rate
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
    // compute flux in the earth frame
    SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
    SEq_1SEq_3 = SEq_1 * SEq_3;
    SEq_1SEq_4 = SEq_1 * SEq_4;
    SEq_3SEq_4 = SEq_3 * SEq_4;
    SEq_2SEq_3 = SEq_2 * SEq_3;
    SEq_2SEq_4 = SEq_2 * SEq_4;
    h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
    q[0] = SEq_1;
    q[1] = SEq_2;
    q[2] = SEq_3;
    q[3] = SEq_4;
}

/// \fn sensor_9dof_read
/// \brief Read 9dof sensor \n
/// 
void sensor_9dof_read()
{
  //Serial.println(millis());
  float declination;
  if(digitalRead(DRDYG)){
    dof.readGyro();              // Read raw gyro data
    gx = dof.calcGyro(dof.gx) - gbias[0];   // Convert to degrees per seconds
    gy = dof.calcGyro(dof.gy) - gbias[1];
    gz = dof.calcGyro(dof.gz) - gbias[2];
  }
  if(digitalRead(INT1XM)){
    dof.readAccel();              // Read raw accelerometer data
  #if defined ACCEL_16G
    scale_accel_16g();
  #endif
    ax = dof.calcAccel(dof.ax)- abias[0];   // Convert to g's
    ay = dof.calcAccel(dof.ay)- abias[1];
    az = dof.calcAccel(dof.az)- abias[2];   
  }
  if(digitalRead(INT2XM)){
    dof.readMag();                // Read raw magnetometer data
    x = dof.calcMag(dof.mx) - mag_offsets[0];     // Convert to Gauss
    y = dof.calcMag(dof.my) - mag_offsets[1];
    z = dof.calcMag(dof.mz) - mag_offsets[2];
  }
  // Apply mag soft iron error compensation
  mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
  dof.readTemp();
  temp = 21.0 + (float) dof.temperature/8.0;
  //Normalize accelerometer raw values.
  float accXnorm = ax/sqrt(pow(ax,2) + pow(ay,2) + pow(az,2));
  float accYnorm = ay/sqrt(pow(ax,2) + pow(ay,2) + pow(az,2));
  //Calculate pitch and roll
  pitch = asin(-accXnorm);
  roll = asin(accYnorm/cos(pitch));
  float magXcomp = mx*cos(pitch) - mz*sin(pitch);
  float magYcomp = mx*sin(roll)*sin(pitch)+my*cos(roll)+mz*sin(roll)*cos(pitch);
  float magZcomp = mx*cos(roll)*sin(pitch)+my*sin(roll)-mz*cos(roll)*cos(pitch);
  // Filter gyro noise
  if (abs(gx) < gzero[0])
      gx = 0;
  if (abs(gy) < gzero[1])
      gy = 0;
  if (abs(gz) < gzero[2])
      gz = 0;
  filterUpdate(gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, ax, ay, az, mx,  my, -mz);
  yaw   = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI; 
  roll  *= 180.0f / PI;
  if ((az < 0)&&(pitch > 0))
     pitch = 180 - pitch; // pitch change from 0 to 180 degree
  if ((az < 0)&&(pitch < 0))
     pitch = - 180 - pitch; // pitch change from 0 to -180 degree
  declination = get_declination (gps.venus838data_filter.Latitude, gps.venus838data_raw.Longitude);
  heading = yaw + 180 + declination;
  if (heading >= 360)
      heading = heading - 360;
  if (heading < 0)
      heading = heading + 360;
  inclination = 90 - 180*atan2f(sqrt(sq(magYcomp)+sq(magXcomp)),magZcomp)/M_PI;
  if (magZcomp < 0)
     inclination = - inclination;
  print_9dof_data();
}

/// \fn print_9dof_data
/// \brief Print 9dof data in serial \n
///
void print_9dof_data()
{
  // print out accelleration data
  Serial.print("Accel X: "); Serial.print(ax*ACC_SCALE, 3); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(ay*ACC_SCALE, 3);   Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(az*ACC_SCALE, 3);   Serial.println("  \tmg");
  // print out magnetometer data
  Serial.print("Magn. X: "); Serial.print(mx, 3); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(my, 3);   Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(mz, 3);   Serial.println("  \tgauss");
  // print out gyroscopic data
  Serial.print("Gyro  X: "); Serial.print(gx, 3); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(gy, 3);   Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(gz, 3);   Serial.println("  \tdps");

  // print out temperature data
  Serial.print("Temp: "); Serial.print(temp); Serial.println(" *C");

  // 'orientation' should have valid .roll and .pitch fields 
  Serial.println(F("Orientation: "));
  Serial.print("Roll: ");
  Serial.println(roll, 7);
  Serial.print("Pitch: ");
  Serial.println(pitch, 7);
  Serial.print("Yaw: ");
  Serial.println(yaw, 7);
  Serial.print("Heading: ");
  Serial.println(heading, 7);
  Serial.print("Inclination: ");
  Serial.println(inclination, 7);
  Serial.print("Pressure: "); Serial.print(bmp280_pressure, 3); Serial.println("  \tmBar");
  Serial.print(F(" "));
  Serial.println("**********************\n");

  Serial.print(q[0], 7);  
  Serial.print(","); 
  Serial.print(q[1], 7);
  Serial.print(",");
  Serial.print(q[2], 7);
  Serial.print(",");
  Serial.print(q[3], 7);
  Serial.println(",");
}

