#include "LSM9DS0.h"

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

int INT1XM = 16; // INT1XM tells us when accel data is ready
int INT2XM = 17; // INT2XM tells us when mag data is ready
int DRDYG  = 12; // DRDYG  tells us when gyro data is ready

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
//For Mahoni

#define GyroMeasError PI * (3.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 10.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f
float eInt[3] = {0.0f, 0.0f, 0.0f};



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


float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t now = 0;           // used to calculate integration interval
float abias[3] = {0, 0, 0}, gbias[3] = {-0.7, -6, -4};

LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

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

/// \fn MahonyQuaternionUpdate
/// \brief Return quaternions from 9 dof data \n
/// 
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
    {
        float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
        float norm;
        float hx, hy, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;
        float pa, pb, pc;

        // Auxiliary variables to avoid repeated arithmetic
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;   

        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
        hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
        bx = sqrt((hx * hx) + (hy * hy));
        bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

        // Estimated direction of gravity and magnetic field
        vx = 2.0f * (q2q4 - q1q3);
        vy = 2.0f * (q1q2 + q3q4);
        vz = q1q1 - q2q2 - q3q3 + q4q4;
        wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
        wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
        wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

        // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
        if (Ki > 0.0f)
        {
            eInt[0] += ex;      // accumulate integral error
            eInt[1] += ey;
            eInt[2] += ez;
        }
        else
        {
            eInt[0] = 0.0f;     // prevent integral wind up
            eInt[1] = 0.0f;
            eInt[2] = 0.0f;
        }
        // Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0];
        gy = gy + Kp * ey + Ki * eInt[1];
        gz = gz + Kp * ez + Ki * eInt[2];

        // Integrate rate of change of quaternion
        pa = q2;
        pb = q3;
        pc = q4;
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);
        // Normalise quaternion
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        norm = 1.0f / norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;

    }
    
/// \fn MadgwickQuaternionUpdate
/// \brief Return quaternions from 9 dof data \n
/// 
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
  
  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
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
}

/// \fn
/// \brief Scale accels data by factor 1.5 (see datasheet) \n
///
void scale_accel_16g(){  //need to set abias
  dof.ax = 1.5 * dof.ax;
  dof.ay = 1.5 * dof.ay;
  dof.az = 1.5 * dof.az;
}

/// \fn sensor_9dof_read
/// \brief Read 9dof sensor \n
/// 
void sensor_9dof_read()
{
  float declination;
  static float prev_yaw;
  float gx_q, gy_q, gz_q;
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
    mx = dof.calcMag(dof.mx);     // Convert to Gauss
    my = dof.calcMag(dof.my);
    mz = dof.calcMag(dof.mz);
  }
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
  // MadgwickQuaternionUpdate(ax, ay, az, gx_q, gy_q, gz_q, mx, my, mz);
  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
  roll = atan2(2*(q[0]*q[1]+q[2]*q[3]),(1-2*(sq(q[1])+sq(q[2]))))*180.0/M_PI;
  pitch = asin(2*(q[0]*q[2]-q[3]*q[1]))*180.0/M_PI;
  yaw = atan2(2*(q[0]*q[3]+q[1]*q[2]), (1-2*(sq(q[2])+sq(q[3]))))*180.0/M_PI;
  if ((az < 0)&&(pitch > 0))
     pitch = 180 - pitch; // pitch change from 0 to 180 degree
  if ((az < 0)&&(pitch < 0))
     pitch = - 180 - pitch; // pitch change from 0 to -180 degree
  declination = get_declination (gps.venus838data_filter.Latitude, gps.venus838data_raw.Longitude);
  heading = yaw + declination;
  inclination = 90 - 180*atan2f(sqrt(sq(magYcomp)+sq(magXcomp)),magZcomp)/M_PI;
  if (magZcomp < 0)
     inclination = - inclination;
  now = micros();
  deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;
  print_9dof_data();
}

/// \fn print_9dof_data
/// \brief Print 9dof data in serial \n
///
void print_9dof_data()
{
  Serial.print("Delta T: "); Serial.println(deltat, 4);

  // print out accelleration data
  Serial.print("Accel X: "); Serial.print(ax*ACC_SCALE, 3); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(ay*ACC_SCALE, 3);   Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(az*ACC_SCALE, 3);   Serial.println("  \mg");
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
  Serial.print("Pressure: "); Serial.print(bmp280_pressure, 3); Serial.println("  \mBar");
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

