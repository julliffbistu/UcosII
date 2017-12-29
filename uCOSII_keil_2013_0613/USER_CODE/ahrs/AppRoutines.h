/** 
 * @file AppRoutines.h
 * 
 * @detail Description : 
 * Sensor Calibration Header 
 * 
 * @author E. Macias / D. Torres / S. Ravindran
 * @author Texas Instruments, Inc
 * @date March, 2012
 * @version 2.0
 * @note Built with IAR Embedded Workbench: 5.20.1 and CCS Version 5.1.0.09000
 **/
#include "stdint.h"


#ifndef APP_ROUTINE_H_
#define APP_ROUTINE_H_

struct s_sensor_data
{  
    //This data has been corrected based on the calibration values
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float magnetom_x;
    float magnetom_y;
    float magnetom_z;
    float magnetom_heading;
};

typedef enum 
{
    false_state = 0,
    true_state
}boolean_t; 

extern float Accel_Vector[3];    //Store the acceleration in a vector
extern float Mag_Vector[3];      //Store the magnetometer direction in a vector
extern float Gyro_Vector[3];     //Store the gyros turn rate in a vector
extern float Omega_Vector[3];    //Corrected Gyro_Vector data
extern float Omega_P[3];         //Omega Proportional correction
extern float Omega_I[3];         //Omega Integrator
extern float Omega[3];

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi
#define M_PI 	 3.141592654
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 
#define Kp_ROLLPITCH 0.0125
#define Ki_ROLLPITCH 0.000008
#define Kp_YAW 1.2
#define Ki_YAW 0.000008
#define SAMPLE_FREQ_HZ     100
// Euler angles
extern float roll;
extern float pitch;
extern float yaw;

extern float errorRollPitch[3]; 
extern float errorYaw[3];

extern int16_t SENSOR_SIGN[9];

extern float G_Dt;    // Integration time (DCM algorithm)  We will run the integration loop at 200Hz if possible

extern float DCM_Matrix[3][3];
extern float Update_Matrix[3][3]; 
extern float Temporary_Matrix[3][3]; 

extern struct s_sensor_data sen_data;
extern struct s_sensor_offsets sen_offset;
extern void calibrateSensors(void);

#endif
