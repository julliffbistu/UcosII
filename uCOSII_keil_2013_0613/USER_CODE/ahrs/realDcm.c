/** 
 * @file DCM.c
 * 
 * @detail Description : 
 * Direction Cosine Matrix Function Calls 
 * 
 * @author E. Macias / D. Torres / S. Ravindran
 * @author Texas Instruments, Inc
 * @date December, 2011
 * @version 1.0 - Initial version
 * @note Built with IAR Embedded Workbench: 5.20.1 and CCS Version 5.1.0.09000
 **/
#include "lpc17xx.h"
#include "realDCM.h"
#include "AppRoutines.h"
#include <math.h>

float DCM_Matrix[3][3]       = {{1,0,0},{0,1,0},{0,0,1}}; 
float Update_Matrix[3][3]    = {{0,1,2},{3,4,5},{6,7,8}}; 
float Temporary_Matrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

float G_Dt = (float)1/SAMPLE_FREQ_HZ;              // Integration time (DCM algorithm)

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

float Accel_Vector[3]= {0,0,0};    //Store the acceleration in a vector
float Mag_Vector[3]= {0,0,0};      //Store the magnetometer direction in a vector
float Gyro_Vector[3]= {0,0,0};     //Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0};    //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};         //Omega Proportional correction
float Omega_I[3]= {0,0,0};         //Omega Integrator
float Omega[3]= {0,0,0};

/**
* @brief <b>Function Name</b>:     : constrain                                               
* @brief  <b>Description</b>: This function checks if the input value (data), is 
* between a range (bottom_range to top_range). If data is below the range it 
* returns the bottom_range, if is above it returns the top_range, else it 
* returns the input value. 
* @param Input Parameters: 
* <BR>float data  is the input value to be compared against the range.
* <BR>float bottom_range is the bottom number of the range.
* <BR>float top_range is the top number of the range.
* @return Return Values: 
* <BR>float return_value is the result of whether data is within the range, 
* higher than the top_range, or lower than the bottom_range. 
**/ 
float constrain(float data, float bottom_range, float top_range)
{
  // Check if data is between bottom_range and top_range, if so return data
  // Else if data is less/equal than bottom range, return bottom range 
  // Else if data is greater/equal than top_range, return top_range 
  float return_value = data;
  if(data >= top_range)
  	return_value = top_range;
  else if( data <= bottom_range)
  	return_value = bottom_range;
  return return_value;
}

/**
* @brief <b>Function Name</b>:     : Normalize                                               
* @brief  <b>Description</b>: This function normalizes the DCM Matrix.
* @param Input Parameters: None
* @return Return Values: None
**/ 
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  boolean_t problem=false_state;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= Vector_Dot_Product(&temporary[0][0],&temporary[0][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);  
//    USB_Send("Square root called in renormalization",strlen("Square root called in renormalization"));
  } else {
    problem = true_state;
//    USB_Send("Problem detected!   Renorm 1 = ",strlen("Problem detected!   Renorm 1 = "));
//    USB_Send_float(renorm);
  }
      Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= Vector_Dot_Product(&temporary[1][0],&temporary[1][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);    
//    USB_Send("Square root called in renormalization",strlen("Square root called in renormalization"));
  } else {
    problem = true_state;
//    USB_Send("Problem detected!   Renorm 2 = ",strlen("Problem detected!   Renorm 2 = "));
//    USB_Send_float(renorm);
  }
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= Vector_Dot_Product(&temporary[2][0],&temporary[2][0]); 
  if (renorm < 1.5625f && renorm > 0.64f) {
    renorm= .5 * (3-renorm);                                                 //eq.21
  } else if (renorm < 100.0f && renorm > 0.01f) {
    renorm= 1. / sqrt(renorm);  
//    USB_Send("Square root called in renormalization",strlen("Square root called in renormalization")); 
  } else {
    problem = true_state;
//    USB_Send("Problem detected!   Renorm 3 = ",strlen("Problem detected!   Renorm 3 = "));
//    USB_Send_float(renorm);    
  }
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
  
  // Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
  if (problem) {                
      DCM_Matrix[0][0]= 1.0f;
      DCM_Matrix[0][1]= 0.0f;
      DCM_Matrix[0][2]= 0.0f;
      DCM_Matrix[1][0]= 0.0f;
      DCM_Matrix[1][1]= 1.0f;
      DCM_Matrix[1][2]= 0.0f;
      DCM_Matrix[2][0]= 0.0f;
      DCM_Matrix[2][1]= 0.0f;
      DCM_Matrix[2][2]= 1.0f;
      problem = false_state;  
  }
}

/**
* @brief <b>Function Name</b>:     : Drift_correction                                               
* @brief  <b>Description</b>: This function uses the accelerometer and 
* magnetometer readings to calculate a reference vector used to remove the 
* drift from the gyroscope sensor. 
* @param Input Parameters: None
* @return Return Values: None
**/ 
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  float Integrator_magnitude;
//  float mag_projection;
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs((int)(1 - Accel_magnitude)),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(sen_data.magnetom_heading);
  mag_heading_y = sin(sen_data.magnetom_heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  
  //  Here we will place a limit on the integrator so that the integrator cannot ever exceed half the saturation limit of the gyros
  Integrator_magnitude = sqrt(Vector_Dot_Product(Omega_I,Omega_I));
  if (Integrator_magnitude > ToRad(300)) {
    Vector_Scale(Omega_I,Omega_I,0.5f*ToRad(300)/Integrator_magnitude);
//    USB_Send("Integrator being contrained from ",strlen("Integrator being contrained from "));
//    USB_Send_float(ToDeg(Integrator_magnitude));
//    USB_Send(" degrees",strlen(" degrees"));
  }    
}

/**
* @brief <b>Function Name</b>:     : Matrix_update                                               
* @brief  <b>Description</b>: This function corrects the drift in the gyroscope,
* by adding the proportional and intergrator terms to the gyroscope readings. 
* Afterwards the DCM Matrix is updated with the new values. 
* @param Input Parameters: None
* @return Return Values: None
**/ 
void Matrix_update(void)
{
  int16_t x,y;
  Gyro_Vector[0]=ToRad(sen_data.gyro_x); //gyro x roll
  Gyro_Vector[1]=ToRad(sen_data.gyro_y); //gyro y pitch
  Gyro_Vector[2]=ToRad(sen_data.gyro_z); //gyro Z yaw
 
  Accel_Vector[0]=sen_data.accel_x;
  Accel_Vector[1]=sen_data.accel_y;
  Accel_Vector[2]=sen_data.accel_z;
  
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
  
 
 #define OUTPUTMODE 1 
 #if OUTPUTMODE==1         
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(x=0; x<3; x++) //Matrix Addition (update)
  {
    for(y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

/**
* @brief <b>Function Name</b>:     : Euler_angles                                               
* @brief  <b>Description</b>: This function calculates the Euler angles 
* using the DCM Matrix. 
* @param Input Parameters: None
* @return Return Values: None
**/ 
void Euler_angles(void)
{
  #if (OUTPUTMODE==2)         // Only accelerometer info (debugging purposes)
    roll = atan2(Accel_Vector[1],Accel_Vector[2]);    // atan2(acc_y,acc_z)
    pitch = -asin((Accel_Vector[0])/(double)GRAVITY); // asin(acc_x)
    yaw = MAG_Heading;                     // Magnetic heading
  #else
    pitch = -asin(DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);  //  ***** Need to correct for magnetic variation
  #endif
}
