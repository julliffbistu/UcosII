/** 
 * @file Vector.c
 * 
 * @detail Description : 
 * Vector operations - function Calls 
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
#include "vector.h"

/**
* @brief <b>Function Name</b>:     : Vector_Dot_Product                                               
* @brief  <b>Description</b>: This function calculates the dot product of vector1
* and vector2, then it stores the result in op.
* @param Input Parameters: 
*<BR> float vector1 is a 1x3 input vector.
*<BR> float vector2 is a 1x3 input vector.
* @return Return Values:
*<BR> float op is the dot product of vector1 and vector2.
**/ 
float Vector_Dot_Product(float * vector1,float * vector2)
{
  int16_t c;
  float op=0;
  
  for(c=0; c<3; c++)
  {
  	op+=vector1[c]*vector2[c];
  }
  
  return op; 
}

/**
* @brief <b>Function Name</b>:     : Vector_Cross_Product                                               
* @brief  <b>Description</b>: This function calculates the cross product of v1
* and v2, then it stores the result in vectorOut.
* @param Input Parameters: 
*<BR> float *vectorOut is a 1x3 matrix where the cross product is stored.
*<BR> float *v1 is a 1x3 matrix input vector.
*<BR> float *v2 is a 1x3 matrix input vector.
* @return Return Values None
**/ 
void Vector_Cross_Product(float * vectorOut, float * v1,float * v2)
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

/**
* @brief <b>Function Name</b>:     : Vector_Scale                                               
* @brief  <b>Description</b>: This function calculates the scalar multiplication
* of a 1x3 matrix.
* @param Input Parameters: 
*<BR> float *vectorOut is a 1x3 matrix where the scalar multiplication is stored.
*<BR> float *vectorIn is a 1x3 matrix input vector.
*<BR> float scale2 is the scalar input.
* @return Return Values None
**/ 
void Vector_Scale(float * vectorOut,float * vectorIn, float scale2)
{
  int16_t c;
  for(c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2; 
  }
}

/**
* @brief <b>Function Name</b>:     : Vector_Add                                               
* @brief  <b>Description</b>: This function calculates the addition of two
* 1x3 vectors. 
* @param Input Parameters: 
*<BR> float *vectorOut is a 1x3 matrix where the addition is stored.
*<BR> float *vectorIn1 is a 1x3 matrix input vector.
*<BR> float *vectorIn2 is a 1x3 matrix input vector.
* @return Return Values None
**/ 
void Vector_Add(float * vectorOut,float * vectorIn1, float * vectorIn2)
{
  int16_t c;
  for(c=0; c<3; c++)
  {
     vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}
