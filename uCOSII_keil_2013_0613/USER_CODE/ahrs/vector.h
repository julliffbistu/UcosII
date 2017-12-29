/** 
 * @file Vector.h
 * 
 * @detail Description : 
 * Vector operations - header 
 * 
 * @author E. Macias / D. Torres / S. Ravindran
 * @author Texas Instruments, Inc
 * @date December, 2011
 * @version 1.0 - Initial version
 * @note Built with IAR Embedded Workbench: 5.20.1 and CCS Version 5.1.0.09000
 **/

#ifndef VECTOR_H_
#define VECTOR_H_

extern float Vector_Dot_Product(float * vector1,float * vector2);
extern void Vector_Cross_Product(float * vectorOut, float * v1,float * v2);
extern void Vector_Scale(float * vectorOut,float * vectorIn, float scale2);
extern void Vector_Add(float * vectorOut,float * vectorIn1, float * vectorIn2);

#endif
