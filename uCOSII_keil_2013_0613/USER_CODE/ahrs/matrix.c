/** 
 * @file Matrix.c
 * 
 * @detail Description : 
 * Matrix multiply function
 * 
 * @author E. Macias / D. Torres / S. Ravindran
 * @author Texas Instruments, Inc
 * @date December, 2011
 * @version 1.0 - Initial version
 * @note Built with IAR Embedded Workbench: 5.20.1 and CCS Version 5.1.0.09000
 **/

#include "lpc17xx.h"
#include "matrix.h"

/**************************************************/
//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's.

/**
* @brief <b>Function Name</b>:     : Matrix_Multiply                                               
* @brief  <b>Description</b>: This function multiplies two 3x3 matrixes( a and b)
* and stores the result in matrix mat. 
* @param Input Parameters: 
*<BR> float a[3][3] is an input matrix.
*<BR> float b[3][3] is an input matrix.
*<BR> float mat[3][3] is the where the multiplication of a*b will be stored.
* @return Return Values None
**/ 
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
  int16_t x,y,w;
  float op[3]; 
  for(x=0; x<3; x++)
  {
    for(y=0; y<3; y++)
    {
      for(w=0; w<3; w++)
      {
       op[w]=a[x][w]*b[w][y];
      } 
      mat[x][y]=0;
      mat[x][y]=op[0]+op[1]+op[2];
    }
  }
}
