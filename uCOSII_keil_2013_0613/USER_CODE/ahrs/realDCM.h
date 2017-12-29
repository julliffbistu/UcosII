														   //=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#include "vector.h"
#include "matrix.h"

#ifndef DCM_H_
#define DCM_H_

extern void Normalize(void);
extern void Drift_correction(void);
extern void Matrix_update(void);
extern void Euler_angles(void);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
