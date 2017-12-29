#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "..\config.h"
#include "transplant.h"
#include "main.h"
#include "i2c_driver.h"
#include "ahrs\AppRoutines.h"
#include "ahrs\realDCM.h"

struct Sensors sensor;
struct SensorsRaw sensorsRaw;
struct SensorCalibration sensorCalibration;
struct s_sensor_data sen_data;
extern OS_EVENT *plckSensorAquire;

float roll=0;
float pitch=0;
float yaw=0;
unsigned int  Compass_counter=0;

int StoreCalibrationToFlash(struct SensorCalibration *psc)
{
	Uint16 fs=0xffff;
	Uint16 valid_stamp=0xAAAA;
	INT8U err=0;

	//the gyro dimension is defined as mrad 1/1000 d/s
	psc->gyro[0].scale=8960;		//in q7.9
	psc->gyro[1].scale=8960;		//in q7.9
	psc->gyro[2].scale=8960;		//in q7.9
	psc->accel[0].scale=-psc->accel[0].scale;
	psc->accel[1].scale=-psc->accel[1].scale;
	psc->accel[2].scale=-psc->accel[2].scale;

	CloseAll();
	fs=Open(253,FILE_WRITE_ONLY);
	Write(fs,&valid_stamp,1);
	Write(fs,(Uint16 *)psc,sizeof(struct SensorCalibration)/sizeof(Uint16));
	CloseAll();

	OSMutexPend(plckSensorAquire,0,&err);
	memcpy(&sensorCalibration,psc,sizeof(sensorCalibration));
	OSMutexPost(plckSensorAquire);
	return 1;
}

int LoadCalibrationFromFlash(struct SensorCalibration *psc)
{
	Uint16 fs=0xffff;
	Uint16 valid_stamp=0xffff;
	int ret=0;

	CloseAll();
	fs=Open(253,FILE_READ_ONLY);
	Read(fs,&valid_stamp,1);
	if(valid_stamp==0xAAAA)
	{
		Read(fs,(Uint16 *)psc,sizeof(struct SensorCalibration));
		ret=1;
	}
	CloseAll();
	return ret;
}

void IntializeCalibration(struct SensorCalibration *pc)
{
	//read initial configuration from flash file system
	if(LoadCalibrationFromFlash(pc))
		return;
	//the gyro dimension is defined as mrad 1/1000 d/s
	pc->gyro[0].offset=0;
	pc->gyro[0].scale=8960;		//in q7.9
	pc->gyro[1].offset=0;
	pc->gyro[1].scale=8960;		//in q7.9
	pc->gyro[2].offset=0;
	pc->gyro[2].scale=8960;		//in q7.9

	//the accelerator dimension is not important, only relative value concerns
	pc->accel[0].offset=0;
	pc->accel[0].scale=-512;		//in q7.9
	pc->accel[1].offset=0;
	pc->accel[1].scale=-512;		//in q7.9
	pc->accel[2].offset=0;
	pc->accel[2].scale=-512;		//in q7.9

	//the magnetic sensor dimention is not important,  only relative value concerns
	pc->mag[0].offset=0;
	pc->mag[0].scale=512;		//in q7.9
	pc->mag[1].offset=0;
	pc->mag[1].scale=512;		//in q7.9
	pc->mag[2].offset=0;
	pc->mag[2].scale=512;		//in q7.9
	return;
}

Uint16 InitSensor()
{
	INT8U temp=0;

	//Initialize sensor configuration
	OSTimeDly(500);	

	temp=0x34;
	I2C_WriteNByte(0x3C, 1, 0x00, &temp, 1);
	temp=0x60;
	I2C_WriteNByte(0x3C, 1, 0x01, &temp, 1);
	temp=0x00;
	I2C_WriteNByte(0x3C, 1, 0x02, &temp, 1);
	OSTimeDly(500);

	temp=0x08;
	I2C_WriteNByte(0xA6, 1, 0x2D, &temp, 1);
	temp=0x80;
	I2C_WriteNByte(0xA6, 1, 0x2E, &temp, 1);
	temp=0x0B;
	I2C_WriteNByte(0xA6, 1, 0x31, &temp, 1);

	temp=0x6F;
	I2C_WriteNByte(0xD2, 1, 0x20, &temp, 1);
	temp=0x04;
	I2C_WriteNByte(0xD2, 1, 0x21, &temp, 1);
	temp=0x08;
	I2C_WriteNByte(0xD2, 1, 0x22, &temp, 1);
	temp=0x10;
	I2C_WriteNByte(0xD2, 1, 0x23, &temp, 1);
	temp=0x10;
	I2C_WriteNByte(0xD2, 1, 0x24, &temp, 1);

	//initialize sensor storage
	memset(&sensor,0,sizeof(sensor));
	memset(&sensorsRaw,0,sizeof(sensorsRaw));
	memset(&sensorCalibration,0,sizeof(sensorCalibration));	
	memset(&sen_data,0,sizeof(sen_data));
	
	Compass_counter=0;
	roll=0;
	pitch=0;
	yaw=0;

	IntializeCalibration(&sensorCalibration);
	//initialize calibration parameter

	return 0;
}

/**
* @brief <b>Function Name</b>:     : Compass_Heading                                               
* @brief  <b>Description</b>: This function calculates the heading of the AHRS
* board using the hard and soft calibrated magnetometer values. Then stores the
* result in the sen_data structure. 
* @param Input Parameters: None
* @return Return Values: None
**/ 
void Compass_Heading(void)
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated Magnetic field X:
  MAG_X = sen_data.magnetom_x*cos_pitch + sen_data.magnetom_y*sin_roll*sin_pitch + sen_data.magnetom_z*cos_roll*sin_pitch;
  
  // Tilt compensated Magnetic field Y:
  MAG_Y = sen_data.magnetom_y*cos_roll - sen_data.magnetom_z*sin_roll;
  
  // Magnetic Heading
  sen_data.magnetom_heading = atan2(-1*MAG_Y,MAG_X);
  
  if(sen_data.magnetom_heading < 0) 
    sen_data.magnetom_heading += 2 * M_PI; 
}

Uint16 ConstrainMag(int16 *pmag)
{
	Uint16 i=0;
	for(i=0;i<3;i++)
	{
		if(pmag[i]>2046)
			pmag[i]=2046;
		if(pmag[i]<-2046)
			pmag[i]=-2046;
	}
	return 0;
}

Uint16 ConstrainAccel(int16 *paccel)
{
	return 0;
}

Uint16 ConstrainGyro(int16 *pgyro)
{
	return 0;
}

Uint16 HandleSensor()
{
	INT8U test[6];
	INT8U err=0;
	//static Uint32 inter;
	//read out the raw value from 9 axis sensor
	//read magnetic sensor
	
	I2C_ReadNByte (0x3C, 1, 0x03, test, 6);
	OSMutexPend(plckSensorAquire,0,&err);
	sensorsRaw.mag[0]=((((int16)test[0])<<8)|test[1]);
	sensorsRaw.mag[1]=((((int16)test[4])<<8)|test[5]);
	sensorsRaw.mag[2]=((((int16)test[2])<<8)|test[3]);
	ConstrainMag(sensorsRaw.mag);	
	OSMutexPost(plckSensorAquire);

	//read acceleration sensor
	I2C_ReadNByte (0xA6, 1, 0x32, test, 6);
	OSMutexPend(plckSensorAquire,0,&err);
	sensorsRaw.accel[0]=((((int16)test[1])<<8)|test[0]);
	sensorsRaw.accel[1]=((((int16)test[3])<<8)|test[2]);
	sensorsRaw.accel[2]=((((int16)test[5])<<8)|test[4]);
	ConstrainAccel(sensorsRaw.accel);	
	OSMutexPost(plckSensorAquire);

	I2C_ReadNByte (0xD2, 1, 0xA8, test, 6);
	OSMutexPend(plckSensorAquire,0,&err);
	sensorsRaw.gyro[0]=((((int16)test[1])<<8)|test[0]);
	sensorsRaw.gyro[1]=((((int16)test[3])<<8)|test[2]);
	sensorsRaw.gyro[2]=((((int16)test[5])<<8)|test[4]);
	ConstrainGyro(sensorsRaw.gyro);		
	OSMutexPost(plckSensorAquire);

	//inter=OSTimeGet();
	//apply offset and scale and convert from q7.9 to float
	OSMutexPend(plckSensorAquire,0,&err);
	sen_data.accel_x = ((float)(sensorsRaw.accel[0]-sensorCalibration.accel[0].offset)*sensorCalibration.accel[0].scale)/512.0f;
	sen_data.accel_y = ((float)(sensorsRaw.accel[1]-sensorCalibration.accel[1].offset)*sensorCalibration.accel[1].scale)/512.0f;
	sen_data.accel_z = ((float)(sensorsRaw.accel[2]-sensorCalibration.accel[2].offset)*sensorCalibration.accel[2].scale)/512.0f;

	sen_data.gyro_x = ((float)(sensorsRaw.gyro[0]-sensorCalibration.gyro[0].offset)*sensorCalibration.gyro[0].scale)/512.0f/1000.0f; 
	sen_data.gyro_y = ((float)(sensorsRaw.gyro[1]-sensorCalibration.gyro[1].offset)*sensorCalibration.gyro[1].scale)/512.0f/1000.0f;
	sen_data.gyro_z = ((float)(sensorsRaw.gyro[2]-sensorCalibration.gyro[2].offset)*sensorCalibration.gyro[2].scale)/512.0f/1000.0f; 

	sen_data.magnetom_x= ((float)(sensorsRaw.mag[0]-sensorCalibration.mag[0].offset)*sensorCalibration.mag[0].scale)/512.0f;
	sen_data.magnetom_y= ((float)(sensorsRaw.mag[1]-sensorCalibration.mag[1].offset)*sensorCalibration.mag[1].scale)/512.0f;
	sen_data.magnetom_z= ((float)(sensorsRaw.mag[2]-sensorCalibration.mag[2].offset)*sensorCalibration.mag[2].scale)/512.0f;
	OSMutexPost(plckSensorAquire);

	//every 50ms the compass is used to calculate directions
	if (Compass_counter++ >= 5)  
	{
		Compass_counter=0;          
		Compass_Heading();    // Read magnetometer 
	}

	Matrix_update(); 
	Normalize();
	Drift_correction();
	Euler_angles();

	OSMutexPend(plckSensorAquire,0,&err);
	sensor.incline[0]=(short)(roll*512);
	sensor.incline[1]=(short)(pitch*512);
	sensor.incline[2]=(short)(yaw*512);
	OSMutexPost(plckSensorAquire);

	//inter=OSTimeGet()-inter;
	/*
	Uint16 nbytes;
	Uint16 result[16];
	int32 temp=0;
	INT8U err=0;
	
	nbytes=ReadADCResult(result,sizeof(result));
	//execute acceleration filtering process
	//put the result in buffer
	//new file should be created to execute filtering and other tasks
	OSMutexPend(plckSensorAquire,0,&err);
	
	while(nbytes-->4)
	{
		temp=(((int32)result[nbytes]-1700)<<PRECISION_BIT)/1300;
		if(temp>(1<<PRECISION_BIT))
			temp=(1<<PRECISION_BIT);
		else if(temp<-(1<<PRECISION_BIT))
			temp=-(1<<PRECISION_BIT);
		temp=ARCSIN(temp);
		temp=temp*180/PI;
		if(nbytes%2==0)
			sensor.incline[0]=(int16)temp;
		else
			sensor.incline[1]=(int16)temp;
	}
	OSMutexPost(plckSensorAquire);
	*/
	return 0;
}

Uint16 GetSensorRawResult(struct SensorsRaw *psr)
{
	INT8U err=0;
	OSMutexPend(plckSensorAquire,0,&err);
	psr->gyro[0]=sensorsRaw.gyro[0];
	psr->gyro[1]=sensorsRaw.gyro[1];
	psr->gyro[2]=sensorsRaw.gyro[2];

	psr->accel[0]=sensorsRaw.accel[0];
	psr->accel[1]=sensorsRaw.accel[1];
	psr->accel[2]=sensorsRaw.accel[2];

	psr->mag[0]=sensorsRaw.mag[0];
	psr->mag[1]=sensorsRaw.mag[1];
	psr->mag[2]=sensorsRaw.mag[2];
	OSMutexPost(plckSensorAquire);
	return 0;
}

Uint16 GetSensorResult(struct Sensors *psensor)
{
	INT8U err=0;
	OSMutexPend(plckSensorAquire,0,&err);
	psensor->incline[0]=sensor.incline[0];
	psensor->incline[1]=sensor.incline[1];
	psensor->incline[2]=sensor.incline[2];
	//memcpy(psensor,&sensor,sizeof(sensor));
	OSMutexPost(plckSensorAquire);
	return 0;
}

Uint16 HandleSensorFeedback(struct StateSwapInput *pin,struct StateSwapOutput *pout)
{
	INT8U err=0;
	OSMutexPend(plckSensorAquire,0,&err);
	pout->sensors.incline[0]=sensor.incline[0];
	pout->sensors.incline[1]=sensor.incline[1];
	pout->sensors.incline[2]=sensor.incline[2];
	OSMutexPost(plckSensorAquire);
	return 0;
}

