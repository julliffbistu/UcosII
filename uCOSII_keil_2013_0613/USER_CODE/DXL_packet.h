//////////////////////////////////////////////////////////
//		Name: DXL_packet.h
//		Version: 1.10
//		Date: 2005.12.6
//		By:	ojh6t3k
//		Copyright 2005 ROBOTIS
//////////////////////////////////////////////////////////
#ifndef _DXL_PACKET_H
#define _DXL_PACKET_H


/**************** Data structure ************************/
#define MAX_DXL_PARAM_NUM		8			// Maximum parameter number

////////////// Instruction packet ////////////////////////
struct DXL_INST_PACKET
{
	Byte	id;							// Dynamixel ID
	Byte	length;						//		
	Byte	instruction;				// Instruction code
	Byte	parameter[MAX_DXL_PARAM_NUM];	// Parameters
};

////////////////// Status packet /////////////////////////
struct DXL_STATUS_PACKET
{
	Byte	id;							// Dynamixel ID
	Byte	length;						// Packet length = parameter's number + 1(ID) + 1(error)
	Byte	error;						// Error code
	Byte	parameter[MAX_DXL_PARAM_NUM];	// Parameters								
};

/****************** Define Macro ************************/

///////////////// Instruction code ///////////////////////
#define INST_PING           0x01		// Ping instruction
#define INST_READ           0x02		// Read instruction
#define INST_WRITE          0x03		// Write instruction
#define INST_REG_WRITE      0x04		// Reg_write instruction
#define INST_ACTION         0x05		// Action instruction
#define INST_RESET          0x06		// Reset instruction
#define INST_SYNC_WRITE     0x83		// Sync_write instruction

///////////////// Memory address /////////////////////////
// EEPROM
#define P_MODEL_NUMBER_L      0x00		// Model number lower Byte address
#define P_MODOEL_NUMBER_H     0x01		// Model number higher Byte address
#define P_VERSION             0x02		// DXL version address
#define P_ID                  0x03		// DXL ID address
#define P_BAUD_RATE           0x04		// DXL baudrate address
#define P_RETURN_DELAY_TIME   0x05		// Return delay time address
#define P_CW_ANGLE_LIMIT_L    0x06		// CW limited angle lower Byte address
#define P_CW_ANGLE_LIMIT_H    0x07		// CW limited angle higher Byte address
#define P_CCW_ANGLE_LIMIT_L   0x08		// CCW limited angle lower Byte address
#define P_CCW_ANGLE_LIMIT_H   0x09		// CCW limited angle higher Byte address
#define P_LIMIT_TEMPERATURE   0x0b		// Limited temperature address
#define P_DOWN_LIMIT_VOLTAGE  0x0c		// Down limited voltage address
#define P_UP_LIMIT_VOLTAGE    0x0d		// Up limited voltage address
#define P_MAX_TORQUE_L        0x0e		// Max torque lower Byte address
#define P_MAX_TORQUE_H        0x0f		// Max torque higher Byte address
#define P_RETURN_LEVEL        0x10		// Return level address
#define P_ALARM_LED           0x11		// Alarm LED address
#define P_ALARM_SHUTDOWN      0x12		// Alarm shutdown address
#define P_DOWN_CALIBRATION_L  0x14		// Down calibration lower Byte address
#define P_DOWN_CALIBRATION_H  0x15		// Down calibration higher Byte address
#define P_UP_CALIBRATION_L    0x16		// Up calibration lower Byte address
#define P_UP_CALIBRATION_H    0x17		// Up calibration higher Byte address
// RAM
#define P_TORQUE_ENABLE				0x18	// Torque enable flag address
#define P_LED						0x19	// LED on/off flag address
#define P_CW_COMPLIANCE_MARGIN		0x1a	// CW compliance margin address
#define P_CCW_COMPLIANCE_MARGIN		0x1b	// CCW compliance margin address
#define P_CW_COMPLIANCE_SLOPE		0x1c	// CW compliance slope address
#define P_CCW_COMPLIANCE_SLOPE		0x1d	// CCW compliance slope address
#define P_GOAL_POSITION_L			0x1e	// Goal position lower Byte address
#define P_GOAL_POSITION_H			0x1f	// Goal position higher Byte address
#define P_GOAL_SPEED_L				0x20	// Goal speed lower Byte address
#define P_GOAL_SPEED_H				0x21	// Goal speed higher Byte address
#define P_TORQUE_LIMIT_L			0x22	// Limited torque lower Byte address
#define P_TORQUE_LIMIT_H			0x23	// Limited torque higher Byte address
#define P_PRESENT_POSITION_L		0x24	// Present position lower Byte address
#define P_PRESENT_POSITION_H		0x25	// Present position higher Byte address
#define P_PRESENT_SPEED_L			0x26	// Present speed lower Byte address
#define P_PRESENT_SPEED_H			0x27	// Present speed higher Byte address
#define P_PRESENT_LOAD_L			0x28	// Present load lower Byte address
#define P_PRESENT_LOAD_H			0x29	// Present load higher Byte address
#define P_PRESENT_VOLTAGE			0x2a	// Present voltage address
#define P_PRESENT_TEMPERATURE		0x2b	// Present temperature address
#define P_REGISTERED_INSTRUCTION	0x2c	// Registered instruction address
#define P_MOVING					0x2e	// Moving state flag address
#define P_EEPROM_LOCK				0x2f	// EEPROM lock flag address
#define P_PUNCH_L					0x30	// Punch lower Byte address
#define P_PUNCH_H					0x31	// Punch higher Byte address

/////////////////// Error code bit /////////////////////////
#define ERROR_VOLTAGE			0x01	// 0th error bit	'00000001'	- DXL voltage error
#define ERROR_ANGLE				0x02	// 1th error bit	'00000010'	- DXL limited angle error
#define ERROR_OVERHEAT			0x04	// 2th error bit	'00000100'	- DXL overheatting error
#define ERROR_RANGE				0x08	// 3th error bit	'00001000'	- DXL range error
#define ERROR_CHECKSUM			0x10	// 4th error bit	'00010000'	- DXL packet's checksum error
#define ERROR_OVERLOAD			0x20	// 5th error bit	'00100000'	- DXL overload error
#define ERROR_INSTRUCTION		0x40	// 6th error bit	'01000000'	- DXL instruction code error

/////////////////// Alarm LED bit //////////////////////////
#define ALARM_VOLTAGE			0x01	// 0th alram bit	'00000001'	- DXL voltage error
#define ALARM_ANGLE				0x02	// 1th alram bit	'00000010'	- DXL limited angle error
#define ALARM_OVERHEAT			0x04	// 2th alram bit	'00000100'	- DXL overheating error
#define ALARM_RANGE				0x08	// 3th alram bit	'00001000'	- DXL range error
#define ALARM_CHECKSUM			0x10	// 4th alram bit	'00010000'	- DXL packet's checksum error
#define ALARM_OVERLOAD			0x20	// 5th alram bit	'00100000'	- DXL overload error
#define ALARM_INSTRUCTION		0x40	// 6th alram bit	'01000000'	- DXL instruction code error

///////////////// Alarm Shutdown bit ///////////////////////
#define SHUTDOWN_VOLTAGE		0x01	// 0th shutdown bit	'00000001'	- DXL voltage error
#define SHUTDOWN_ANGLE			0x02	// 1th shutdown bit	'00000010'	- DXL limited angle error
#define SHUTDOWN_OVERHEAT		0x04	// 2th shutdown bit	'00000100'	- DXL overheatting error
#define SHUTDOWN_RANGE			0x08	// 3th shutdown bit	'00001000'	- DXL range error
#define SHUTDOWN_CHECKSUM		0x10	// 4th shutdown bit	'00010000'	- DXL packet's checksum error
#define SHUTDOWN_OVERLOAD		0x20	// 5th shutdown bit	'00100000'	- DXL overload error
#define SHUTDOWN_INSTRUCTION	0x40	// 6th shutdown bit	'01000000'	- DXL instruction code error

////////////////// Dynamixel Baud rate /////////////////////
#define DXL_BAUD_1		0x01		// 1000000 bps
#define DXL_BAUD_3		0x03		// 500000 bps
#define DXL_BAUD_4		0x04		// 400000 bps
#define DXL_BAUD_7		0x07		// 250000 bps
#define DXL_BAUD_9		0x09		// 200000 bps
#define DXL_BAUD_16		0x10		// 115200 bps
#define DXL_BAUD_34		0x22		// 57600 bps
#define DXL_BAUD_103	0x67		// 19200 bps
#define DXL_BAUD_207	0xcf		// 9600 bps

///////////////////// Return level /////////////////////////
#define RETURN_NONE		0x00		// Return no status packet
#define RETURN_READ		0x01		// Return read instruction only
#define RETURN_ALL		0x02		// Return all instruction

/////////////////// Load direction /////////////////////////
#define CCW_LOAD		0x00		// CCW load direction
#define CW_LOAD			0x01		// CW load direction

////////////////// Registered state ////////////////////////
#define REGISTER_DONE	0x00		// Registered done
#define REGISTERED_INST	0x01		// Registered instruction

////////////////////// LED state ///////////////////////////
#define LED_OFF			0x00		// Turn off LED
#define LED_ON			0x01		// Turn on LED

//////////////////// Torque state //////////////////////////
#define TORQUE_OFF		0x00		// Turn off Torque
#define TORQUE_ON		0x01		// Turn on Torque

//////////////////// Moving state //////////////////////////
#define NOT_MOVING		0x00		// Stop state
#define MOVING			0x01		// Moving state

//////////////////// Range value /////////////////////////
#define MAX_ID				253
#define MAX_TEMPERATURE		150
#define MIN_VOLTAGE			50
#define MAX_VOLTAGE			250
#define MAX_POSITION		1023
#define MAX_SPEED			1023
#define MAX_TORQUE			1023

/////////////////// Miscellaneous //////////////////////////
#define BROADCASTING_ID		0x00fe	// Broadcasting ID



#endif
