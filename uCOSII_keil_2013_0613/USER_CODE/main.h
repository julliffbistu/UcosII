#ifndef MAIN_H
#define MAIN_H

enum DXLDirectionType 
{
	RS485TX,
	RS485RX,
	TTLTX,
	TTLRX
};

Byte GetChar(void);
Uint16 SendPacket(Byte *pbuf,Uint16 len);
void UART0_IRQHandler(void);

Byte GetDXLChar(void);
Uint16 SendDXLPacket(Byte *pbuf,Uint16 len);
void UART3_IRQHandler(void);

#define MAX_ROBOT_JOINTS			32

#define SINGLE_PAGE_SIZE		128
#define FILE_READ_ONLY			0x0001
#define FILE_WRITE_ONLY			0x0002
void SSP0_IRQHandler(void);
void SpiFlashRead(Uint32 address,Uint16 *buffer,Uint16 size);
void SpiFlashProgram(Uint32 address,Uint16 *buffer,Uint16 size);
void InitFileSystem(void);
int16 Open(Uint16 id,Uint16 flag);
int16 Read(Uint16 file,Uint16 *pbuf,Uint16 len);
int16 Write(Uint16 file,Uint16 *pbuf,Uint16 len);
int16 Close(Uint16 file);
int16 CloseAll(void);

//sensor feedback interface
struct Sensors
{
	int16 incline[3];
};

struct SensorsRaw
{
	int16 gyro[3];
	int16 accel[3];
	int16 mag[3];
};

struct singleAxisCalibration
{
	int16 offset;
	int16 scale;
};

struct SensorCalibration
{
	struct singleAxisCalibration gyro[3];
	struct singleAxisCalibration accel[3];
	struct singleAxisCalibration mag[3];
};

Uint16 InitSensor(void);
Uint16 ReadADCResult(Uint16 *pbuf,Uint16 size);
Uint16 HandleSensor(void);
Uint16 GetSensorResult(struct Sensors *psensor);
void ADC_IRQHandler(void);

//signal and event management
extern Bool isTimerInterruptEnable;
extern Bool isADCInterruptEnable;
#define EnableTimerInterrupt(enable)	(isTimerInterruptEnable=(enable))
#define EnableADCInterrupt(enable)		(isADCInterruptEnable=(enable))
void WaitADCInterrupt(void);
void WaitDXLInterrupt(void);
void HandleInterrupt(void);

//gait manage interface
struct GaitEffect
{
	int16 xOffset;
	int16 yOffset;
	int16 thetaOffset;
};
typedef struct DownloadGaitHead
{
	Uint16 gaitID;
	Uint16 frameLength;
	Uint16 frameStep;
	Uint16 period;
	struct GaitEffect effect;	
}DownloadGaitHead;

#define MAX_ROBOT_JOINTS			32
#define DEFAULT_TURNING_SPEED		384
Uint16 ExecuteConfiguration(Uint16 *pconfig,Uint16 len);
Uint16 SmoothConfiguration(Uint16 *pconfig,Uint16 len);
Uint16 ExecuteGait(Uint16 id,Uint16 times);
Uint16 ExecuteGaitDirectionExt(struct GaitEffect *peffect);
void InitGaitGeneration(void);
Bool GenerateGaitStep(void);

//fast math calculation
extern const int16 SIN_TABLE[];
extern const int16 ASIN_TABLE[];
#define PRECISION_BIT		9
#define LENGTH_BIT_LIMIT	10
#define PI					1608
#define ARCSIN(a) 		((a)<0?(-ASIN_TABLE[-(a)]):(ASIN_TABLE[a]))
#define ARCCOS(a) 		((a)<0?(PI/2+ASIN_TABLE[-(a)]):(PI/2-ASIN_TABLE[a]))
int16 SQRT16(Uint32 M);
int16 ARCTAN2(int16 y,int16 x);
int16 SIN(int16 a);
int16 COS(int16 a);

//fast inverse kinematics related
struct RigidOffset
{
	int16 x;
	int16 y;
	int16 z;
};

struct RigidPose
{
	int16 alpha;
	int16 beta;
	int16 theta;
};

struct RigidBody
{
	struct RigidOffset offset;
	struct RigidPose pose;
};

#define INVALID_RIGIDBODY_VALUE	0xffff

struct SpecialGaitCommand
{
	Uint16 id;
	Uint16 times;
};

struct HeadMovingCommand
{
	int16 pitch;
	int16 yaw;
};
 
struct StateSwapInput
{
	Uint16 ctrReg1;
	Uint16 ctrReg2;
	struct GaitEffect dirInst;
	struct SpecialGaitCommand spcInst;
	struct HeadMovingCommand headInst;
};

#define GAIT_DIRECTION_VALID	0x0001
#define SPECIAL_GAIT_VALID		0x0002
#define GAIT_RESET_VALID		0x0004
#define COPY_RESET_ODOMETER		0x0008
#define HEAD_MOVE_VALID			0x0010
#define TORQUE_ENABLE_VALID		0x0020
#define SENSOR_ENABLE_VALID		0x0040
#define WALK_KICK_LEFT			0x0080
#define WALK_KICK_RIGHT			0x0100

#define SPECIAL_GAIT_PENDING	0x0001
#define GAIT_RESET_PENDING		0x0002
#define RESET_ODOMETER_PENDING	0x0004
#define WALK_KICK_PENDING		0x0008

struct StateSwapOutput
{
	Uint16 stsReg1;
	Uint16 stsReg2;
	struct GaitEffect dirSts;
	struct SpecialGaitCommand spcSts;
	struct HeadMovingCommand headSts;
	struct GaitEffect odometer;
	struct Sensors sensors;
	Uint16 isLeft;
	struct RigidBody torsoPose;
};

Uint16 HandleGaitInst(struct StateSwapInput *pin,struct StateSwapOutput *pout);
Uint16 HandleOdometer(struct StateSwapInput *pin,struct StateSwapOutput *pout);
Uint16 HandleSensorFeedback(struct StateSwapInput *pin,struct StateSwapOutput *pout);
Uint16 HandleHeadMoving(struct StateSwapInput *pin,struct StateSwapOutput *pout);
Uint16 HandleOtherState(struct StateSwapInput *pin,struct StateSwapOutput *pout);

struct LegParameter
{
	int16 legOffset;
	int16 hipHight;
	int16 thigh;
	int16 crus;
	int16 foot;
};


struct WalkingConfig
{
	int16 ystage;
	int16 xswing;
	int16 yswing;
	int16 zswing;
	int16 alphaswing;
	int16 betaswing;
	int16 thetaswing;
	int16 xlanding;
	int16 ylanding;
	int16 zlanding;
	int16 alphalanding;
	int16 betalanding;
	int16 thetalanding;
	int16 xpercentage;
	int16 ypercentage;
	int16 thetapercentage;
	int16 elbowswing;
	int16 armfly;
	int16 armswing;
	int16 maxTurningInc;
	int16 maxTurningAngle;
	int16 acceleration;
	int16 maxTurningRadius;
	int16 maxStepRadius;
	int16 planningStableCycle;
};

Uint16 GetInverseKine(struct RigidBody *pankle, struct RigidBody *phip, struct LegParameter *pleg,int16 *result);

/*
DSP_STATUS_PACKET *GetCOMHead(Byte *pbuf);
Uint16 WaitCOMWholePacket(Byte *pbuf,Uint16 size,Uint16 timeout);
Uint16 HandleInstPacket(Byte *pbuf,Uint16 size);

Byte GetDXLChar();
struct DXL_INST_PACKET *GetDXLHead(Byte *pbuf);
void ChangeDXLDirection(enum DXLDirectionType type);
Uint16 SendDXLPacket(Byte *pbuf,Uint16 len);
Uint16 WaitDXLWholePacket(Byte *pbuf,Uint16 size,Uint16 timeout);

//flash filesystem read/write interface
#define SINGLE_PAGE_SIZE		128
#define FILE_READ_ONLY			0x0001
#define FILE_WRITE_ONLY			0x0002
void SpiFlashRead(Uint32 address,Byte *buffer,Uint16 size);
void SpiFlashProgram(Uint32 address,Byte *buffer,Uint16 size);
void InitFileSystem();
int16 Open(Uint16 id,Uint16 flag);
int16 Read(Uint16 file,Uint16 *pbuf,Uint16 len);
int16 Write(Uint16 file,Uint16 *pbuf,Uint16 len);
int16 Close(Uint16 file);
int16 CloseAll();

//sensor feedback interface
struct Sensors
{
	int16 incline[2];
};
Uint16 InitSensor();
Uint16 ReadADCResult(Uint16 *pbuf,Uint16 size);
Uint16 HandleSensor();
Uint16 GetSensorResult(struct Sensors *psensor);

//gait manage interface
struct GaitEffect
{
	int16 xOffset;
	int16 yOffset;
	int16 thetaOffset;
};
typedef struct DownloadGaitHead
{
	Uint16 gaitID;
	Uint16 frameLength;
	Uint16 frameStep;
	Uint16 period;
	struct GaitEffect effect;	
}DownloadGaitHead;

#define MAX_ROBOT_JOINTS			32
#define DEFAULT_TURNING_SPEED		128
Uint16 ExecuteConfiguration(Uint16 *pconfig,Uint16 len);
Uint16 SmoothConfiguration(Uint16 *pconfig,Uint16 len);
Uint16 ExecuteGait(Uint16 id,Uint16 times);
Uint16 ExecuteGaitDirectionExt(struct GaitEffect *peffect);

//fast math calculation
extern const int16 SIN_TABLE[];
extern const int16 ASIN_TABLE[];
#define PRECISION_BIT		9
#define LENGTH_BIT_LIMIT	10
#define PI					1608
#define ARCSIN(a) 		((a)<0?(-ASIN_TABLE[-(a)]):(ASIN_TABLE[a]))
#define ARCCOS(a) 		((a)<0?(PI/2+ASIN_TABLE[-(a)]):(PI/2-ASIN_TABLE[a]))
int16 SQRT16(Uint32 M);
int16 ARCTAN2(int16 y,int16 x);
int16 SIN(int16 a);
int16 COS(int16 a);

//fast inverse kinematics related
struct RigidOffset
{
	int16 x;
	int16 y;
	int16 z;
};

struct RigidPose
{
	int16 alpha;
	int16 beta;
	int16 theta;
};

struct RigidBody
{
	struct RigidOffset offset;
	struct RigidPose pose;
};

struct LegParameter
{
	int16 legOffset;
	int16 hipHight;
	int16 thigh;
	int16 crus;
	int16 foot;
};


struct WalkingConfig
{
	int16 ystage;
	int16 xswing;
	int16 yswing;
	int16 zswing;
	int16 alphaswing;
	int16 betaswing;
	int16 thetaswing;
	int16 xlanding;
	int16 ylanding;
	int16 zlanding;
	int16 alphalanding;
	int16 betalanding;
	int16 thetalanding;
	int16 xpercentage;
	int16 ypercentage;
	int16 thetapercentage;
	int16 elbowswing;
	int16 armfly;
	int16 armswing;
	int16 maxTurningInc;
	int16 maxTurningAngle;
	int16 acceleration;
	int16 maxTurningRadius;
	int16 maxStepRadius;
	int16 planningStableCycle;
};

Uint16 GetInverseKine(struct RigidBody *pankle, struct RigidBody *phip, struct LegParameter *pleg,int16 *result);
*/


#endif


