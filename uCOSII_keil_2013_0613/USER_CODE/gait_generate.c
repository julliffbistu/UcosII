#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "COM_packet.h"
#include "main.h"

#define MAX_ROBOT_JOINTS			32
#define GAIT_BUFFER_QUEUE_LENGTH	0x400
#define GAIT_ID_SINGLE_CONFIG		0xfffe
#define GAIT_ID_GAIT_DIRECTION		254
#define INVALID_GAIT_ID				0x0000
//#define SERVO_SPEED_CONSTANT	150
#define SERVO_SPEED_CONSTANT	73
//#define SERVO_SPEED_CONSTANT	43
#define SPEED_DEGRADE_CONST		0

#define COMMAND_SINGLE_CONFIGURATION	0x0001
#define COMMAND_SPECIAL_GAIT			0x0002
#define COMMAND_GAIT_DIRECTION			0x0004
#define COMMAND_GAIT_RESET				0x0008
#define COMMAND_SPECIAL_GAIT_QUERY		0x0010
#define COMMAND_WALK_KICK				0x0020

struct MechanicalZeroPoint
{
	Uint16 joints;
	Uint16 buf[MAX_ROBOT_JOINTS];
};

struct SingleFrameCommand
{
	Uint16 size;
	Uint16 nextStep;
	Uint16 frameBuffer[MAX_ROBOT_JOINTS*2];
};

struct GaitDirectionCommand
{
	Uint16 lastFoot;
	Uint16 ystage;
	Uint16 standingCntr;
	Uint32 stableClock;
	struct GaitEffect lastFootDir;
	struct GaitEffect thisFootDir;
	struct GaitEffect footMesuredSpeed;
	struct GaitEffect instEffect;
	struct GaitEffect lastHipDir;
	struct GaitEffect thisHipDir;
	struct GaitEffect cmdEffect;
};

struct WalkKickCmd
{
	Uint16 isLeft;
	struct GaitEffect cmdEffect;
};

struct RunningOdometer
{
	struct GaitEffect footLandingOdometer;
	struct GaitEffect hipSwingOdometer;
};

typedef DownloadGaitHead GaitHead;

struct GaitBufferQueue
{
	Uint16 f;
	Uint16 r;
	Uint16 q[GAIT_BUFFER_QUEUE_LENGTH];
};

#define GaitBufferQueueLen(pq) 		(((pq)->r-(pq)->f+GAIT_BUFFER_QUEUE_LENGTH)%GAIT_BUFFER_QUEUE_LENGTH)
#define ClearGaitBufferQueue(pq) 	{((pq)->f=0);((pq)->r=0);}
		
extern OS_EVENT *psemGaitGen;
extern OS_EVENT *psemGaitExe;
extern OS_EVENT *plckUpdateCommand;
extern OS_EVENT *plckGaitQueue;

struct HeadMovingCommand headMovingCommand;
struct SingleFrameCommand singleFrameCommand;
struct SpecialGaitCommand specialGaitCommand;
struct GaitDirectionCommand gDC;
struct WalkKickCmd walkKickCmd;
Uint16 commandState;
Uint16 priorityState;

struct RunningOdometer runningOdometer;

struct GaitBufferQueue gaitBufferQueue;
GaitHead currentGaitHead;
Uint16 gaitStepCntr;

struct MechanicalZeroPoint mechZeroPoint;
struct WalkingConfig walkingConfig;
struct RigidBody ankle;
struct RigidBody hip;
struct LegParameter leg;

void InitGaitGeneration(void)
{
	gaitBufferQueue.f=0;
	gaitBufferQueue.r=0;
	gaitStepCntr=0;
	memset(&currentGaitHead,0,sizeof(currentGaitHead));
	memset(&singleFrameCommand,0,sizeof(singleFrameCommand));
	memset(&specialGaitCommand,0,sizeof(specialGaitCommand));
	memset(&gDC,0,sizeof(gDC));
	memset(&runningOdometer,0,sizeof(runningOdometer));
	memset(&mechZeroPoint,0,sizeof(mechZeroPoint));
	memset(&headMovingCommand,0,sizeof(headMovingCommand));
	memset(&walkKickCmd,0,sizeof(walkKickCmd));
	//gDC.lastFoot=1;
	commandState=0;
	priorityState=0;
}

Uint16 AddGaitData(Uint16 size,Uint16 *pbuf)
{
	Uint16 len=0;
	Uint16 i=0;
	Uint16 *pdata=NULL;
	INT8U err=0;
	
	OSMutexPend(plckGaitQueue,0,&err);
	len=GAIT_BUFFER_QUEUE_LENGTH-GaitBufferQueueLen(&gaitBufferQueue);
	while(size>len)
	{
		//halt here until space available
		while(OSSemAccept(psemGaitExe)!=0);
		OSMutexPost(plckGaitQueue);
		OSSemPend(psemGaitExe,0,&err);
		OSMutexPend(plckGaitQueue,0,&err);
		len=GAIT_BUFFER_QUEUE_LENGTH-GaitBufferQueueLen(&gaitBufferQueue);
	}

	len=size;
	pdata=pbuf;
	for(i=0;i<len;i++)
	{
		gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
		gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
	}
	EnableTimerInterrupt(TRUE);
	OSMutexPost(plckGaitQueue);
	return 0;
}

Uint16 AddGaitDataWithPose(Uint16 size,Uint16 *pbuf,struct RigidBody *pankle, struct RigidBody *phip,Uint16 isLeft)
{
	Uint16 len=0;
	Uint16 i=0;
	Uint16 *pdata=NULL;
	INT8U err=0;
	Uint16 realLength=0;

	realLength=size+1;
	if(pankle!=NULL)
		realLength+=sizeof(struct RigidBody)/sizeof(Uint16);
	if(phip!=NULL)
		realLength+=sizeof(struct RigidBody)/sizeof(Uint16);
		
	OSMutexPend(plckGaitQueue,0,&err);
	len=GAIT_BUFFER_QUEUE_LENGTH-GaitBufferQueueLen(&gaitBufferQueue);
	while(realLength>len)
	{
		//halt here until space available
		while(OSSemAccept(psemGaitExe)!=0);
		OSMutexPost(plckGaitQueue);
		OSSemPend(psemGaitExe,0,&err);
		OSMutexPend(plckGaitQueue,0,&err);
		len=GAIT_BUFFER_QUEUE_LENGTH-GaitBufferQueueLen(&gaitBufferQueue);
	}

	len=size;
	pdata=pbuf;
	for(i=0;i<len;i++)
	{
		gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
		gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
	}

	//add left or right step to the queue
	{
		gaitBufferQueue.q[gaitBufferQueue.r]=isLeft;
		gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
	}

	if(pankle!=NULL)
	{
		len=sizeof(struct RigidBody)/sizeof(Uint16);
		pdata=(Uint16 *)pankle;
		for(i=0;i<len;i++)
		{
			gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
			gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
		}
	}
	if(phip!=NULL)
	{
		len=sizeof(struct RigidBody)/sizeof(Uint16);
		pdata=(Uint16 *)phip;
		for(i=0;i<len;i++)
		{
			gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
			gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
		}
	}
	EnableTimerInterrupt(TRUE);
	OSMutexPost(plckGaitQueue);
	return 0;
}

Uint16 WaitUntilQueueLen(Uint16 tgtLen)
{
	INT8U err=0;
	OSMutexPend(plckGaitQueue,0,&err);
	while(tgtLen<GaitBufferQueueLen(&gaitBufferQueue))
	{
		//halt here until space available
		while(OSSemAccept(psemGaitExe)!=0);
		OSMutexPost(plckGaitQueue);
		OSSemPend(psemGaitExe,0,&err);
		OSMutexPend(plckGaitQueue,0,&err);
	}
	OSMutexPost(plckGaitQueue);
	return 0;
}

Uint16 AddNewGaitWithPose(GaitHead *phead,Uint16 *pbuf,struct RigidBody *pankle, struct RigidBody *phip,Uint16 isLeft)
{
	Uint16 len=0;
	Uint16 i=0;
	Uint16 *pdata=NULL;
	INT8U err=0;
	Uint16 realLength=0;

	realLength=phead->frameStep+1;
	if(pankle!=NULL)
	{
		realLength+=sizeof(struct RigidBody)/sizeof(Uint16);
		phead->frameStep|=0x8000;
	}
	if(phip!=NULL)
	{
		realLength+=sizeof(struct RigidBody)/sizeof(Uint16);
		phead->frameStep|=0x4000;
	}

	OSMutexPend(plckGaitQueue,0,&err);
	len=GAIT_BUFFER_QUEUE_LENGTH-GaitBufferQueueLen(&gaitBufferQueue);
	while(realLength+sizeof(GaitHead)/sizeof(Uint16)>len)
	{
		//halt here until space available
		while(OSSemAccept(psemGaitExe)!=0);
		OSMutexPost(plckGaitQueue);
		OSSemPend(psemGaitExe,0,&err);
		OSMutexPend(plckGaitQueue,0,&err);
		len=GAIT_BUFFER_QUEUE_LENGTH-GaitBufferQueueLen(&gaitBufferQueue);
	}

	len=sizeof(GaitHead)/sizeof(Uint16);
	pdata=(Uint16 *)phead;
	for(i=0;i<len;i++)
	{
		gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
		gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
	}
	len=(phead->frameStep&(~0xC000));
	pdata=pbuf;
	for(i=0;i<len;i++)
	{
		gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
		gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
	}
	
	//add left or right step to the queue
	{
		gaitBufferQueue.q[gaitBufferQueue.r]=isLeft;
		gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
	}

	if(phead->frameStep&0x8000)
	{
		len=sizeof(struct RigidBody)/sizeof(Uint16);
		pdata=(Uint16 *)pankle;
		for(i=0;i<len;i++)
		{
			gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
			gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
		}
	}
	if(phead->frameStep&0x4000)
	{
		len=sizeof(struct RigidBody)/sizeof(Uint16);
		pdata=(Uint16 *)phip;
		for(i=0;i<len;i++)
		{
			gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
			gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
		}
	}
	phead->frameStep&=~0xC000;
	EnableTimerInterrupt(TRUE);
	OSMutexPost(plckGaitQueue);
	return 0;
}

Uint16 AddNewGait(GaitHead *phead,Uint16 *pbuf)
{
	Uint16 len=0;
	Uint16 i=0;
	Uint16 *pdata=NULL;
	INT8U err=0;

	OSMutexPend(plckGaitQueue,0,&err);
	len=GAIT_BUFFER_QUEUE_LENGTH-GaitBufferQueueLen(&gaitBufferQueue);
	while(phead->frameStep+sizeof(GaitHead)/sizeof(Uint16)>len)
	{
		//halt here until space available
		while(OSSemAccept(psemGaitExe)!=0);
		OSMutexPost(plckGaitQueue);
		OSSemPend(psemGaitExe,0,&err);
		OSMutexPend(plckGaitQueue,0,&err);
		len=GAIT_BUFFER_QUEUE_LENGTH-GaitBufferQueueLen(&gaitBufferQueue);
	}

	len=sizeof(GaitHead)/sizeof(Uint16);
	pdata=(Uint16 *)phead;
	for(i=0;i<len;i++)
	{
		gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
		gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
	}
	len=phead->frameStep;
	pdata=pbuf;
	for(i=0;i<len;i++)
	{
		gaitBufferQueue.q[gaitBufferQueue.r]=pdata[i];
		gaitBufferQueue.r=(gaitBufferQueue.r+1)%GAIT_BUFFER_QUEUE_LENGTH;
	}
	EnableTimerInterrupt(TRUE);
	OSMutexPost(plckGaitQueue);
	return 0;
}

Uint16 ModifyToTurnningAngle(int16 *presult)
{
//	this is the mos2007 robot configuration
/*
	presult[0]=-presult[0];
	presult[3]=-presult[3];
	presult[4]=-presult[4];
	
	presult[7]=-presult[7];
	presult[9]=-presult[9];
	presult[10]=-presult[10];
	presult[11]=-presult[11];

	presult[15]=-presult[15];
*/
//	this is the mos-strong mos-lite robots configuration
	presult[0]=-presult[0];
	presult[1]=-presult[1];
	presult[4]=-presult[4];
	presult[5]=-presult[5];

	presult[6]=-presult[6];	
	presult[7]=-presult[7];
	presult[8]=-presult[8];
	presult[9]=-presult[9];
	presult[11]=-presult[11];

	presult[14]=-presult[14];
	presult[15]=-presult[15];
	presult[17]=-presult[17];	
	return 0;
}

Uint16 ExecuteWalkKick()
{
	if(gDC.lastFoot==0)					//this step will be left when gDC.lastFoot==0
	{
		if(walkKickCmd.cmdEffect.yOffset>=0)
		{
			//possibly direct kick action
			gDC.thisFootDir.yOffset=walkKickCmd.cmdEffect.yOffset;
			//gDC.thisHipDir.yOffset=(gDC.thisFootDir.yOffset>>1);
			gDC.thisHipDir.yOffset=(int16)((
				((int32)gDC.thisFootDir.yOffset)*
				walkingConfig.ypercentage
				)>>PRECISION_BIT);
			gDC.footMesuredSpeed.yOffset=gDC.thisFootDir.yOffset;
		}
		else
		{
			gDC.thisFootDir.yOffset=0;
			gDC.thisHipDir.yOffset=0;
		}

		if(walkKickCmd.cmdEffect.thetaOffset>=0)
		{
			gDC.thisFootDir.thetaOffset=walkKickCmd.cmdEffect.thetaOffset;
			//gDC.thisHipDir.thetaOffset=(gDC.thisFootDir.thetaOffset>>1);
			gDC.thisHipDir.thetaOffset=(int16)((
				((int32)gDC.thisFootDir.thetaOffset)*
				walkingConfig.thetapercentage
				)>>PRECISION_BIT);
			gDC.footMesuredSpeed.thetaOffset=gDC.thisFootDir.thetaOffset;
		}else								//no instruction then zero command is taken
		{
			//possibly next kick action
			gDC.thisFootDir.thetaOffset=0;
			gDC.thisHipDir.thetaOffset=0;
		}
	}else
	{
		if(walkKickCmd.cmdEffect.yOffset<=0)
		{
			//possibly direct kick action
			gDC.thisFootDir.yOffset=walkKickCmd.cmdEffect.yOffset;
			//gDC.thisHipDir.yOffset=(gDC.thisFootDir.yOffset>>1);
			gDC.thisHipDir.yOffset=(int16)((
				((int32)gDC.thisFootDir.yOffset)*
				walkingConfig.ypercentage
				)>>PRECISION_BIT);
			gDC.footMesuredSpeed.yOffset=gDC.thisFootDir.yOffset;
		}
		else
		{
			gDC.thisFootDir.yOffset=0;
			gDC.thisHipDir.yOffset=0;
		}

		if(walkKickCmd.cmdEffect.thetaOffset<=0)
		{
			gDC.thisFootDir.thetaOffset=walkKickCmd.cmdEffect.thetaOffset;
			//gDC.thisHipDir.thetaOffset=(gDC.thisFootDir.thetaOffset>>1);
			gDC.thisHipDir.thetaOffset=(int16)((
				((int32)gDC.thisFootDir.thetaOffset)*
				walkingConfig.thetapercentage
				)>>PRECISION_BIT);
			gDC.footMesuredSpeed.thetaOffset=gDC.thisFootDir.thetaOffset;
		}else								//no instruction then zero command is taken
		{
			//possibly next kick action
			gDC.thisFootDir.thetaOffset=0;
			gDC.thisHipDir.thetaOffset=0;
		}
	}

	gDC.thisFootDir.xOffset=walkKickCmd.cmdEffect.xOffset;
	gDC.footMesuredSpeed.xOffset=gDC.thisFootDir.xOffset;		
	gDC.thisHipDir.xOffset=
	(int16)((
		((int32)gDC.thisFootDir.xOffset)*
		walkingConfig.xpercentage
	)>>PRECISION_BIT);
	return 0;
}

Uint16 WalkKickControlStep()
{
	//firstly check if the kick step is correspondant with current step	
	if(gDC.lastFoot==0)					//this step will be left when gDC.lastFoot==0
	{
		if(walkKickCmd.isLeft)
		{
			if(gDC.lastFootDir.yOffset>0)		//last right step is right then this step should be zero
			{
				//completion of the next step
				if(walkKickCmd.cmdEffect.yOffset!=0)
				{
					gDC.thisFootDir.yOffset=0;
					gDC.thisHipDir.yOffset=0;
					gDC.thisFootDir.thetaOffset=0;
					gDC.thisHipDir.thetaOffset=0;
					gDC.thisFootDir.xOffset=0;
					gDC.thisHipDir.xOffset=0;					
					//current step should be dropped for the next control step
					//reset to mark time steps
					//return for next querry
					gDC.footMesuredSpeed.xOffset=0;
					gDC.footMesuredSpeed.yOffset=0;
					gDC.footMesuredSpeed.thetaOffset=0;
					return 0;
				}

				//possibly direct kick action
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
			}else if(walkKickCmd.cmdEffect.yOffset>=0)	//last right step is not right but the instruction is steping left then take the instruction into account
			{
				//possibly direct kick action
				gDC.thisFootDir.yOffset=walkKickCmd.cmdEffect.yOffset;
				//gDC.thisHipDir.yOffset=(gDC.thisFootDir.yOffset>>1);
				gDC.thisHipDir.yOffset=(int16)((
					((int32)gDC.thisFootDir.yOffset)*
					walkingConfig.ypercentage
					)>>PRECISION_BIT);
				gDC.footMesuredSpeed.yOffset=gDC.thisFootDir.yOffset;
			}else if(walkKickCmd.cmdEffect.yOffset<0)
			{
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
				gDC.thisFootDir.xOffset=0;
				gDC.thisHipDir.xOffset=0;					
				//current step should be dropped for the next control step
				//reset to mark time steps
				//return for next querry
				gDC.footMesuredSpeed.xOffset=0;
				gDC.footMesuredSpeed.yOffset=0;
				gDC.footMesuredSpeed.thetaOffset=0;
				return 0;
			}				
			
			if(gDC.lastFootDir.thetaOffset>0)
			{
				//completion of the next step
				if(walkKickCmd.cmdEffect.thetaOffset!=0)
				{
					gDC.thisFootDir.yOffset=0;
					gDC.thisHipDir.yOffset=0;
					gDC.thisFootDir.thetaOffset=0;
					gDC.thisHipDir.thetaOffset=0;
					gDC.thisFootDir.xOffset=0;
					gDC.thisHipDir.xOffset=0;					
					//current step should be dropped for the next control step
					//reset to mark time steps
					//return for next querry
					gDC.footMesuredSpeed.xOffset=0;
					gDC.footMesuredSpeed.yOffset=0;
					gDC.footMesuredSpeed.thetaOffset=0;
					return 0;
				}
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
			}else if(walkKickCmd.cmdEffect.thetaOffset>=0)
			{
				gDC.thisFootDir.thetaOffset=walkKickCmd.cmdEffect.thetaOffset;
				//gDC.thisHipDir.thetaOffset=(gDC.thisFootDir.thetaOffset>>1);
				gDC.thisHipDir.thetaOffset=(int16)((
					((int32)gDC.thisFootDir.thetaOffset)*
					walkingConfig.thetapercentage
					)>>PRECISION_BIT);
				gDC.footMesuredSpeed.thetaOffset=gDC.thisFootDir.thetaOffset;
			}else if(walkKickCmd.cmdEffect.thetaOffset<0)
			{
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
				gDC.thisFootDir.xOffset=0;
				gDC.thisHipDir.xOffset=0;					
				//current step should be dropped for the next control step
				//reset to mark time steps
				//return for next querry
				gDC.footMesuredSpeed.xOffset=0;
				gDC.footMesuredSpeed.yOffset=0;
				gDC.footMesuredSpeed.thetaOffset=0;
				return 0;
			}

			//get current foot x placement according to last foot x and inst
			if(gDC.lastFootDir.xOffset*walkKickCmd.cmdEffect.xOffset>0)
			{
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
				gDC.thisFootDir.xOffset=0;
				gDC.thisHipDir.xOffset=0;					
				//current step should be dropped for the next control step
				//reset to mark time steps
				//return for next querry
				gDC.footMesuredSpeed.xOffset=0;
				gDC.footMesuredSpeed.yOffset=0;
				gDC.footMesuredSpeed.thetaOffset=0;
				return 0;
			}else
			{
				//accept the kick step
				gDC.thisFootDir.xOffset=walkKickCmd.cmdEffect.xOffset;
			}
			
			gDC.footMesuredSpeed.xOffset=gDC.thisFootDir.xOffset;
		
			gDC.thisHipDir.xOffset=
			(int16)((
				((int32)gDC.thisFootDir.xOffset)*
				walkingConfig.xpercentage
			)>>PRECISION_BIT);

			//single step mode check passed and direct kick can be issued here
			return 2;
		}else
		{
			if(gDC.lastFootDir.yOffset>0)		//last right step is right then this step should be zero
			{
				//completion of the next step
				if(walkKickCmd.cmdEffect.yOffset>0)
				{
					gDC.thisFootDir.yOffset=0;
					gDC.thisHipDir.yOffset=0;
					gDC.thisFootDir.thetaOffset=0;
					gDC.thisHipDir.thetaOffset=0;
					gDC.thisFootDir.xOffset=0;
					gDC.thisHipDir.xOffset=0;					
					//current step should be dropped for the next control step
					//reset to mark time steps
					//return for next querry
					gDC.footMesuredSpeed.xOffset=0;
					gDC.footMesuredSpeed.yOffset=0;
					gDC.footMesuredSpeed.thetaOffset=0;
					return 0;
				}

				//possibly next step kick action
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
			}else if(walkKickCmd.cmdEffect.yOffset>=0)	//last right step is not right but the instruction is steping left then take the instruction into account
			{
				//possibly next kick action
				gDC.thisFootDir.yOffset=walkKickCmd.cmdEffect.yOffset;
				//gDC.thisHipDir.yOffset=(gDC.thisFootDir.yOffset>>1);
				gDC.thisHipDir.yOffset=(int16)((
					((int32)gDC.thisFootDir.yOffset)*
					walkingConfig.ypercentage
					)>>PRECISION_BIT);
				gDC.footMesuredSpeed.yOffset=gDC.thisFootDir.yOffset;
			}else								//no instruction then zero command is taken
			{
				//possibly next kick action
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
			}				
			
			if(gDC.lastFootDir.thetaOffset>0)
			{
				//completion of the next step
				if(walkKickCmd.cmdEffect.thetaOffset>0)
				{
					gDC.thisFootDir.yOffset=0;
					gDC.thisHipDir.yOffset=0;
					gDC.thisFootDir.thetaOffset=0;
					gDC.thisHipDir.thetaOffset=0;
					gDC.thisFootDir.xOffset=0;
					gDC.thisHipDir.xOffset=0;					
					//current step should be dropped for the next control step
					//reset to mark time steps
					//return for next querry
					gDC.footMesuredSpeed.xOffset=0;
					gDC.footMesuredSpeed.yOffset=0;
					gDC.footMesuredSpeed.thetaOffset=0;
					return 0;
				}
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
			}else if(walkKickCmd.cmdEffect.thetaOffset>=0)
			{
				gDC.thisFootDir.thetaOffset=walkKickCmd.cmdEffect.thetaOffset;
				//gDC.thisHipDir.thetaOffset=(gDC.thisFootDir.thetaOffset>>1);
				gDC.thisHipDir.thetaOffset=(int16)((
					((int32)gDC.thisFootDir.thetaOffset)*
					walkingConfig.thetapercentage
					)>>PRECISION_BIT);
				gDC.footMesuredSpeed.thetaOffset=gDC.thisFootDir.thetaOffset;
			}else
			{
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
			}

			//get current foot x placement according to last foot x and inst
			gDC.thisFootDir.xOffset=0;
			
			gDC.footMesuredSpeed.xOffset=gDC.thisFootDir.xOffset;
		
			gDC.thisHipDir.xOffset=
			(int16)((
				((int32)gDC.thisFootDir.xOffset)*
				walkingConfig.xpercentage
			)>>PRECISION_BIT);

			//double step mode check passed and next kick can be issued here
			return 1;			
		}
	}else								//this step will be right when gDC.lastfoot=1
	{
		if(!walkKickCmd.isLeft)
		{
			if(gDC.lastFootDir.yOffset<0)		//last right step is right then this step should be zero
			{
				//completion of the next step
				if(walkKickCmd.cmdEffect.yOffset!=0)
				{
					gDC.thisFootDir.yOffset=0;
					gDC.thisHipDir.yOffset=0;
					gDC.thisFootDir.thetaOffset=0;
					gDC.thisHipDir.thetaOffset=0;
					gDC.thisFootDir.xOffset=0;
					gDC.thisHipDir.xOffset=0;					
					//current step should be dropped for the next control step
					//reset to mark time steps
					//return for next querry
					gDC.footMesuredSpeed.xOffset=0;
					gDC.footMesuredSpeed.yOffset=0;
					gDC.footMesuredSpeed.thetaOffset=0;
					return 0;
				}

				//possibly direct kick action
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
			}else if(walkKickCmd.cmdEffect.yOffset<=0)	//last right step is not right but the instruction is steping left then take the instruction into account
			{
				//possibly direct kick action
				gDC.thisFootDir.yOffset=walkKickCmd.cmdEffect.yOffset;
				//gDC.thisHipDir.yOffset=(gDC.thisFootDir.yOffset>>1);
				gDC.thisHipDir.yOffset=(int16)((
					((int32)gDC.thisFootDir.yOffset)*
					walkingConfig.ypercentage
					)>>PRECISION_BIT);
				gDC.footMesuredSpeed.yOffset=gDC.thisFootDir.yOffset;
			}else if(walkKickCmd.cmdEffect.yOffset>0)
			{
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
				gDC.thisFootDir.xOffset=0;
				gDC.thisHipDir.xOffset=0;					
				//current step should be dropped for the next control step
				//reset to mark time steps
				//return for next querry
				gDC.footMesuredSpeed.xOffset=0;
				gDC.footMesuredSpeed.yOffset=0;
				gDC.footMesuredSpeed.thetaOffset=0;
				return 0;
			}				
			
			if(gDC.lastFootDir.thetaOffset<0)
			{
				//completion of the next step
				if(walkKickCmd.cmdEffect.thetaOffset!=0)
				{
					gDC.thisFootDir.yOffset=0;
					gDC.thisHipDir.yOffset=0;
					gDC.thisFootDir.thetaOffset=0;
					gDC.thisHipDir.thetaOffset=0;
					gDC.thisFootDir.xOffset=0;
					gDC.thisHipDir.xOffset=0;					
					//current step should be dropped for the next control step
					//reset to mark time steps
					//return for next querry
					gDC.footMesuredSpeed.xOffset=0;
					gDC.footMesuredSpeed.yOffset=0;
					gDC.footMesuredSpeed.thetaOffset=0;
					return 0;
				}
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
			}else if(walkKickCmd.cmdEffect.thetaOffset<=0)
			{
				gDC.thisFootDir.thetaOffset=walkKickCmd.cmdEffect.thetaOffset;
				//gDC.thisHipDir.thetaOffset=(gDC.thisFootDir.thetaOffset>>1);
				gDC.thisHipDir.thetaOffset=(int16)((
					((int32)gDC.thisFootDir.thetaOffset)*
					walkingConfig.thetapercentage
					)>>PRECISION_BIT);
				gDC.footMesuredSpeed.thetaOffset=gDC.thisFootDir.thetaOffset;
			}else if(walkKickCmd.cmdEffect.thetaOffset>0)
			{
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
				gDC.thisFootDir.xOffset=0;
				gDC.thisHipDir.xOffset=0;					
				//current step should be dropped for the next control step
				//reset to mark time steps
				//return for next querry
				gDC.footMesuredSpeed.xOffset=0;
				gDC.footMesuredSpeed.yOffset=0;
				gDC.footMesuredSpeed.thetaOffset=0;
				return 0;
			}

			//get current foot x placement according to last foot x and inst
			if(gDC.lastFootDir.xOffset*walkKickCmd.cmdEffect.xOffset>0)
			{
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
				gDC.thisFootDir.xOffset=0;
				gDC.thisHipDir.xOffset=0;					
				//current step should be dropped for the next control step
				//reset to mark time steps
				//return for next querry
				gDC.footMesuredSpeed.xOffset=0;
				gDC.footMesuredSpeed.yOffset=0;
				gDC.footMesuredSpeed.thetaOffset=0;
				return 0;
			}else
			{
				//accept the kick step
				gDC.thisFootDir.xOffset=walkKickCmd.cmdEffect.xOffset;
			}
			
			gDC.footMesuredSpeed.xOffset=gDC.thisFootDir.xOffset;
		
			gDC.thisHipDir.xOffset=
			(int16)((
				((int32)gDC.thisFootDir.xOffset)*
				walkingConfig.xpercentage
			)>>PRECISION_BIT);

			//single step mode check passed and direct kick can be issued here
			return 2;
		}else
		{
			if(gDC.lastFootDir.yOffset<0)		//last right step is right then this step should be zero
			{
				//completion of the next step
				if(walkKickCmd.cmdEffect.yOffset<0)
				{
					gDC.thisFootDir.yOffset=0;
					gDC.thisHipDir.yOffset=0;
					gDC.thisFootDir.thetaOffset=0;
					gDC.thisHipDir.thetaOffset=0;
					gDC.thisFootDir.xOffset=0;
					gDC.thisHipDir.xOffset=0;					
					//current step should be dropped for the next control step
					//reset to mark time steps
					//return for next querry
					gDC.footMesuredSpeed.xOffset=0;
					gDC.footMesuredSpeed.yOffset=0;
					gDC.footMesuredSpeed.thetaOffset=0;
					return 0;
				}

				//possibly next step kick action
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
			}else if(walkKickCmd.cmdEffect.yOffset<=0)	//last right step is not right but the instruction is steping left then take the instruction into account
			{
				//possibly next kick action
				gDC.thisFootDir.yOffset=walkKickCmd.cmdEffect.yOffset;
				//gDC.thisHipDir.yOffset=(gDC.thisFootDir.yOffset>>1);
				gDC.thisHipDir.yOffset=(int16)((
					((int32)gDC.thisFootDir.yOffset)*
					walkingConfig.ypercentage
					)>>PRECISION_BIT);
				gDC.footMesuredSpeed.yOffset=gDC.thisFootDir.yOffset;
			}else								//no instruction then zero command is taken
			{
				//possibly next kick action
				gDC.thisFootDir.yOffset=0;
				gDC.thisHipDir.yOffset=0;
			}				
			
			if(gDC.lastFootDir.thetaOffset<0)
			{
				//completion of the next step
				if(walkKickCmd.cmdEffect.thetaOffset<0)
				{
					gDC.thisFootDir.yOffset=0;
					gDC.thisHipDir.yOffset=0;
					gDC.thisFootDir.thetaOffset=0;
					gDC.thisHipDir.thetaOffset=0;
					gDC.thisFootDir.xOffset=0;
					gDC.thisHipDir.xOffset=0;					
					//current step should be dropped for the next control step
					//reset to mark time steps
					//return for next querry
					gDC.footMesuredSpeed.xOffset=0;
					gDC.footMesuredSpeed.yOffset=0;
					gDC.footMesuredSpeed.thetaOffset=0;
					return 0;
				}
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
			}else if(walkKickCmd.cmdEffect.thetaOffset<=0)
			{
				gDC.thisFootDir.thetaOffset=walkKickCmd.cmdEffect.thetaOffset;
				//gDC.thisHipDir.thetaOffset=(gDC.thisFootDir.thetaOffset>>1);
				gDC.thisHipDir.thetaOffset=(int16)((
					((int32)gDC.thisFootDir.thetaOffset)*
					walkingConfig.thetapercentage
					)>>PRECISION_BIT);
				gDC.footMesuredSpeed.thetaOffset=gDC.thisFootDir.thetaOffset;
			}else
			{
				gDC.thisFootDir.thetaOffset=0;
				gDC.thisHipDir.thetaOffset=0;
			}

			//get current foot x placement according to last foot x and inst
			gDC.thisFootDir.xOffset=0;
			
			gDC.footMesuredSpeed.xOffset=gDC.thisFootDir.xOffset;
		
			gDC.thisHipDir.xOffset=
			(int16)((
				((int32)gDC.thisFootDir.xOffset)*
				walkingConfig.xpercentage
			)>>PRECISION_BIT);

			//double step mode check passed and next kick can be issued here
			return 1;			
		}	
	}
	
	return 0;
}

Uint16 WalkingSpeedControl()
{
	int16 turnningCircle=0;
	int32 temp32;
	int16 temp1;
	int16 temp2;
	//only direct copy is applied for test
	//complex speed control is in need here
	memcpy(
		&gDC.instEffect,
		&gDC.cmdEffect,
		sizeof(struct GaitEffect)
		);
		
	if(gDC.instEffect.thetaOffset>walkingConfig.maxTurningAngle)
		gDC.instEffect.thetaOffset=walkingConfig.maxTurningAngle;
	else if(gDC.instEffect.thetaOffset<-walkingConfig.maxTurningAngle)
		gDC.instEffect.thetaOffset=-walkingConfig.maxTurningAngle;
	
	if(gDC.instEffect.thetaOffset>walkingConfig.maxTurningInc+gDC.footMesuredSpeed.thetaOffset)
		gDC.instEffect.thetaOffset=gDC.footMesuredSpeed.thetaOffset+walkingConfig.maxTurningInc;
	else if(gDC.instEffect.thetaOffset+walkingConfig.maxTurningInc<gDC.footMesuredSpeed.thetaOffset)
		gDC.instEffect.thetaOffset=gDC.footMesuredSpeed.thetaOffset-walkingConfig.maxTurningInc;
	
	if(gDC.instEffect.thetaOffset>0)
		turnningCircle=(int16)((int32)walkingConfig.maxTurningRadius*
							((int32)walkingConfig.maxTurningAngle-gDC.instEffect.thetaOffset)/
							walkingConfig.maxTurningAngle);
	else if(gDC.instEffect.thetaOffset==0)
		turnningCircle=walkingConfig.maxStepRadius;
	else
		turnningCircle=(int16)((int32)walkingConfig.maxTurningRadius*
							((int32)walkingConfig.maxTurningAngle+gDC.instEffect.thetaOffset)/
							walkingConfig.maxTurningAngle);
	
	temp32=(int32)gDC.instEffect.xOffset*gDC.instEffect.xOffset+(int32)gDC.instEffect.yOffset*gDC.instEffect.yOffset;
	if(temp32>(int32)turnningCircle*turnningCircle)
	{
		temp1=(int16)((int32)turnningCircle*gDC.instEffect.xOffset/(int32)SQRT16(temp32));
		temp2=(int16)((int32)turnningCircle*gDC.instEffect.yOffset/(int32)SQRT16(temp32));
		gDC.instEffect.xOffset=temp1;
		gDC.instEffect.yOffset=temp2;
	}
	
	temp32=((int32)gDC.footMesuredSpeed.xOffset-gDC.instEffect.xOffset)*
			((int32)gDC.footMesuredSpeed.xOffset-gDC.instEffect.xOffset);
	temp32+=((int32)gDC.footMesuredSpeed.yOffset-gDC.instEffect.yOffset)*
			((int32)gDC.footMesuredSpeed.yOffset-gDC.instEffect.yOffset);
	temp32=SQRT16(temp32);
	
	if(temp32>walkingConfig.acceleration)
	{
		gDC.instEffect.xOffset=(int16)(((int32)gDC.instEffect.xOffset-gDC.footMesuredSpeed.xOffset)*
									(int32)walkingConfig.acceleration/temp32)+
									gDC.footMesuredSpeed.xOffset;		
		gDC.instEffect.yOffset=(int16)(((int32)gDC.instEffect.yOffset-gDC.footMesuredSpeed.yOffset)*
									(int32)walkingConfig.acceleration/temp32)+
									gDC.footMesuredSpeed.yOffset;
		
		temp32=((int32)gDC.instEffect.xOffset*gDC.instEffect.xOffset)+
				((int32)gDC.instEffect.yOffset*gDC.instEffect.yOffset);
		temp32=SQRT16(temp32);
		
		if(temp32>turnningCircle)
		{
			if(temp32>walkingConfig.maxTurningRadius)
				temp1=0;
			else
			{
				temp1=(int16)(temp32*walkingConfig.maxTurningAngle/
						walkingConfig.maxTurningRadius);
				temp1=walkingConfig.maxTurningAngle-temp1;
			}
			
			if(gDC.instEffect.thetaOffset>gDC.footMesuredSpeed.thetaOffset)
				gDC.instEffect.thetaOffset=temp1;
			else
				gDC.instEffect.thetaOffset=-temp1;
		}
	}
	return 0;
}

Uint16 SingleStepControl()
{
	if(gDC.lastFoot==0)
	{
		if(gDC.lastFootDir.yOffset>0)
		{
			gDC.thisFootDir.yOffset=0;
			gDC.thisHipDir.yOffset=0;
		}else if(gDC.instEffect.yOffset>=0)
		{
			gDC.thisFootDir.yOffset=gDC.instEffect.yOffset;
			//gDC.thisHipDir.yOffset=(gDC.thisFootDir.yOffset>>1);
			gDC.thisHipDir.yOffset=(int16)((
				((int32)gDC.thisFootDir.yOffset)*
				walkingConfig.ypercentage
				)>>PRECISION_BIT);
			gDC.footMesuredSpeed.yOffset=gDC.thisFootDir.yOffset;
		}else
		{
			gDC.thisFootDir.yOffset=0;
			gDC.thisHipDir.yOffset=0;
		}				
		
		if(gDC.lastFootDir.thetaOffset>0)
		{
			gDC.thisFootDir.thetaOffset=0;
			gDC.thisHipDir.thetaOffset=0;
		}else if(gDC.instEffect.thetaOffset>=0)
		{
			gDC.thisFootDir.thetaOffset=gDC.instEffect.thetaOffset;
			//gDC.thisHipDir.thetaOffset=(gDC.thisFootDir.thetaOffset>>1);
			gDC.thisHipDir.thetaOffset=(int16)((
				((int32)gDC.thisFootDir.thetaOffset)*
				walkingConfig.thetapercentage
				)>>PRECISION_BIT);
			gDC.footMesuredSpeed.thetaOffset=gDC.thisFootDir.thetaOffset;
		}else
		{
			gDC.thisFootDir.thetaOffset=0;
			gDC.thisHipDir.thetaOffset=0;
		}
	}else
	{
		if(gDC.lastFootDir.yOffset<0)
		{
			gDC.thisFootDir.yOffset=0;
			gDC.thisHipDir.yOffset=0;
		}else if(gDC.instEffect.yOffset<=0)
		{
			gDC.thisFootDir.yOffset=gDC.instEffect.yOffset;
			//gDC.thisHipDir.yOffset=(gDC.thisFootDir.yOffset>>1);
			gDC.thisHipDir.yOffset=(int16)((
				((int32)gDC.thisFootDir.yOffset)*
				walkingConfig.ypercentage
				)>>PRECISION_BIT);
			gDC.footMesuredSpeed.yOffset=gDC.thisFootDir.yOffset;
		}else
		{
			gDC.thisFootDir.yOffset=0;
			gDC.thisHipDir.yOffset=0;
		}				
		
		if(gDC.lastFootDir.thetaOffset<0)
		{
			gDC.thisFootDir.thetaOffset=0;
			gDC.thisHipDir.thetaOffset=0;
		}else if(gDC.instEffect.thetaOffset<=0)
		{
			gDC.thisFootDir.thetaOffset=gDC.instEffect.thetaOffset;
			//gDC.thisHipDir.thetaOffset=(gDC.thisFootDir.thetaOffset>>1);
			gDC.thisHipDir.thetaOffset=(int16)((
				((int32)gDC.thisFootDir.thetaOffset)*
				walkingConfig.thetapercentage
				)>>PRECISION_BIT);
			gDC.footMesuredSpeed.thetaOffset=gDC.thisFootDir.thetaOffset;
		}else
		{
			gDC.thisFootDir.thetaOffset=0;
			gDC.thisHipDir.thetaOffset=0;
		}	
	}
	
	//get current foot x placement according to last foot x and inst
	if(gDC.lastFootDir.xOffset*gDC.instEffect.xOffset>0)
		gDC.thisFootDir.xOffset=0;
	else
		gDC.thisFootDir.xOffset=gDC.instEffect.xOffset;
	
	gDC.footMesuredSpeed.xOffset=gDC.thisFootDir.xOffset;
	//get current hip x placement according to current foot x
	/*
	if(gDC.thisFootDir.xOffset<0)
	{
		gDC.thisHipDir.xOffset=
			(int16)((
				((int32)gDC.thisFootDir.xOffset)*
				((1<<PRECISION_BIT)-
				walkingConfig.xpercentage)
			)>>PRECISION_BIT);
	}else
	{
		gDC.thisHipDir.xOffset=
			(int16)((
				((int32)gDC.thisFootDir.xOffset)*
				walkingConfig.xpercentage
			)>>PRECISION_BIT);
	}
	*/
	gDC.thisHipDir.xOffset=
	(int16)((
		((int32)gDC.thisFootDir.xOffset)*
		walkingConfig.xpercentage
	)>>PRECISION_BIT);
	
	return 0;
}

Uint16 FillInStepParameter(int16 *pframe,Uint16 len)
{
	int32 temp=0;
	
	leg.legOffset=abs(leg.legOffset);
	if(gDC.lastFoot==0)
	{	
		temp=(int32)pframe[0]*walkingConfig.xlanding;
		temp+=((((int32)gDC.lastFootDir.xOffset))<<PRECISION_BIT);
		temp+=-(((int32)leg.legOffset*SIN(gDC.lastFootDir.thetaOffset))>>1);
		temp+=((int32)gDC.thisFootDir.xOffset-(int32)gDC.lastFootDir.xOffset)*pframe[12];
		temp+=-(((int32)leg.legOffset*pframe[12]*(SIN(gDC.thisFootDir.thetaOffset)-SIN(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
	/*	
		temp=((int32)pframe[0]*walkingConfig.xlanding)+
			(((int32)gDC.lastFootDir.xOffset)<<PRECISION_BIT)-
			(((int32)leg.legOffset*SIN(gDC.lastFootDir.thetaOffset))>>1)+
			((int32)gDC.thisFootDir.xOffset-(int32)gDC.lastFootDir.xOffset)*pframe[12]-
			((((int32)leg.legOffset*pframe[12])*
			(SIN(gDC.thisFootDir.thetaOffset)-SIN(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
	*/		
		ankle.offset.x=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[1]*walkingConfig.ylanding;
		temp+=(((int32)gDC.lastFootDir.yOffset+(leg.legOffset>>1))<<PRECISION_BIT);
		temp+=(((int32)leg.legOffset*COS(gDC.lastFootDir.thetaOffset))>>1);
		temp+=((int32)gDC.thisFootDir.yOffset-(int32)gDC.lastFootDir.yOffset)*pframe[13];
		temp+=(((int32)leg.legOffset*pframe[13]*(COS(gDC.thisFootDir.thetaOffset)-COS(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
		ankle.offset.y=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[2]*walkingConfig.zlanding;
		ankle.offset.z=(int16)(temp>>PRECISION_BIT);
		temp=(((int32)pframe[3]*walkingConfig.alphalanding)>>PRECISION_BIT);
		ankle.pose.alpha=(int16)temp;
		temp=(((int32)pframe[4]*walkingConfig.betalanding)>>PRECISION_BIT);
		ankle.pose.beta=(int16)temp;
		temp=(((int32)pframe[5]*walkingConfig.thetalanding)>>PRECISION_BIT);
		temp+=(int32)gDC.lastFootDir.thetaOffset;
		temp+=(((int32)pframe[14]*(gDC.thisFootDir.thetaOffset-gDC.lastFootDir.thetaOffset))>>PRECISION_BIT);
		ankle.pose.theta=(int16)temp;
		
		temp=(int32)pframe[6]*walkingConfig.xswing;
		temp+=(((int32)gDC.lastHipDir.xOffset)<<PRECISION_BIT);
		temp+=((int32)gDC.thisHipDir.xOffset-(int32)gDC.lastHipDir.xOffset)*pframe[15];
		hip.offset.x=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[7]*walkingConfig.yswing;
		temp+=(((int32)leg.legOffset+(int32)gDC.lastHipDir.yOffset*2)<<(PRECISION_BIT-1));
		temp+=(int32)pframe[16]*(gDC.thisHipDir.yOffset-gDC.lastHipDir.yOffset);
		hip.offset.y=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[8]*walkingConfig.zswing;
		hip.offset.z=(int16)(temp>>PRECISION_BIT);
		temp=(((int32)pframe[9]*walkingConfig.alphaswing)>>PRECISION_BIT);
		hip.pose.alpha=(int16)temp;
		temp=(((int32)pframe[10]*walkingConfig.betaswing)>>PRECISION_BIT);
		hip.pose.beta=(int16)temp;
		temp=(((int32)pframe[11]*walkingConfig.thetaswing)>>PRECISION_BIT);
		temp+=gDC.lastHipDir.thetaOffset;
		temp+=(((int32)pframe[17]*(gDC.thisHipDir.thetaOffset-gDC.lastHipDir.thetaOffset))>>PRECISION_BIT);
		hip.pose.theta=(int32)temp;
		
		leg.legOffset=abs(leg.legOffset);
	}else
	{
		temp=(int32)pframe[0]*walkingConfig.xlanding;
		temp+=((((int32)gDC.lastFootDir.xOffset))<<PRECISION_BIT);
		temp+=(((int32)leg.legOffset*SIN(gDC.lastFootDir.thetaOffset))>>1);
		temp+=((int32)gDC.thisFootDir.xOffset-(int32)gDC.lastFootDir.xOffset)*pframe[12];
		temp+=(((int32)leg.legOffset*pframe[12]*(SIN(gDC.thisFootDir.thetaOffset)-SIN(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
	/*	
		temp=((int32)pframe[0]*walkingConfig.xlanding)+
			(((int32)gDC.lastFootDir.xOffset)<<PRECISION_BIT)-
			(((int32)leg.legOffset*SIN(gDC.lastFootDir.thetaOffset))>>1)+
			((int32)gDC.thisFootDir.xOffset-(int32)gDC.lastFootDir.xOffset)*pframe[12]-
			((((int32)leg.legOffset*pframe[12])*
			(SIN(gDC.thisFootDir.thetaOffset)-SIN(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
	*/		
		ankle.offset.x=(int16)(temp>>PRECISION_BIT);
		temp=-(int32)pframe[1]*walkingConfig.ylanding;
		temp+=(((int32)gDC.lastFootDir.yOffset-(leg.legOffset>>1))<<PRECISION_BIT);
		temp+=-(((int32)leg.legOffset*COS(gDC.lastFootDir.thetaOffset))>>1);
		temp+=((int32)gDC.thisFootDir.yOffset-(int32)gDC.lastFootDir.yOffset)*pframe[13];
		temp+=-(((int32)leg.legOffset*pframe[13]*(COS(gDC.thisFootDir.thetaOffset)-COS(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
		ankle.offset.y=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[2]*walkingConfig.zlanding;
		ankle.offset.z=(int16)(temp>>PRECISION_BIT);
		temp=-(((int32)pframe[3]*walkingConfig.alphalanding)>>PRECISION_BIT);
		ankle.pose.alpha=(int16)temp;
		temp=(((int32)pframe[4]*walkingConfig.betalanding)>>PRECISION_BIT);
		ankle.pose.beta=(int16)temp;
		temp=-(((int32)pframe[5]*walkingConfig.thetalanding)>>PRECISION_BIT);
		temp+=(int32)gDC.lastFootDir.thetaOffset;
		temp+=(((int32)pframe[14]*(gDC.thisFootDir.thetaOffset-gDC.lastFootDir.thetaOffset))>>PRECISION_BIT);
		ankle.pose.theta=(int16)temp;
		
		temp=(int32)pframe[6]*walkingConfig.xswing;
		temp+=(((int32)gDC.lastHipDir.xOffset)<<PRECISION_BIT);
		temp+=((int32)gDC.thisHipDir.xOffset-(int32)gDC.lastHipDir.xOffset)*pframe[15];
		hip.offset.x=(int16)(temp>>PRECISION_BIT);
		temp=-(int32)pframe[7]*walkingConfig.yswing;
		temp+=((-(int32)leg.legOffset+(int32)gDC.lastHipDir.yOffset*2)<<(PRECISION_BIT-1));
		temp+=(int32)pframe[16]*(gDC.thisHipDir.yOffset-gDC.lastHipDir.yOffset);
		hip.offset.y=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[8]*walkingConfig.zswing;
		hip.offset.z=(int16)(temp>>PRECISION_BIT);
		temp=-(((int32)pframe[9]*walkingConfig.alphaswing)>>PRECISION_BIT);
		hip.pose.alpha=(int16)temp;
		temp=(((int32)pframe[10]*walkingConfig.betaswing)>>PRECISION_BIT);
		hip.pose.beta=(int16)temp;
		temp=-(((int32)pframe[11]*walkingConfig.thetaswing)>>PRECISION_BIT);
		temp+=gDC.lastHipDir.thetaOffset;
		temp+=(((int32)pframe[17]*(gDC.thisHipDir.thetaOffset-gDC.lastHipDir.thetaOffset))>>PRECISION_BIT);
		hip.pose.theta=(int32)temp;
		
		leg.legOffset=-abs(leg.legOffset);
	}
	
	return 0;
}

Uint16 FillInStepParameterWalkKick(int16 *pframe,Uint16 len)
{
	int32 temp=0;
	
	leg.legOffset=abs(leg.legOffset);
	if(gDC.lastFoot==0)
	{	
		//temp=(int32)pframe[0]*walkingConfig.xlanding;
		temp=(((int32)pframe[0]*gDC.thisFootDir.xOffset*4)>>2);
		temp+=((((int32)gDC.lastFootDir.xOffset))<<PRECISION_BIT);
		temp+=-(((int32)leg.legOffset*SIN(gDC.lastFootDir.thetaOffset))>>1);
		temp+=((int32)gDC.thisFootDir.xOffset-(int32)gDC.lastFootDir.xOffset)*pframe[12];
		temp+=-(((int32)leg.legOffset*pframe[12]*(SIN(gDC.thisFootDir.thetaOffset)-SIN(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
	/*	
		temp=((int32)pframe[0]*walkingConfig.xlanding)+
			(((int32)gDC.lastFootDir.xOffset)<<PRECISION_BIT)-
			(((int32)leg.legOffset*SIN(gDC.lastFootDir.thetaOffset))>>1)+
			((int32)gDC.thisFootDir.xOffset-(int32)gDC.lastFootDir.xOffset)*pframe[12]-
			((((int32)leg.legOffset*pframe[12])*
			(SIN(gDC.thisFootDir.thetaOffset)-SIN(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
	*/		
		ankle.offset.x=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[1]*walkingConfig.ylanding;
		temp+=(((int32)gDC.lastFootDir.yOffset+(leg.legOffset>>1))<<PRECISION_BIT);
		temp+=(((int32)leg.legOffset*COS(gDC.lastFootDir.thetaOffset))>>1);
		temp+=((int32)gDC.thisFootDir.yOffset-(int32)gDC.lastFootDir.yOffset)*pframe[13];
		temp+=(((int32)leg.legOffset*pframe[13]*(COS(gDC.thisFootDir.thetaOffset)-COS(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
		ankle.offset.y=(int16)(temp>>PRECISION_BIT);
		temp=(((int32)pframe[2]*walkingConfig.zlanding*4)>>2);
		ankle.offset.z=(int16)(temp>>PRECISION_BIT);
		temp=(((int32)pframe[3]*walkingConfig.alphalanding)>>PRECISION_BIT);
		ankle.pose.alpha=(int16)temp;
		temp=(((int32)pframe[4]*walkingConfig.betalanding)>>PRECISION_BIT);
		ankle.pose.beta=(int16)temp;
		temp=(((int32)pframe[5]*walkingConfig.thetalanding)>>PRECISION_BIT);
		temp+=(int32)gDC.lastFootDir.thetaOffset;
		temp+=(((int32)pframe[14]*(gDC.thisFootDir.thetaOffset-gDC.lastFootDir.thetaOffset))>>PRECISION_BIT);
		ankle.pose.theta=(int16)temp;
		
		temp=(int32)pframe[6]*walkingConfig.xswing;
		temp+=(((int32)gDC.lastHipDir.xOffset)<<PRECISION_BIT);
		temp+=((int32)gDC.thisHipDir.xOffset-(int32)gDC.lastHipDir.xOffset)*pframe[15];
		hip.offset.x=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[7]*walkingConfig.yswing;
		temp+=(((int32)leg.legOffset+(int32)gDC.lastHipDir.yOffset*2)<<(PRECISION_BIT-1));
		temp+=(int32)pframe[16]*(gDC.thisHipDir.yOffset-gDC.lastHipDir.yOffset);
		hip.offset.y=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[8]*walkingConfig.zswing;
		hip.offset.z=(int16)(temp>>PRECISION_BIT);
		temp=(((int32)pframe[9]*walkingConfig.alphaswing)>>PRECISION_BIT);
		hip.pose.alpha=(int16)temp;
		temp=(((int32)pframe[10]*walkingConfig.betaswing)>>PRECISION_BIT);
		hip.pose.beta=(int16)temp;
		temp=(((int32)pframe[11]*walkingConfig.thetaswing)>>PRECISION_BIT);
		temp+=gDC.lastHipDir.thetaOffset;
		temp+=(((int32)pframe[17]*(gDC.thisHipDir.thetaOffset-gDC.lastHipDir.thetaOffset))>>PRECISION_BIT);
		hip.pose.theta=(int32)temp;
		
		leg.legOffset=abs(leg.legOffset);
	}else
	{
		//temp=(int32)pframe[0]*walkingConfig.xlanding;
		temp=(((int32)pframe[0]*gDC.thisFootDir.xOffset*4)>>2);
		temp+=((((int32)gDC.lastFootDir.xOffset))<<PRECISION_BIT);
		temp+=(((int32)leg.legOffset*SIN(gDC.lastFootDir.thetaOffset))>>1);
		temp+=((int32)gDC.thisFootDir.xOffset-(int32)gDC.lastFootDir.xOffset)*pframe[12];
		temp+=(((int32)leg.legOffset*pframe[12]*(SIN(gDC.thisFootDir.thetaOffset)-SIN(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
	/*	
		temp=((int32)pframe[0]*walkingConfig.xlanding)+
			(((int32)gDC.lastFootDir.xOffset)<<PRECISION_BIT)-
			(((int32)leg.legOffset*SIN(gDC.lastFootDir.thetaOffset))>>1)+
			((int32)gDC.thisFootDir.xOffset-(int32)gDC.lastFootDir.xOffset)*pframe[12]-
			((((int32)leg.legOffset*pframe[12])*
			(SIN(gDC.thisFootDir.thetaOffset)-SIN(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
	*/		
		ankle.offset.x=(int16)(temp>>PRECISION_BIT);
		temp=-(int32)pframe[1]*walkingConfig.ylanding;
		temp+=(((int32)gDC.lastFootDir.yOffset-(leg.legOffset>>1))<<PRECISION_BIT);
		temp+=-(((int32)leg.legOffset*COS(gDC.lastFootDir.thetaOffset))>>1);
		temp+=((int32)gDC.thisFootDir.yOffset-(int32)gDC.lastFootDir.yOffset)*pframe[13];
		temp+=-(((int32)leg.legOffset*pframe[13]*(COS(gDC.thisFootDir.thetaOffset)-COS(gDC.lastFootDir.thetaOffset)))>>(PRECISION_BIT+1));
		ankle.offset.y=(int16)(temp>>PRECISION_BIT);
		temp=(((int32)pframe[2]*walkingConfig.zlanding*4)>>2);
		ankle.offset.z=(int16)(temp>>PRECISION_BIT);
		temp=-(((int32)pframe[3]*walkingConfig.alphalanding)>>PRECISION_BIT);
		ankle.pose.alpha=(int16)temp;
		temp=(((int32)pframe[4]*walkingConfig.betalanding)>>PRECISION_BIT);
		ankle.pose.beta=(int16)temp;
		temp=-(((int32)pframe[5]*walkingConfig.thetalanding)>>PRECISION_BIT);
		temp+=(int32)gDC.lastFootDir.thetaOffset;
		temp+=(((int32)pframe[14]*(gDC.thisFootDir.thetaOffset-gDC.lastFootDir.thetaOffset))>>PRECISION_BIT);
		ankle.pose.theta=(int16)temp;
		
		temp=(int32)pframe[6]*walkingConfig.xswing;
		temp+=(((int32)gDC.lastHipDir.xOffset)<<PRECISION_BIT);
		temp+=((int32)gDC.thisHipDir.xOffset-(int32)gDC.lastHipDir.xOffset)*pframe[15];
		hip.offset.x=(int16)(temp>>PRECISION_BIT);
		temp=-(int32)pframe[7]*walkingConfig.yswing;
		temp+=((-(int32)leg.legOffset+(int32)gDC.lastHipDir.yOffset*2)<<(PRECISION_BIT-1));
		temp+=(int32)pframe[16]*(gDC.thisHipDir.yOffset-gDC.lastHipDir.yOffset);
		hip.offset.y=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[8]*walkingConfig.zswing;
		hip.offset.z=(int16)(temp>>PRECISION_BIT);
		temp=-(((int32)pframe[9]*walkingConfig.alphaswing)>>PRECISION_BIT);
		hip.pose.alpha=(int16)temp;
		temp=(((int32)pframe[10]*walkingConfig.betaswing)>>PRECISION_BIT);
		hip.pose.beta=(int16)temp;
		temp=-(((int32)pframe[11]*walkingConfig.thetaswing)>>PRECISION_BIT);
		temp+=gDC.lastHipDir.thetaOffset;
		temp+=(((int32)pframe[17]*(gDC.thisHipDir.thetaOffset-gDC.lastHipDir.thetaOffset))>>PRECISION_BIT);
		hip.pose.theta=(int32)temp;
		
		leg.legOffset=-abs(leg.legOffset);
	}
	
	return 0;
}

Uint16 GetArmSwing(int16 *pframe,Uint16 len,int16 *presult)
{
	int32 temp=0;
	if(gDC.lastFoot==0)
	{
		temp=(int32)pframe[20]*walkingConfig.elbowswing;
		presult[13]=(int16)(temp>>PRECISION_BIT);
		presult[18]=-(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[19]*walkingConfig.armfly;
		presult[14]=(int16)(temp>>PRECISION_BIT);
		presult[17]=-(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[18]*walkingConfig.armswing;
		presult[15]=-(int16)(temp>>PRECISION_BIT);
		presult[16]=(int16)(temp>>PRECISION_BIT);
	}else
	{
		temp=(int32)pframe[20]*walkingConfig.elbowswing;
		presult[13]=-(int16)(temp>>PRECISION_BIT);
		presult[18]=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[19]*walkingConfig.armfly;
		presult[14]=-(int16)(temp>>PRECISION_BIT);
		presult[17]=(int16)(temp>>PRECISION_BIT);
		temp=(int32)pframe[18]*walkingConfig.armswing;
		presult[15]=(int16)(temp>>PRECISION_BIT);
		presult[16]=-(int16)(temp>>PRECISION_BIT);
	}
	
	return 0;
}

Uint16 ChangeFoot()
{
	int32 temp=0;
	
	temp=-(int32)gDC.thisFootDir.xOffset*COS(gDC.thisFootDir.thetaOffset);
	temp+=-(int32)gDC.thisFootDir.yOffset*SIN(gDC.thisFootDir.thetaOffset);
	gDC.lastFootDir.xOffset=(int16)(temp>>PRECISION_BIT);
	temp=(int32)gDC.thisFootDir.xOffset*SIN(gDC.thisFootDir.thetaOffset);
	temp+=-(int32)gDC.thisFootDir.yOffset*COS(gDC.thisFootDir.thetaOffset);
	gDC.lastFootDir.yOffset=(int16)(temp>>PRECISION_BIT);
	gDC.lastFootDir.thetaOffset=-gDC.thisFootDir.thetaOffset;
	
	temp=((int32)gDC.thisHipDir.xOffset-(int32)gDC.thisFootDir.xOffset)*COS(gDC.thisFootDir.thetaOffset);
	temp+=((int32)gDC.thisHipDir.yOffset-(int32)gDC.thisFootDir.yOffset)*SIN(gDC.thisFootDir.thetaOffset);
	gDC.lastHipDir.xOffset=(int16)(temp>>PRECISION_BIT);
	temp=-((int32)gDC.thisHipDir.xOffset-(int32)gDC.thisFootDir.xOffset)*SIN(gDC.thisFootDir.thetaOffset);
	temp+=((int32)gDC.thisHipDir.yOffset-(int32)gDC.thisFootDir.yOffset)*COS(gDC.thisFootDir.thetaOffset);
	gDC.lastHipDir.yOffset=(int16)(temp>>PRECISION_BIT);
	gDC.lastHipDir.thetaOffset=-gDC.thisFootDir.thetaOffset+gDC.thisHipDir.thetaOffset;
	

	if(gDC.lastFoot==0)
		gDC.lastFoot=1;
	else
		gDC.lastFoot=0;
	return 0;
}

Uint16 UpdateOdometer(struct GaitEffect *pfootEffect,struct GaitEffect *phipEffect)
{
	int32 temp;
	int16 tempAngle=0;
	
	tempAngle=runningOdometer.footLandingOdometer.thetaOffset;
	if(tempAngle>PI)
	{
		tempAngle%=(2*PI);
		if(tempAngle>PI)
			tempAngle-=(2*PI);
	}else if(tempAngle<-PI)
	{
		tempAngle=((-tempAngle)%(2*PI));
		tempAngle=-tempAngle;
		if(tempAngle<-PI)
			tempAngle+=(2*PI);
	}
	
	temp=(((int32)(pfootEffect->xOffset)*
			COS(tempAngle))>>PRECISION_BIT);
	temp-=(((int32)(pfootEffect->yOffset)*
			SIN(tempAngle))>>PRECISION_BIT);
	runningOdometer.footLandingOdometer.xOffset+=(int16)temp;
	
	temp=(((int32)(pfootEffect->xOffset)*
			SIN(tempAngle))>>PRECISION_BIT);
	temp+=(((int32)(pfootEffect->yOffset)*
			COS(tempAngle))>>PRECISION_BIT);
	runningOdometer.footLandingOdometer.yOffset+=(int16)temp;
	
	runningOdometer.footLandingOdometer.thetaOffset+=pfootEffect->thetaOffset;

	tempAngle=runningOdometer.footLandingOdometer.thetaOffset;
	if(tempAngle>PI)
	{
		tempAngle%=(2*PI);
		if(tempAngle>PI)
			tempAngle-=(2*PI);
	}else if(tempAngle<-PI)
	{
		tempAngle=((-tempAngle)%(2*PI));
		tempAngle=-tempAngle;
		if(tempAngle<-PI)
			tempAngle+=(2*PI);
	}
		
	if(phipEffect!=NULL)
	{
		temp=(((int32)(phipEffect->xOffset)*
				COS(tempAngle))>>PRECISION_BIT);
		temp-=(((int32)(phipEffect->yOffset)*
				SIN(tempAngle))>>PRECISION_BIT);
		runningOdometer.hipSwingOdometer.xOffset=runningOdometer.footLandingOdometer.xOffset+(int16)temp;
		
		temp=(((int32)(phipEffect->xOffset)*
				SIN(tempAngle))>>PRECISION_BIT);
		temp+=(((int32)(phipEffect->yOffset)*
				COS(tempAngle))>>PRECISION_BIT);
		runningOdometer.hipSwingOdometer.yOffset=runningOdometer.footLandingOdometer.yOffset+(int16)temp;
		
		runningOdometer.hipSwingOdometer.thetaOffset=runningOdometer.footLandingOdometer.thetaOffset+phipEffect->thetaOffset;
	}else
		memcpy(&runningOdometer.hipSwingOdometer,&runningOdometer.footLandingOdometer,sizeof(struct GaitEffect));
		
	/*
	temp=(((int32)(pfootEffect->xOffset)*
			COS(runningOdometer.footLandingOdometer.thetaOffset))>>PRECISION_BIT);
	temp-=(((int32)(pfootEffect->yOffset)*
			SIN(runningOdometer.footLandingOdometer.thetaOffset))>>PRECISION_BIT);
	runningOdometer.footLandingOdometer.xOffset+=(int16)temp;
	
	temp=(((int32)(pfootEffect->xOffset)*
			SIN(runningOdometer.footLandingOdometer.thetaOffset))>>PRECISION_BIT);
	temp+=(((int32)(pfootEffect->yOffset)*
			COS(runningOdometer.footLandingOdometer.thetaOffset))>>PRECISION_BIT);
	runningOdometer.footLandingOdometer.yOffset+=(int16)temp;
	
	runningOdometer.footLandingOdometer.thetaOffset+=pfootEffect->thetaOffset;
	
	if(phipEffect!=NULL)
	{
		temp=(((int32)(phipEffect->xOffset)*
				COS(runningOdometer.footLandingOdometer.thetaOffset))>>PRECISION_BIT);
		temp-=(((int32)(phipEffect->yOffset)*
				SIN(runningOdometer.footLandingOdometer.thetaOffset))>>PRECISION_BIT);
		runningOdometer.hipSwingOdometer.xOffset=runningOdometer.footLandingOdometer.xOffset+(int16)temp;
		
		temp=(((int32)(phipEffect->xOffset)*
				SIN(runningOdometer.footLandingOdometer.thetaOffset))>>PRECISION_BIT);
		temp+=(((int32)(phipEffect->yOffset)*
				COS(runningOdometer.footLandingOdometer.thetaOffset))>>PRECISION_BIT);
		runningOdometer.hipSwingOdometer.yOffset=runningOdometer.footLandingOdometer.yOffset+(int16)temp;
		
		runningOdometer.hipSwingOdometer.thetaOffset=runningOdometer.footLandingOdometer.thetaOffset+phipEffect->thetaOffset;
	}else
		memcpy(&runningOdometer.hipSwingOdometer,&runningOdometer.footLandingOdometer,sizeof(struct GaitEffect));
	*/
	
	return 0;
}

Bool GenerateGaitStep(void)
{
	GaitHead tempHead;
	static Uint16 tempGaitFrame[MAX_ROBOT_JOINTS*2];
	Uint16 step=0;
	Uint16 i=0;
	Uint16 j=0;
	int16 fs=0;
	int32 angle;
	int32 speed;
	Uint32 interval;
	Uint16 gaitPlanningRunning=0xffff;
	INT8U err=0;
	
	OSMutexPend(plckUpdateCommand,0,&err);
	while(OSSemAccept(psemGaitGen)!=0);
	
	switch(priorityState)
	{
	case 0:
		if(commandState==0)
		{
			//this process should occur when there is no instruction from upper machine
			OSMutexPost(plckUpdateCommand);
			OSSemPend(psemGaitGen,0,&err);
			OSMutexPend(plckUpdateCommand,0,&err);
		}

		//clear any walk kick command and it is not proper here
		commandState&=~COMMAND_WALK_KICK;
						
		if(commandState&COMMAND_SINGLE_CONFIGURATION)
		{
			commandState&=~COMMAND_SINGLE_CONFIGURATION;
			gaitPlanningRunning=0;
		}else if(commandState&COMMAND_GAIT_RESET)
		{
			commandState&=~COMMAND_GAIT_RESET;
			gaitPlanningRunning=1;
		}else if(commandState&COMMAND_SPECIAL_GAIT)
		{
			if(specialGaitCommand.times>0)
			{
				if(--specialGaitCommand.times==0)
					commandState&=~COMMAND_SPECIAL_GAIT;
				gaitPlanningRunning=2;
			}else
				commandState&=~COMMAND_SPECIAL_GAIT;
		}else if(commandState&COMMAND_SPECIAL_GAIT_QUERY)
		{
			if(specialGaitCommand.times>0)
			{
				if(--specialGaitCommand.times==0)
					commandState&=~COMMAND_SPECIAL_GAIT_QUERY;
				gaitPlanningRunning=4;
			}else
				commandState&=~COMMAND_SPECIAL_GAIT_QUERY;
		}else if(commandState&COMMAND_GAIT_DIRECTION)
		{
			if(gDC.cmdEffect.xOffset!=0||
				gDC.cmdEffect.yOffset!=0||
				gDC.cmdEffect.thetaOffset!=0)
			{				
				priorityState=1;
				gDC.ystage=0;
			}else
				commandState&=~COMMAND_GAIT_DIRECTION;
		}else
		{
			commandState=0;
		}
	break;
	case 1:
		//read zeropoint and walking parameters		
		CloseAll();
		fs=Open(GAIT_ID_GAIT_DIRECTION,FILE_READ_ONLY);
		Read(fs,(Uint16 *)&tempHead,sizeof(tempHead)/sizeof(Uint16));
		mechZeroPoint.joints=tempHead.frameStep;
		Read(fs,mechZeroPoint.buf,tempHead.frameStep);
		Read(fs,(Uint16 *)&leg,sizeof(leg)/sizeof(Uint16));
		Read(fs,(Uint16 *)&walkingConfig,sizeof(walkingConfig)/sizeof(Uint16));
		Close(fs);
			
		if(gDC.ystage<walkingConfig.ystage)
		{
			gDC.ystage++;
			walkingConfig.yswing=walkingConfig.yswing*gDC.ystage/walkingConfig.ystage;
			walkingConfig.alphaswing=walkingConfig.alphaswing*gDC.ystage/walkingConfig.ystage;
			//walkingConfig.zlanding=walkingConfig.zlanding*gDC.ystage/walkingConfig.ystage;
			walkingConfig.zlanding=0;
			walkingConfig.xlanding=0;
			walkingConfig.ylanding=0;
			walkingConfig.alphalanding=0;
			walkingConfig.betalanding=0;
			walkingConfig.thetalanding=0;
			memset(&gDC.instEffect,0,sizeof(struct GaitEffect));
			gaitPlanningRunning=3;
		}else
		{
			//ChangeFoot();
			gDC.standingCntr=0;
			priorityState=2;

		}
	break;
	case 2:
	{
		if((commandState&
			(COMMAND_SINGLE_CONFIGURATION|COMMAND_SPECIAL_GAIT|
			COMMAND_GAIT_RESET|COMMAND_SPECIAL_GAIT_QUERY))||
			!(commandState&COMMAND_GAIT_DIRECTION))
		{
			memset(&gDC.cmdEffect,0,sizeof(gDC.cmdEffect));
		}

		if(commandState&COMMAND_WALK_KICK)
		{
			//initialize the walk kick state and translate to walk kick state
			//tranlate to state according to single step mode or double step mode
			switch(WalkKickControlStep())
			{
				//continous tunning of steps
				case 0:
					gaitPlanningRunning=5;
				break;
				//next step will be kick
				case 1:
					priorityState=5;
					gaitPlanningRunning=5;
				break;
				//direct walk kick action is issued
				case 2:
					gaitPlanningRunning=6;
					commandState&=~COMMAND_WALK_KICK;
					//post processing of walk kick
					gDC.cmdEffect.xOffset=1;
					gDC.cmdEffect.yOffset=0;
					gDC.cmdEffect.thetaOffset=0;
					//reset direction command to marktime
				break;
				default:
				break;
			}
			break;
		}
				
		WalkingSpeedControl();
		if(gDC.footMesuredSpeed.xOffset==0&&
			gDC.footMesuredSpeed.yOffset==0&&
			gDC.footMesuredSpeed.thetaOffset==0&&
			gDC.instEffect.xOffset==0&&
			gDC.instEffect.yOffset==0&&
			gDC.instEffect.thetaOffset==0)
		{
			
			if(++gDC.standingCntr>walkingConfig.ystage)
			{
				priorityState=3;
				commandState&=~COMMAND_GAIT_DIRECTION;
			}else
				gaitPlanningRunning=3;
		}else
		{
			gDC.standingCntr=0;
			gaitPlanningRunning=3;
		}
	}
	break;
	case 3:
		//read zeropoint and walking parameters		
		CloseAll();
		fs=Open(GAIT_ID_GAIT_DIRECTION,FILE_READ_ONLY);
		Read(fs,(Uint16 *)&tempHead,sizeof(tempHead)/sizeof(Uint16));
		mechZeroPoint.joints=tempHead.frameStep;
		Read(fs,mechZeroPoint.buf,tempHead.frameStep);
		Read(fs,(Uint16 *)&leg,sizeof(leg)/sizeof(Uint16));
		Read(fs,(Uint16 *)&walkingConfig,sizeof(walkingConfig)/sizeof(Uint16));
		Close(fs);
		
		if(gDC.ystage>0)
		{
			gDC.ystage--;
			walkingConfig.yswing=walkingConfig.yswing*gDC.ystage/walkingConfig.ystage;
			walkingConfig.alphaswing=walkingConfig.alphaswing*gDC.ystage/walkingConfig.ystage;
			walkingConfig.zlanding=walkingConfig.zlanding*gDC.ystage/walkingConfig.ystage;
			memset(&gDC.instEffect,0,sizeof(struct GaitEffect));
			gaitPlanningRunning=3;
		}else
		{
			gDC.stableClock=OSTimeGet();
			priorityState=4;
		}	
	break;
	case 4:
	{
		if(OSTimeGet()-gDC.stableClock>walkingConfig.planningStableCycle*10)
			priorityState=0;
	}
	break;
	case 5:
	{
		//modulize kick direction step is neccessary and completed
		//issue real kick command and translate to walk kick state
		ExecuteWalkKick();
		commandState&=~COMMAND_WALK_KICK;	
		gaitPlanningRunning=6;

		//reset direction command to marktime
		gDC.cmdEffect.xOffset=1;
		gDC.cmdEffect.yOffset=0;
		gDC.cmdEffect.thetaOffset=0;
		priorityState=2;
	}
	default:
	break;
	}	
	
	switch(gaitPlanningRunning)
	{
	case 0:
	{
		//here is the only position that locks command mutex with adding gait data
		tempHead.gaitID=GAIT_ID_SINGLE_CONFIG;
		memset(&(tempHead.effect),0,sizeof(tempHead.effect));
		tempHead.frameLength=1;
		tempHead.period=singleFrameCommand.nextStep;
		tempHead.frameStep=singleFrameCommand.size-1;			
		AddNewGaitWithPose(&tempHead,(singleFrameCommand.frameBuffer+1),NULL,NULL,INVALID_RIGIDBODY_VALUE);
		for(i=0;i<tempHead.period-1;i++)
			AddGaitDataWithPose(tempHead.frameStep,singleFrameCommand.frameBuffer+1,NULL,NULL,INVALID_RIGIDBODY_VALUE);
		OSMutexPost(plckUpdateCommand);
		
		//gait planning rythm control
		WaitUntilQueueLen(0);
		
		//return certain packet to upper machine for compliance with previous edition
		{
			static Byte buffer[6];
			DSP_STATUS_PACKET *pret=NULL;
			Uint16 ret=0;
			pret=(DSP_STATUS_PACKET *)GetCOMHead(buffer);
			pret->id=ID_DSP;
			pret->infomation=INFO_GAIT_EXECUTED;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
			SendPacket(buffer,ret);
		}
	}
	break;
	case 1:
	{
		OSMutexPost(plckUpdateCommand);
		TakeSingleAction(20,512,DEFAULT_TURNING_SPEED);
		TakeSingleAction(21,512,DEFAULT_TURNING_SPEED);		
		//read zeropoint and walking parameters		
		CloseAll();
		fs=Open(GAIT_ID_GAIT_DIRECTION,FILE_READ_ONLY);
		Read(fs,(Uint16 *)&tempHead,sizeof(tempHead)/sizeof(Uint16));
		mechZeroPoint.joints=tempHead.frameStep;
		Read(fs,mechZeroPoint.buf,tempHead.frameStep);
		Read(fs,(Uint16 *)&leg,sizeof(leg)/sizeof(Uint16));
		Read(fs,(Uint16 *)&walkingConfig,sizeof(walkingConfig)/sizeof(Uint16));
		Close(fs);
		
		fs=Open(GAIT_ID_GAIT_DIRECTION+1,FILE_READ_ONLY);
		Read(fs,(Uint16 *)&tempHead,sizeof(tempHead)/sizeof(Uint16));
		memcpy(&tempHead.effect,&gDC.thisFootDir,sizeof(gDC.thisFootDir));
		step=tempHead.frameStep;
		
		//and read from flash about the style to tempbuffer
		Read(fs,tempGaitFrame,step);
		
		//and update to hip ankle and leg parameter for inverse kinematics
		FillInStepParameter((int16 *)tempGaitFrame,step);
		
		//calculate the inverse kinematics
		memset(singleFrameCommand.frameBuffer,0,sizeof(singleFrameCommand.frameBuffer));
		GetInverseKine(&ankle,&hip,&leg,(int16 *)(singleFrameCommand.frameBuffer));
		GetArmSwing((int16 *)tempGaitFrame,step,(int16 *)(singleFrameCommand.frameBuffer));
		ModifyToTurnningAngle((int16 *)(singleFrameCommand.frameBuffer));
		
		for(i=0;i<mechZeroPoint.joints;i++)
		{
			//update position
			angle=(int16)(singleFrameCommand.frameBuffer[i]);
			angle=((angle*801053)>>21);
			angle+=(int32)(mechZeroPoint.buf[i]);
			angle=(angle<0?0:angle);
			angle=(angle>1023?1023:angle);
			singleFrameCommand.frameBuffer[i]=(Uint16)angle;
			//update speed
			singleFrameCommand.frameBuffer[i+mechZeroPoint.joints]=DEFAULT_TURNING_SPEED;
		}
		tempHead.frameStep=mechZeroPoint.joints*2;
		tempHead.frameLength=1;
		tempHead.period=1;
		AddNewGaitWithPose(&tempHead,singleFrameCommand.frameBuffer,NULL,&hip,gDC.lastFoot);
		Close(fs);		
	}
	break;
	case 2:
	case 4:
	{
		CloseAll();
		fs=Open(specialGaitCommand.id,FILE_READ_ONLY);
		OSMutexPost(plckUpdateCommand);
		
		Read(fs,(Uint16 *)&tempHead,sizeof(tempHead)/sizeof(Uint16));
		step=tempHead.frameStep;
		tempHead.frameStep=(tempHead.frameStep-1)*2;
		
		//get the next command from flash
		Read(fs,singleFrameCommand.frameBuffer,step);
		for(i=0;i<step-1;i++)
			singleFrameCommand.frameBuffer[i+step]=1;
		AddNewGaitWithPose(&tempHead,singleFrameCommand.frameBuffer+1,NULL,NULL,INVALID_RIGIDBODY_VALUE);
		// add additional gait to running buffer
		for(i=0;i<tempHead.frameLength-1;i++)
		{
			Uint16 cntr=0;
			Read(fs,singleFrameCommand.frameBuffer+step,step);
			//spline line or linear interpolation
			for(j=0;j<step-1;j++)
			{
				speed=((int32)singleFrameCommand.frameBuffer[j+1]-
						(int32)singleFrameCommand.frameBuffer[j+1+step])*
						SERVO_SPEED_CONSTANT/singleFrameCommand.frameBuffer[0];
				speed=(speed<0?-speed:speed);
				speed=(speed<=SPEED_DEGRADE_CONST?1:speed-SPEED_DEGRADE_CONST);
				speed=(speed>1023?0:speed);
				//speed=0;
				tempGaitFrame[j+step-1]=(Uint16)speed;
			}
			
			for(cntr=1;cntr<singleFrameCommand.frameBuffer[0];cntr++)
			{
				for(j=0;j<step-1;j++)
				{
					angle=((int32)(singleFrameCommand.frameBuffer[j+1+step])-
							(int32)(singleFrameCommand.frameBuffer[j+1]))*
							cntr/singleFrameCommand.frameBuffer[0]+
							(int32)(singleFrameCommand.frameBuffer[j+1]);
					tempGaitFrame[j]=(Uint16)angle;
				}
				AddGaitDataWithPose(tempHead.frameStep,tempGaitFrame,NULL,NULL,INVALID_RIGIDBODY_VALUE);
			}
			
			memcpy(tempGaitFrame,singleFrameCommand.frameBuffer+step+1,(step-1)*sizeof(Uint16));
			AddGaitDataWithPose(tempHead.frameStep,tempGaitFrame,NULL,NULL,INVALID_RIGIDBODY_VALUE);
			
			memcpy(singleFrameCommand.frameBuffer,singleFrameCommand.frameBuffer+step,step*sizeof(Uint16));
		}
		
		//gait planning rythm control
		WaitUntilQueueLen(tempHead.frameStep);
		OSMutexPend(plckUpdateCommand,0,&err);
		UpdateOdometer(&tempHead.effect,NULL);
		OSMutexPost(plckUpdateCommand);
		//return certain packet to upper machine for compliance with previous edition
		if(gaitPlanningRunning==2)
		{
			static Byte buffer[6];
			DSP_STATUS_PACKET *pret=NULL;
			Uint16 ret=0;
			pret=(DSP_STATUS_PACKET *)GetCOMHead(buffer);
			pret->id=ID_DSP;
			pret->infomation=INFO_GAIT_EXECUTED;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
			SendPacket(buffer,ret);
		}
	}
	break;
	case 3:
	{
		//first get next target landing foot position
		SingleStepControl();	
		OSMutexPost(plckUpdateCommand);
		
		interval=OSTimeGet();	
		fs=Open(GAIT_ID_GAIT_DIRECTION+1,FILE_READ_ONLY);
		Read(fs,(Uint16 *)&tempHead,sizeof(tempHead)/sizeof(Uint16));
		memcpy(&tempHead.effect,&gDC.thisFootDir,sizeof(gDC.thisFootDir));
		step=tempHead.frameStep;
		
		//and read from flash about the style to tempbuffer
		Read(fs,tempGaitFrame,step);
		
		//and update to hip ankle and leg parameter for inverse kinematics
		FillInStepParameter((int16 *)tempGaitFrame,step);
		
		//calculate the inverse kinematics
		memset(singleFrameCommand.frameBuffer,0,sizeof(singleFrameCommand.frameBuffer));
		GetInverseKine(&ankle,&hip,&leg,(int16 *)(singleFrameCommand.frameBuffer));
		GetArmSwing((int16 *)tempGaitFrame,step,(int16 *)(singleFrameCommand.frameBuffer));
		ModifyToTurnningAngle((int16 *)(singleFrameCommand.frameBuffer));
		
		for(i=0;i<mechZeroPoint.joints;i++)
		{
			//update position
			angle=(int16)(singleFrameCommand.frameBuffer[i]);
			angle=((angle*801053)>>21);
			angle+=(int32)(mechZeroPoint.buf[i]);
			angle=(angle<0?0:angle);
			angle=(angle>1023?1023:angle);
			singleFrameCommand.frameBuffer[i]=(Uint16)angle;
			//update speed
			singleFrameCommand.frameBuffer[i+mechZeroPoint.joints]=1;
		}
		tempHead.frameStep=mechZeroPoint.joints*2;
		AddNewGaitWithPose(&tempHead,singleFrameCommand.frameBuffer,NULL,&hip,gDC.lastFoot);
		
		for(i=1;i<tempHead.period;i++)
		{
			//read flash and get ready for inverse kinematics
			Read(fs,tempGaitFrame,step);
			FillInStepParameter((int16 *)tempGaitFrame,step);
			
			//memset(singleFrameCommand.frameBuffer+step,0,step);
			for(j=0;j<mechZeroPoint.joints;j++)
				singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]=0;
			GetInverseKine(&ankle,&hip,&leg,(int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			GetArmSwing((int16 *)tempGaitFrame,step,(int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			ModifyToTurnningAngle((int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			
			for(j=0;j<mechZeroPoint.joints;j++)
			{
				//update position
				angle=(int16)(singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]);
				angle=((angle*801053)>>21);
				angle+=(int32)(mechZeroPoint.buf[j]);
				angle=(angle<0?0:angle);
				angle=(angle>1023?1023:angle);
				singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]=(Uint16)angle;
				
				//update speed
				speed=((int32)singleFrameCommand.frameBuffer[j]-
						(int32)singleFrameCommand.frameBuffer[j+mechZeroPoint.joints])*
						SERVO_SPEED_CONSTANT;
				speed=(speed<0?-speed:speed);
				speed=(speed<=SPEED_DEGRADE_CONST?1:speed-SPEED_DEGRADE_CONST);
				speed=(speed>1023?0:speed);

				//install motion command
				tempGaitFrame[j+mechZeroPoint.joints]=(Uint16)speed;
				tempGaitFrame[j]=singleFrameCommand.frameBuffer[j+mechZeroPoint.joints];
			}
			AddGaitDataWithPose(tempHead.frameStep,tempGaitFrame,NULL,&hip,gDC.lastFoot);
			memcpy(singleFrameCommand.frameBuffer,singleFrameCommand.frameBuffer+mechZeroPoint.joints,mechZeroPoint.joints*sizeof(Uint16));
		}
		Close(fs);
		ChangeFoot();
		OSMutexPend(plckUpdateCommand,0,&err);
		UpdateOdometer(&gDC.thisFootDir,&gDC.lastHipDir);
		OSMutexPost(plckUpdateCommand);
		interval=OSTimeGet()-interval;
	}
	break;
	case 5:
	{
		OSMutexPost(plckUpdateCommand);
		
		interval=OSTimeGet();	
		fs=Open(GAIT_ID_GAIT_DIRECTION+1,FILE_READ_ONLY);
		Read(fs,(Uint16 *)&tempHead,sizeof(tempHead)/sizeof(Uint16));
		memcpy(&tempHead.effect,&gDC.thisFootDir,sizeof(gDC.thisFootDir));
		step=tempHead.frameStep;
		
		//and read from flash about the style to tempbuffer
		Read(fs,tempGaitFrame,step);
		
		//and update to hip ankle and leg parameter for inverse kinematics
		FillInStepParameter((int16 *)tempGaitFrame,step);
		
		//calculate the inverse kinematics
		memset(singleFrameCommand.frameBuffer,0,sizeof(singleFrameCommand.frameBuffer));
		GetInverseKine(&ankle,&hip,&leg,(int16 *)(singleFrameCommand.frameBuffer));
		GetArmSwing((int16 *)tempGaitFrame,step,(int16 *)(singleFrameCommand.frameBuffer));
		ModifyToTurnningAngle((int16 *)(singleFrameCommand.frameBuffer));
		
		for(i=0;i<mechZeroPoint.joints;i++)
		{
			//update position
			angle=(int16)(singleFrameCommand.frameBuffer[i]);
			angle=((angle*801053)>>21);
			angle+=(int32)(mechZeroPoint.buf[i]);
			angle=(angle<0?0:angle);
			angle=(angle>1023?1023:angle);
			singleFrameCommand.frameBuffer[i]=(Uint16)angle;
			//update speed
			singleFrameCommand.frameBuffer[i+mechZeroPoint.joints]=1;
		}
		tempHead.frameStep=mechZeroPoint.joints*2;
		AddNewGaitWithPose(&tempHead,singleFrameCommand.frameBuffer,NULL,&hip,gDC.lastFoot);
		
		for(i=1;i<tempHead.period;i++)
		{
			//read flash and get ready for inverse kinematics
			Read(fs,tempGaitFrame,step);
			FillInStepParameter((int16 *)tempGaitFrame,step);
			
			//memset(singleFrameCommand.frameBuffer+step,0,step);
			for(j=0;j<mechZeroPoint.joints;j++)
				singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]=0;
			GetInverseKine(&ankle,&hip,&leg,(int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			GetArmSwing((int16 *)tempGaitFrame,step,(int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			ModifyToTurnningAngle((int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			
			for(j=0;j<mechZeroPoint.joints;j++)
			{
				//update position
				angle=(int16)(singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]);
				angle=((angle*801053)>>21);
				angle+=(int32)(mechZeroPoint.buf[j]);
				angle=(angle<0?0:angle);
				angle=(angle>1023?1023:angle);
				singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]=(Uint16)angle;
				
				//update speed
				speed=((int32)singleFrameCommand.frameBuffer[j]-
						(int32)singleFrameCommand.frameBuffer[j+mechZeroPoint.joints])*
						SERVO_SPEED_CONSTANT;
				speed=(speed<0?-speed:speed);
				speed=(speed<=SPEED_DEGRADE_CONST?1:speed-SPEED_DEGRADE_CONST);
				speed=(speed>1023?0:speed);

				//install motion command
				tempGaitFrame[j+mechZeroPoint.joints]=(Uint16)speed;
				tempGaitFrame[j]=singleFrameCommand.frameBuffer[j+mechZeroPoint.joints];
			}
			AddGaitDataWithPose(tempHead.frameStep,tempGaitFrame,NULL,&hip,gDC.lastFoot);
			memcpy(singleFrameCommand.frameBuffer,singleFrameCommand.frameBuffer+mechZeroPoint.joints,mechZeroPoint.joints*sizeof(Uint16));
		}
		Close(fs);
		ChangeFoot();
		OSMutexPend(plckUpdateCommand,0,&err);
		UpdateOdometer(&gDC.thisFootDir,&gDC.lastHipDir);
		OSMutexPost(plckUpdateCommand);
		interval=OSTimeGet()-interval;
	}
	break;
	case 6:
	{
		OSMutexPost(plckUpdateCommand);
		
		interval=OSTimeGet();	
		fs=Open(GAIT_ID_GAIT_DIRECTION+1,FILE_READ_ONLY);
		Read(fs,(Uint16 *)&tempHead,sizeof(tempHead)/sizeof(Uint16));
		memcpy(&tempHead.effect,&gDC.thisFootDir,sizeof(gDC.thisFootDir));
		step=tempHead.frameStep;
		
		//and read from flash about the style to tempbuffer
		Read(fs,tempGaitFrame,step);

		//before filling in parameter, modification should be made to enable a stable walk kick process
				
		//and update to hip ankle and leg parameter for inverse kinematics
		FillInStepParameterWalkKick((int16 *)tempGaitFrame,step);
		
		//calculate the inverse kinematics
		memset(singleFrameCommand.frameBuffer,0,sizeof(singleFrameCommand.frameBuffer));
		GetInverseKine(&ankle,&hip,&leg,(int16 *)(singleFrameCommand.frameBuffer));
		GetArmSwing((int16 *)tempGaitFrame,step,(int16 *)(singleFrameCommand.frameBuffer));
		ModifyToTurnningAngle((int16 *)(singleFrameCommand.frameBuffer));
		
		for(i=0;i<mechZeroPoint.joints;i++)
		{
			//update position
			angle=(int16)(singleFrameCommand.frameBuffer[i]);
			angle=((angle*801053)>>21);
			angle+=(int32)(mechZeroPoint.buf[i]);
			angle=(angle<0?0:angle);
			angle=(angle>1023?1023:angle);
			singleFrameCommand.frameBuffer[i]=(Uint16)angle;
			//update speed
			singleFrameCommand.frameBuffer[i+mechZeroPoint.joints]=1;
		}
		tempHead.frameStep=mechZeroPoint.joints*2;
		AddNewGaitWithPose(&tempHead,singleFrameCommand.frameBuffer,NULL,&hip,gDC.lastFoot);
		
		for(i=1;i<tempHead.period;i++)
		{
			//read flash and get ready for inverse kinematics
			Read(fs,tempGaitFrame,step);
			FillInStepParameterWalkKick((int16 *)tempGaitFrame,step);
			
			//memset(singleFrameCommand.frameBuffer+step,0,step);
			for(j=0;j<mechZeroPoint.joints;j++)
				singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]=0;
			GetInverseKine(&ankle,&hip,&leg,(int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			GetArmSwing((int16 *)tempGaitFrame,step,(int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			ModifyToTurnningAngle((int16 *)(singleFrameCommand.frameBuffer+mechZeroPoint.joints));
			
			for(j=0;j<mechZeroPoint.joints;j++)
			{
				//update position
				angle=(int16)(singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]);
				angle=((angle*801053)>>21);
				angle+=(int32)(mechZeroPoint.buf[j]);
				angle=(angle<0?0:angle);
				angle=(angle>1023?1023:angle);
				singleFrameCommand.frameBuffer[j+mechZeroPoint.joints]=(Uint16)angle;
				
				//update speed
				speed=((int32)singleFrameCommand.frameBuffer[j]-
						(int32)singleFrameCommand.frameBuffer[j+mechZeroPoint.joints])*
						SERVO_SPEED_CONSTANT;
				speed=(speed<0?-speed:speed);
				speed=(speed<=SPEED_DEGRADE_CONST?1:speed-SPEED_DEGRADE_CONST);
				speed=(speed>1023?0:speed);

				//install motion command
				tempGaitFrame[j+mechZeroPoint.joints]=(Uint16)speed;
				tempGaitFrame[j]=singleFrameCommand.frameBuffer[j+mechZeroPoint.joints];
			}
			AddGaitDataWithPose(tempHead.frameStep,tempGaitFrame,NULL,&hip,gDC.lastFoot);
			memcpy(singleFrameCommand.frameBuffer,singleFrameCommand.frameBuffer+mechZeroPoint.joints,mechZeroPoint.joints*sizeof(Uint16));
		}
		Close(fs);
		ChangeFoot();
		OSMutexPend(plckUpdateCommand,0,&err);
		UpdateOdometer(&gDC.thisFootDir,&gDC.lastHipDir);
		OSMutexPost(plckUpdateCommand);
		interval=OSTimeGet()-interval;
	}
	break;
	default:
		OSMutexPost(plckUpdateCommand);
		break;
	}

	return 0;
}

Uint16 HandleGaitInst(struct StateSwapInput *pin,struct StateSwapOutput *pout)
{
	INT8U err=0;
	OSMutexPend(plckUpdateCommand,0,&err);
	
	if(pin->ctrReg1&GAIT_DIRECTION_VALID)
	{
		commandState|=COMMAND_GAIT_DIRECTION;
		//memcpy(&gDC.cmdEffect,&(pin->dirInst),sizeof(struct GaitEffect));
		gDC.cmdEffect.xOffset=pin->dirInst.xOffset;
		gDC.cmdEffect.yOffset=pin->dirInst.yOffset;
		gDC.cmdEffect.thetaOffset=pin->dirInst.thetaOffset;
		pout->stsReg1|=	GAIT_DIRECTION_VALID;
	}

	if((pin->ctrReg1&WALK_KICK_LEFT)||(pin->ctrReg1&WALK_KICK_RIGHT))
	{
		//possibly pending state reqiring
		if((pin->ctrReg1&WALK_KICK_LEFT)&&(pin->ctrReg1&WALK_KICK_RIGHT))
		{
			//command is already issued
			if(!(commandState&COMMAND_WALK_KICK))
			{
				//notify completence of walk kick procedure
				pout->stsReg1&=~WALK_KICK_LEFT;
				pout->stsReg1&=~WALK_KICK_RIGHT;
			}else
			{
				//notify pending of kick
				pout->stsReg1|=WALK_KICK_LEFT;
				pout->stsReg1|=WALK_KICK_RIGHT;
			}
		}else if(pin->ctrReg1&WALK_KICK_LEFT)
		{
				commandState|=COMMAND_WALK_KICK;
				walkKickCmd.cmdEffect.xOffset=pin->dirInst.xOffset;
				walkKickCmd.cmdEffect.yOffset=pin->dirInst.yOffset;
				walkKickCmd.cmdEffect.thetaOffset=pin->dirInst.thetaOffset;
				walkKickCmd.isLeft=TRUE;
				
				//notify pending of kick
				pout->stsReg1|=WALK_KICK_LEFT;
				pout->stsReg1|=WALK_KICK_RIGHT;
		}else if(pin->ctrReg1&WALK_KICK_RIGHT)
		{
				commandState|=COMMAND_WALK_KICK;
				walkKickCmd.cmdEffect.xOffset=pin->dirInst.xOffset;
				walkKickCmd.cmdEffect.yOffset=pin->dirInst.yOffset;
				walkKickCmd.cmdEffect.thetaOffset=pin->dirInst.thetaOffset;
				walkKickCmd.isLeft=FALSE;

				//notify pending of kick
				pout->stsReg1|=WALK_KICK_LEFT;
				pout->stsReg1|=WALK_KICK_RIGHT;
		}
	}else
	{
		//clearance of any remaining walk kick command
		//commandState&=~COMMAND_WALK_KICK;
		pout->stsReg1&=~WALK_KICK_LEFT;
		pout->stsReg1&=~WALK_KICK_RIGHT;
	}
		
	if(pin->ctrReg1&SPECIAL_GAIT_VALID)
	{
		commandState|=COMMAND_SPECIAL_GAIT_QUERY;
		//memcpy(&specialGaitCommand,&pin->spcInst,sizeof(specialGaitCommand));
		specialGaitCommand.id=pin->spcInst.id;
		specialGaitCommand.times=pin->spcInst.times;
		if(specialGaitCommand.id>255)
		{
			specialGaitCommand.id=0;
		}
		pout->stsReg1&=	~SPECIAL_GAIT_VALID;
	}
	
	if(pin->ctrReg1&GAIT_RESET_VALID)
	{
		commandState|=COMMAND_GAIT_RESET;
		pout->stsReg1&=	~GAIT_RESET_VALID;
	}
	
	if(commandState!=0)
		OSSemPost(psemGaitGen);
	
	//memcpy(&(pout->dirSts),&(gDC.footMesuredSpeed),sizeof(struct GaitEffect));
	pout->dirSts.xOffset=gDC.footMesuredSpeed.xOffset;
	pout->dirSts.yOffset=gDC.footMesuredSpeed.yOffset;
	pout->dirSts.thetaOffset=gDC.footMesuredSpeed.thetaOffset;
	//memcpy(&(pout->spcSts),&(specialGaitCommand),sizeof(specialGaitCommand));
	pout->spcSts.id=specialGaitCommand.id;
	pout->spcSts.times=specialGaitCommand.times;

	if(commandState&COMMAND_SPECIAL_GAIT_QUERY)
		pout->stsReg2|=SPECIAL_GAIT_PENDING;
	else
		pout->stsReg2&=~SPECIAL_GAIT_PENDING;
		
	if(commandState&COMMAND_GAIT_RESET)
		pout->stsReg2|=GAIT_RESET_PENDING;
	else
		pout->stsReg2&=~GAIT_RESET_PENDING;

	if(commandState&COMMAND_WALK_KICK)
		pout->stsReg2|=WALK_KICK_PENDING;
	else
		pout->stsReg2&=~WALK_KICK_PENDING;
		
	OSMutexPost(plckUpdateCommand);
	
	return 0;
}

Uint16 HandleHeadMoving(struct StateSwapInput *pin,struct StateSwapOutput *pout)
{
	int32 angle;
	int32 speed;

	pout->headSts.pitch=headMovingCommand.pitch;
	pout->headSts.yaw=headMovingCommand.yaw;
		
	if(!(pin->ctrReg1&HEAD_MOVE_VALID))
		return 0;
	pout->stsReg1|=HEAD_MOVE_VALID;
	if(pin->headInst.yaw!=headMovingCommand.yaw)
	{
		//update position
		angle=(int16)(pin->headInst.yaw);
		angle=((angle*801053)>>21);
		angle+=512;
		angle=(angle<0?0:angle);
		angle=(angle>1023?1023:angle);
		
		//update speed
		speed=((int32)pin->headInst.yaw-
				(int32)headMovingCommand.yaw)*
				(SERVO_SPEED_CONSTANT>>1);
		speed=(speed<0?-speed:speed);
		speed=(speed<=SPEED_DEGRADE_CONST?1:speed-SPEED_DEGRADE_CONST);
		speed=(speed>1023?0:speed);
		TakeSingleAction(20,(Uint16)angle,(Uint16)speed);
		headMovingCommand.yaw=pin->headInst.yaw;
	}
	
	if(pin->headInst.pitch!=headMovingCommand.pitch)
	{
		//update position
		angle=(int16)(pin->headInst.pitch);
		angle=((angle*801053)>>21);
		angle+=512;
		angle=(angle<0?0:angle);
		angle=(angle>1023?1023:angle);
		
		//update speed
		speed=((int32)pin->headInst.pitch-
				(int32)headMovingCommand.pitch)*
				(SERVO_SPEED_CONSTANT>>1);
		speed=(speed<0?-speed:speed);
		speed=(speed<=SPEED_DEGRADE_CONST?1:speed-SPEED_DEGRADE_CONST);
		speed=(speed>1023?0:speed);
		TakeSingleAction(21,(Uint16)angle,(Uint16)speed);
		headMovingCommand.pitch=pin->headInst.pitch;
	}
	
	return 0;
}

Uint16 ExecuteGaitDirectionExt(struct GaitEffect *peffect)
{
	INT8U err=0;
	OSMutexPend(plckUpdateCommand,0,&err);
	memcpy(&gDC.cmdEffect,peffect,sizeof(struct GaitEffect));
	commandState|=COMMAND_GAIT_DIRECTION;
	OSSemPost(psemGaitGen);
	OSMutexPost(plckUpdateCommand);
	return 0;
}

Uint16 ExecuteGait(Uint16 id,Uint16 times)
{
	INT8U err=0;
	OSMutexPend(plckUpdateCommand,0,&err);
	specialGaitCommand.id=id;
	specialGaitCommand.times=times;
	commandState|=COMMAND_SPECIAL_GAIT;
	OSSemPost(psemGaitGen);
	OSMutexPost(plckUpdateCommand);
	return 0;
}

Uint16 SmoothConfiguration(Uint16 *pconfig,Uint16 len)
{
	Uint16 i=0;
	int32 temp=0;
	INT8U err=0;
	//copy the current configuration to singleFrameBuffer
	OSMutexPend(plckUpdateCommand,0,&err);
	commandState|=COMMAND_SINGLE_CONFIGURATION;
	singleFrameCommand.nextStep=singleFrameCommand.frameBuffer[0];
	for(i=1;i<len;i++)
	{
		//temp=((int32)singleFrameCommand.frameBuffer[i]-(int32)pconfig[i])*71/(int32)singleFrameCommand.nextStep;
		temp=((int32)singleFrameCommand.frameBuffer[i]-(int32)pconfig[i])*SERVO_SPEED_CONSTANT/(int32)singleFrameCommand.nextStep;
		temp=(temp>0?temp:-temp);
		temp=(temp<=SPEED_DEGRADE_CONST?1:temp-SPEED_DEGRADE_CONST);
		singleFrameCommand.frameBuffer[len+i-1]=(Uint16)(temp>1023?0:temp);
	}
	memcpy(singleFrameCommand.frameBuffer,pconfig,len*sizeof(Uint16));
	singleFrameCommand.size=len+len-1;
	OSSemPost(psemGaitGen);
	OSMutexPost(plckUpdateCommand);
	return 0;
}

Uint16 ExecuteConfiguration(Uint16 *pconfig,Uint16 len)
{
	Uint16 i=0;
	INT8U err=0;
	//copy the current configuration to singleFrameBuffer
	OSMutexPend(plckUpdateCommand,0,&err);
	commandState|=COMMAND_SINGLE_CONFIGURATION;
	singleFrameCommand.nextStep=1;
	i=len-1;
	while(i--)
		singleFrameCommand.frameBuffer[len+i]=DEFAULT_TURNING_SPEED;
	memcpy(singleFrameCommand.frameBuffer,pconfig,len*sizeof(Uint16));
	singleFrameCommand.size=len+len-1;
	OSSemPost(psemGaitGen);
	OSMutexPost(plckUpdateCommand);

	return 0;
}

Uint16 HandleOdometer(struct StateSwapInput *pin,struct StateSwapOutput *pout)
{
	INT8U err=0;
	OSMutexPend(plckUpdateCommand,0,&err);
	pout->odometer.xOffset=runningOdometer.hipSwingOdometer.xOffset;
	pout->odometer.yOffset=runningOdometer.hipSwingOdometer.yOffset;
	pout->odometer.thetaOffset=runningOdometer.hipSwingOdometer.thetaOffset;
	if(pin->ctrReg1&COPY_RESET_ODOMETER)
	{
		memset(&runningOdometer,0,sizeof(runningOdometer));
		pout->stsReg1&=	~COPY_RESET_ODOMETER;
	}
	pout->stsReg2&= ~RESET_ODOMETER_PENDING;
	OSMutexPost(plckUpdateCommand);
	return 0;
}

int16 GetNextConfiguration(Uint16 *pbuf,Uint16 size)
{
	Uint16 i=0;
	int16 ret=0;
	Uint16 *pdata=NULL;
	INT8U err=0;
	
	OSMutexPend(plckGaitQueue,0,&err);
	if(
		gaitStepCntr>=currentGaitHead.period||
		currentGaitHead.gaitID==INVALID_GAIT_ID
		)
	{		
		//try to read the gait head or return failure
		if(GaitBufferQueueLen(&gaitBufferQueue)>=sizeof(currentGaitHead)/sizeof(Uint16))
		{	
			pdata=(Uint16 *)&currentGaitHead;
			for(i=0;i<sizeof(currentGaitHead)/sizeof(Uint16);i++)
			{
				pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
				gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
			}
			pdata=pbuf;			
			for(i=0;i<currentGaitHead.frameStep;i++)
			{
				pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
				gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
			}
			gaitStepCntr=1;
			OSSemPost(psemGaitExe);
			ret=currentGaitHead.frameStep;
		}else
		{
			//current gait is not available return proper info to caller
			memset(&currentGaitHead,0,sizeof(currentGaitHead));
			gaitStepCntr=0;
			ret=-1;
		}
		
	}else
	{
		//extract the following frame and return
		//fast gait generation is supposed or waiting is required
		if(GaitBufferQueueLen(&gaitBufferQueue)>=currentGaitHead.frameStep)
		{
			for(i=0;i<currentGaitHead.frameStep;i++)
			{
				pbuf[i]=gaitBufferQueue.q[gaitBufferQueue.f];
				gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
			}
			gaitStepCntr++;
			OSSemPost(psemGaitExe);
			ret=currentGaitHead.frameStep;
		}else
		{
			//current frame is not available and infom caller to wait
			ret=-2;
		}
	}
	OSMutexPost(plckGaitQueue);
	return ret;
}

int16 GetNextConfigurationWithPose(Uint16 *pbuf,Uint16 size,struct RigidBody *pankle, struct RigidBody *phip, Uint16 *pisLeft)
{
	Uint16 i=0;
	int16 ret=0;
	Uint16 *pdata=NULL;
	INT8U err=0;
	
	OSMutexPend(plckGaitQueue,0,&err);
	if(
		gaitStepCntr>=currentGaitHead.period||
		currentGaitHead.gaitID==INVALID_GAIT_ID
		)
	{		
		//try to read the gait head or return failure
		if(GaitBufferQueueLen(&gaitBufferQueue)>=sizeof(currentGaitHead)/sizeof(Uint16))
		{	
			pdata=(Uint16 *)&currentGaitHead;
			for(i=0;i<sizeof(currentGaitHead)/sizeof(Uint16);i++)
			{
				pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
				gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
			}
			pdata=pbuf;			
			for(i=0;i<(currentGaitHead.frameStep&(~0xC000));i++)
			{
				pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
				gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
			}

			if(pisLeft!=NULL)
				*pisLeft=gaitBufferQueue.q[gaitBufferQueue.f];
			gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;

			if(currentGaitHead.frameStep&0x8000)
			{
				if(pankle!=NULL)
				{
					pdata=(Uint16 *)pankle;
					for(i=0;i<sizeof(struct RigidBody)/sizeof(Uint16);i++)
					{
						pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
						gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
					}
				}else
				{
					for(i=0;i<sizeof(struct RigidBody)/sizeof(Uint16);i++)
						gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;					
				}	
			}else if(pankle!=NULL)
			{
				pankle->offset.x=INVALID_RIGIDBODY_VALUE;
				pankle->offset.y=INVALID_RIGIDBODY_VALUE;
				pankle->offset.z=INVALID_RIGIDBODY_VALUE;
				pankle->pose.alpha=INVALID_RIGIDBODY_VALUE;
				pankle->pose.beta=INVALID_RIGIDBODY_VALUE;
				pankle->pose.theta=INVALID_RIGIDBODY_VALUE;
			}

			if(currentGaitHead.frameStep&0x4000)
			{
				if(phip!=NULL)
				{
					pdata=(Uint16 *)phip;
					for(i=0;i<sizeof(struct RigidBody)/sizeof(Uint16);i++)
					{
						pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
						gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
					}
				}else
				{
					for(i=0;i<sizeof(struct RigidBody)/sizeof(Uint16);i++)
						gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;					
				}
			}else if(phip!=NULL)
			{
				phip->offset.x=INVALID_RIGIDBODY_VALUE;
				phip->offset.y=INVALID_RIGIDBODY_VALUE;
				phip->offset.z=INVALID_RIGIDBODY_VALUE;
				phip->pose.alpha=INVALID_RIGIDBODY_VALUE;
				phip->pose.beta=INVALID_RIGIDBODY_VALUE;
				phip->pose.theta=INVALID_RIGIDBODY_VALUE;
			}
			gaitStepCntr=1;
			OSSemPost(psemGaitExe);
			ret=(currentGaitHead.frameStep&(~0xC000));
		}else
		{
			//current gait is not available return proper info to caller
			memset(&currentGaitHead,0,sizeof(currentGaitHead));
			gaitStepCntr=0;
			ret=-1;
		}
		
	}else
	{
		//extract the following frame and return
		//fast gait generation is supposed or waiting is required
		Uint16 realLength=0;
		realLength=(currentGaitHead.frameStep&~(0xC000))+1;
		if(currentGaitHead.frameStep&0x8000)
		{
			realLength+=sizeof(struct RigidBody)/sizeof(Uint16);
		}
		if(currentGaitHead.frameStep&0x4000)
		{
			realLength+=sizeof(struct RigidBody)/sizeof(Uint16);
		}

		if(GaitBufferQueueLen(&gaitBufferQueue)>=realLength)
		{
			pdata=pbuf;			
			for(i=0;i<(currentGaitHead.frameStep&(~0xC000));i++)
			{
				pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
				gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
			}

			if(pisLeft!=NULL)
				*pisLeft=gaitBufferQueue.q[gaitBufferQueue.f];
			gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;

			if(currentGaitHead.frameStep&0x8000)
			{
				if(pankle!=NULL)
				{
					pdata=(Uint16 *)pankle;
					for(i=0;i<sizeof(struct RigidBody)/sizeof(Uint16);i++)
					{
						pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
						gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
					}
				}else
				{
					for(i=0;i<sizeof(struct RigidBody)/sizeof(Uint16);i++)
						gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;					
				}	
			}else if(pankle!=NULL)
			{
				pankle->offset.x=INVALID_RIGIDBODY_VALUE;
				pankle->offset.y=INVALID_RIGIDBODY_VALUE;
				pankle->offset.z=INVALID_RIGIDBODY_VALUE;
				pankle->pose.alpha=INVALID_RIGIDBODY_VALUE;
				pankle->pose.beta=INVALID_RIGIDBODY_VALUE;
				pankle->pose.theta=INVALID_RIGIDBODY_VALUE;
			}

			if(currentGaitHead.frameStep&0x4000)
			{
				if(phip!=NULL)
				{
					pdata=(Uint16 *)phip;
					for(i=0;i<sizeof(struct RigidBody)/sizeof(Uint16);i++)
					{
						pdata[i]=gaitBufferQueue.q[gaitBufferQueue.f];
						gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;
					}
				}else
				{
					for(i=0;i<sizeof(struct RigidBody)/sizeof(Uint16);i++)
						gaitBufferQueue.f=(gaitBufferQueue.f+1)%GAIT_BUFFER_QUEUE_LENGTH;					
				}
			}else if(phip!=NULL)
			{
				phip->offset.x=INVALID_RIGIDBODY_VALUE;
				phip->offset.y=INVALID_RIGIDBODY_VALUE;
				phip->offset.z=INVALID_RIGIDBODY_VALUE;
				phip->pose.alpha=INVALID_RIGIDBODY_VALUE;
				phip->pose.beta=INVALID_RIGIDBODY_VALUE;
				phip->pose.theta=INVALID_RIGIDBODY_VALUE;
			}

			gaitStepCntr++;
			OSSemPost(psemGaitExe);
			ret=(currentGaitHead.frameStep&(~0xC000));
		}else
		{
			//current frame is not available and infom caller to wait
			ret=-2;
		}
	}
	OSMutexPost(plckGaitQueue);
	return ret;
}
