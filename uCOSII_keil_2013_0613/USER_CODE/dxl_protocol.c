#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "DXL_packet.h"
#include "main.h"

#define COMPLETE_PACKET_RECEIVED	0
#define CHECKSUM_PACKET_ERROR		1
#define INCOMPLETE_PACKET_RECEIVED	2

struct CurrentBodyState
{
	struct RigidBody currentHip;
	Uint16 isLeft;
};

extern OS_EVENT *plckSendDxlPacket;
Bool isRegisterCommandSent=FALSE;
Uint16 cmd[MAX_ROBOT_JOINTS*2];
Byte dxlPacket[sizeof(struct DXL_INST_PACKET)+2];
Bool isTorqueOn=FALSE;

struct CurrentBodyState currentBodyState;

void InitDxlControl(void)
{
	currentBodyState.isLeft=INVALID_RIGIDBODY_VALUE;
	currentBodyState.currentHip.offset.x=INVALID_RIGIDBODY_VALUE;
	currentBodyState.currentHip.offset.y=INVALID_RIGIDBODY_VALUE;		
	currentBodyState.currentHip.offset.z=INVALID_RIGIDBODY_VALUE;
	currentBodyState.currentHip.pose.alpha=INVALID_RIGIDBODY_VALUE;
	currentBodyState.currentHip.pose.beta=INVALID_RIGIDBODY_VALUE;
	currentBodyState.currentHip.pose.theta=INVALID_RIGIDBODY_VALUE;
}

//packet head creator
struct DXL_INST_PACKET *GetDXLHead(Byte *pbuf)
{
	pbuf[0]=0xff;
	pbuf[1]=0xff;
	return (struct DXL_INST_PACKET *)(pbuf+2);
}

//packet tail creator
Uint16 ConstructDXLPacket(struct DXL_INST_PACKET *p)
{
	Byte *pdata=NULL;
	Uint16 i=0;
	Uint16 checkSum=0;
	
	pdata=(Byte *)p;
	for(i=0;i<p->length+1;i++)
	{
		checkSum+=pdata[i];
	}
	checkSum=~checkSum;
	checkSum&=0x00ff;
	pdata[p->length+1]=checkSum;
	return p->length+4;
}

Uint16 DXLPacketCheck(struct DXL_STATUS_PACKET *pRawPacket)
{
	Byte checkSum=pRawPacket->parameter[pRawPacket->length-2];
	Byte *pPacketBuffer=(Byte *)pRawPacket;
	Byte checkSumCal=0;
	Uint16 i=0;
	for(i=0;i<pRawPacket->length+1;i++)
		checkSumCal+=pPacketBuffer[i];
	checkSumCal=~checkSumCal;
	checkSumCal&=0x00ff;
	
	if(checkSum==checkSumCal)
		return 1;
	return 0;
}

Uint16 MakeDXLWholePacket(Byte *pbuf,Uint16 *psize,Byte tgt)
{
	Uint16 currSize=0;
	currSize=*psize;
	
	if(currSize<2)
	{
			if(tgt==0xff)
				pbuf[currSize++]=tgt;
			else
				currSize=0;
	}else if(currSize<=2+sizeof(struct DXL_STATUS_PACKET)-MAX_DXL_PARAM_NUM)
	{
		pbuf[currSize++]=tgt;
	}else
	{
			struct DXL_STATUS_PACKET *info;
			pbuf[currSize++]=tgt;
			info=(struct DXL_STATUS_PACKET *)(pbuf+2);
			if(currSize==info->length+4-1)
			{
				*psize=currSize;
				if(DXLPacketCheck(info))
					return COMPLETE_PACKET_RECEIVED;
				else
					return CHECKSUM_PACKET_ERROR;
			}
	}
	*psize=currSize;
	return INCOMPLETE_PACKET_RECEIVED;
}

Uint16 WaitDXLWholePacket(Byte *pbuf,Uint16 size,Uint16 timeout)
{
	Uint16 ret=0;
	Uint16 i=0;
	do
	{
		ret=GetDXLChar();
	}while((ret=MakeDXLWholePacket(pbuf,&i,ret))==INCOMPLETE_PACKET_RECEIVED);
	return 0;
}

Bool HandleDXLControl(void)
{
	struct DXL_INST_PACKET *pdxl=NULL;
	Uint16 len=0;
	Uint16 i=0;
	int16 ret=0;
	INT8U err=0;
	
	if(isRegisterCommandSent)
	{
		//sent register command execution
		OSMutexPend(plckSendDxlPacket,0,&err);
		if(isTorqueOn)
		{
			ChangeDXLDirection(RS485TX);
			pdxl=GetDXLHead(dxlPacket);
			pdxl->id=BROADCASTING_ID;
			pdxl->instruction=INST_ACTION;
			pdxl->length=2;
			len=ConstructDXLPacket(pdxl);
			SendDXLPacket(dxlPacket,len);
		}
		OSMutexPost(plckSendDxlPacket);
	}
	
	//while(-2==(ret=GetNextConfiguration(cmd,sizeof(cmd))));
	while(-2==(ret=GetNextConfigurationWithPose(cmd,sizeof(cmd),NULL,&currentBodyState.currentHip,&currentBodyState.isLeft)));
	//gait command is compose of target position and target speed
	if(ret>0)
	{
		//send each packet to servos
		OSMutexPend(plckSendDxlPacket,0,&err);
		if(isTorqueOn)
		{
			ChangeDXLDirection(RS485TX);
			for(i=0;i<(ret>>1);i++)
			{
				pdxl=GetDXLHead(dxlPacket);
				pdxl->id=i+1;
				pdxl->instruction=INST_REG_WRITE;
				pdxl->length=7;
				pdxl->parameter[0]=P_GOAL_POSITION_L;
				pdxl->parameter[1]=cmd[i]&0x00ff;
				pdxl->parameter[2]=(cmd[i]>>8)&0x00ff;
				pdxl->parameter[3]=cmd[i+(ret>>1)]&0x00ff;
				pdxl->parameter[4]=(cmd[i+(ret>>1)]>>8)&0x00ff;
				len=ConstructDXLPacket(pdxl);
				SendDXLPacket(dxlPacket,len);
			}
		}
		OSMutexPost(plckSendDxlPacket);
		isRegisterCommandSent=TRUE;
	}else if(ret==-1)
	{
		isRegisterCommandSent=FALSE;
	}
	return (isRegisterCommandSent);
}

Uint16 HandleTorsoPose(struct StateSwapInput *pin,struct StateSwapOutput *pout)
{
	pout->isLeft=currentBodyState.isLeft;
	pout->torsoPose.offset.x=currentBodyState.currentHip.offset.x;
	pout->torsoPose.offset.y=currentBodyState.currentHip.offset.y;
	pout->torsoPose.offset.z=currentBodyState.currentHip.offset.z;
	pout->torsoPose.pose.alpha=currentBodyState.currentHip.pose.alpha;
	pout->torsoPose.pose.beta=currentBodyState.currentHip.pose.beta;
	pout->torsoPose.pose.theta=currentBodyState.currentHip.pose.theta;
	return 1;
}

Uint16 GetGeneratedRobotpose(Uint16 *pdata, struct RigidBody *pr)
{
	*pdata=currentBodyState.isLeft;
	pr->offset.x=currentBodyState.currentHip.offset.x;
	pr->offset.y=currentBodyState.currentHip.offset.y;
	pr->offset.z=currentBodyState.currentHip.offset.z;
	pr->pose.alpha=currentBodyState.currentHip.pose.alpha;
	pr->pose.beta=currentBodyState.currentHip.pose.beta;
	pr->pose.theta=currentBodyState.currentHip.pose.theta;
	return 1;
}

Uint16 TakeSingleAction(Uint16 id,Uint16 pos,Uint16 vel)
{
	struct DXL_INST_PACKET *pdxl=NULL;
	Uint16 len=0;
	INT8U err=0;
	
	pdxl=GetDXLHead(dxlPacket);	
	
	OSMutexPend(plckSendDxlPacket,0,&err);
	if(isTorqueOn)
	{
		ChangeDXLDirection(RS485TX);
		pdxl->id=id;
		pdxl->instruction=INST_WRITE;
		pdxl->length=7;
		pdxl->parameter[0]=P_GOAL_POSITION_L;
		pdxl->parameter[1]=(pos&0x00ff);
		pdxl->parameter[2]=((pos>>8)&0x00ff);
		pdxl->parameter[3]=(vel&0x00ff);
		pdxl->parameter[4]=((vel>>8)&0x00ff);
		len=ConstructDXLPacket(pdxl);
		SendDXLPacket(dxlPacket,len);
	}
	OSMutexPost(plckSendDxlPacket);

	return 0;
}

Uint16 ServoTorqueEnable(Bool enable)
{
	struct DXL_INST_PACKET *pdxl=NULL;
	Uint16 len=0;
	INT8U err=0;

	if(enable==isTorqueOn)
		return 0;

	OSMutexPend(plckSendDxlPacket,0,&err);
	if(enable)
	{
		ChangeDXLDirection(RS485TX);
		pdxl=GetDXLHead(dxlPacket);
		pdxl->id=BROADCASTING_ID;
		pdxl->instruction=INST_WRITE;
		pdxl->length=4;
		pdxl->parameter[0]=P_TORQUE_ENABLE;
		pdxl->parameter[1]=TORQUE_ON;
		len=ConstructDXLPacket(pdxl);
		SendDXLPacket(dxlPacket,len);

		pdxl=GetDXLHead(dxlPacket);
		pdxl->id=BROADCASTING_ID;
		pdxl->instruction=INST_WRITE;
		pdxl->length=7;
		pdxl->parameter[0]=P_CW_COMPLIANCE_MARGIN;
		pdxl->parameter[1]=0;
		pdxl->parameter[2]=0;
		pdxl->parameter[3]=24;
		pdxl->parameter[4]=24;
		len=ConstructDXLPacket(pdxl);
		SendDXLPacket(dxlPacket,len);
		
		pdxl=GetDXLHead(dxlPacket);
		pdxl->id=BROADCASTING_ID;
		pdxl->instruction=INST_WRITE;
		pdxl->length=5;
		pdxl->parameter[0]=P_PUNCH_L;
		pdxl->parameter[1]=32;
		pdxl->parameter[2]=0;
		len=ConstructDXLPacket(pdxl);
		SendDXLPacket(dxlPacket,len);
	}else
	{
		ChangeDXLDirection(RS485TX);
		pdxl=GetDXLHead(dxlPacket);
		pdxl->id=BROADCASTING_ID;
		pdxl->instruction=INST_WRITE;
		pdxl->length=4;
		pdxl->parameter[0]=P_TORQUE_ENABLE;
		pdxl->parameter[1]=TORQUE_OFF;
		len=ConstructDXLPacket(pdxl);
		SendDXLPacket(dxlPacket,len);
	}
	isTorqueOn=enable;
	OSMutexPost(plckSendDxlPacket);
	return 0;
}


