#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "COM_packet.h"

#define COMPLETE_PACKET_RECEIVED	0
#define CHECKSUM_PACKET_ERROR		1
#define INCOMPLETE_PACKET_RECEIVED	2

//packet head creator
DSP_STATUS_PACKET *GetCOMHead(Byte *pbuf)
{
	pbuf[0]=0xff;
	pbuf[1]=0xff;
	return (DSP_STATUS_PACKET *)(pbuf+2);
}

//packet tail creator
Uint16 ConstructCOMPacket(DSP_STATUS_PACKET *p)
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

Uint16 COMPacketCheck(DSP_INST_PACKET *pRawPacket)
{
	Byte checkSum=pRawPacket->parameter[pRawPacket->length-2];
	Byte *pPacketBuffer=(Byte *)pRawPacket;
	Uint16 checkSumCal=0;
	Uint16 i=0;
	for(i=0;i<pRawPacket->length+1;i++)
		checkSumCal+=pPacketBuffer[i];
	checkSumCal=~checkSumCal;
	checkSumCal&=0x00ff;
	
	if(checkSum==checkSumCal)
		return 1;
	return 0;
}

Uint16 MakeCOMWholePacket(Byte *pbuf,Uint16 *psize,Byte tgt)
{
	Uint16 currSize=0;
	currSize=*psize;
	
	if(currSize<2)
	{
			if(tgt==0xff)
				pbuf[currSize++]=tgt;
			else
				currSize=0;
	}else if(currSize<3)
	{
			if(tgt==ID_DSP)
				pbuf[currSize++]=tgt;
			else
				currSize=0;			
	}else if(currSize<2+sizeof(DSP_INST_PACKET)-MAX_PARAM_NUM)
	{
		pbuf[currSize++]=tgt;
	}else
	{
			DSP_INST_PACKET *info;
			pbuf[currSize++]=tgt;
			info=(DSP_INST_PACKET *)(pbuf+2);
			if(currSize==info->length+4)
			{
				*psize=currSize;
				if(COMPacketCheck(info))
					return COMPLETE_PACKET_RECEIVED;
				else
					return CHECKSUM_PACKET_ERROR;
			}
	}
	*psize=currSize;
	return INCOMPLETE_PACKET_RECEIVED;
}

Uint16 WaitCOMWholePacket(Byte *pbuf,Uint16 size,Uint16 timeout)
{
	static Uint32 inter;
	Byte ret=0;
	Uint16 i=0;
	
	do
	{
		inter=OSTimeGet();
		ret=GetChar();
		if(OSTimeGet()-inter>100)
			i=0;
	}while((ret=MakeCOMWholePacket(pbuf,&i,ret))==INCOMPLETE_PACKET_RECEIVED);
	if(ret==CHECKSUM_PACKET_ERROR)
		return 0xffff;
	return i;
}

Uint16 HandleOtherState(struct StateSwapInput *pin,struct StateSwapOutput *pout)
{
	if(pin->ctrReg1&SENSOR_ENABLE_VALID)
	{
		EnableADCInterrupt(TRUE);
		pout->stsReg1|=SENSOR_ENABLE_VALID;
	}else
	{
		EnableADCInterrupt(FALSE);
		pout->stsReg1&=~SENSOR_ENABLE_VALID;
	}
		
	if(pin->ctrReg1&TORQUE_ENABLE_VALID)
	{
		ServoTorqueEnable(TRUE);
		pout->stsReg1|=	TORQUE_ENABLE_VALID;
	}else
	{
		ServoTorqueEnable(FALSE);
		pout-> stsReg1&=~TORQUE_ENABLE_VALID;
	}
	return 0;
}

Uint16 HandleInstPacket(Byte *pbuf,Uint16 size)
{
	DSP_INST_PACKET *ppacket=NULL;
	DSP_STATUS_PACKET *pret=NULL;
	Uint16 ret=0;
	static Uint16 fs=0xffff;
	Uint16 len=0;
	Uint16 i=0;
	Uint16 *pdata=NULL;
			
	ppacket=(DSP_INST_PACKET *)(pbuf+2);
	pret=(DSP_STATUS_PACKET *)(pbuf+2);
	

	switch(ppacket->instruction)
	{
		case INST_CONNECTION_VALID:
		{
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_TORQUE_ON:
		{	
			ServoTorqueEnable(TRUE);										
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_TORQUE_OFF:
		{
			ServoTorqueEnable(FALSE);
			
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);			
		}
		break;
		case INST_INITIAL_STATE:
		case INST_MULTIPLE_ACTION:
		{			
			//if torque is not applied there is no execution
			//this infomation should be aquired from robot state
			//which complied with sensor feedback and other needed state
			
			/*
			len=((ppacket->length-2-1)>>1);
			for(i=0;i<len;i++)
				ppacket->parameter[i+1]=ppacket->parameter[2*i+1]+(ppacket->parameter[2*i+2]<<8);
			*/
			pdata=(Uint16 *)(ppacket->parameter);
			len=((ppacket->length-2-1)>>1);
			for(i=len;i>0;i--)
				pdata[i]=(Uint16)(ppacket->parameter[2*i-1])+((Uint16)(ppacket->parameter[2*i])<<8);
			pdata[0]=(Uint16)(ppacket->parameter[0]);

			if(ppacket->instruction==INST_INITIAL_STATE)
				ExecuteConfiguration((Uint16 *)(ppacket->parameter),len+1);
			else
				SmoothConfiguration((Uint16 *)(ppacket->parameter),len+1);
			
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_SINGLE_ACTION:
		{	
			for(i=0;i<ppacket->length-2;i+=3)
			{
				TakeSingleAction(
					(Uint16)ppacket->parameter[i]+1,
					(Uint16)ppacket->parameter[i+1]+(ppacket->parameter[i+2]<<8),
					DEFAULT_TURNING_SPEED
					);
			}
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_GAIT_MEMORY_START:
		{
			DownloadGaitHead *phead=NULL;
			//to prevent previous version of motion debug downloading
			if(ppacket->length<2+sizeof(DownloadGaitHead)/sizeof(Uint16))
				break;
			
			phead=(DownloadGaitHead *)(ppacket->parameter);
			phead->effect.thetaOffset=ppacket->parameter[7];
			phead->effect.yOffset=ppacket->parameter[6];
			phead->effect.xOffset=ppacket->parameter[5];
			phead->period=ppacket->parameter[3]+(ppacket->parameter[4]<<8);
			phead->frameStep=ppacket->parameter[2];
			phead->frameLength=ppacket->parameter[1];
			phead->gaitID=ppacket->parameter[0];
						
			CloseAll();
			fs=Open(ppacket->parameter[0],FILE_WRITE_ONLY);
			Write(fs,(Uint16 *)(ppacket->parameter),sizeof(DownloadGaitHead)/sizeof(Uint16));
			
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_ADD_SINGLE_GAIT:
		{	
			//if torque is not applied there is no execution
			//this infomation should be aquired from robot state
			//which complied with sensor feedback and other needed state
			/*
			len=((ppacket->length-2-1)>>1);
			for(i=0;i<len;i++)
				ppacket->parameter[i+1]=ppacket->parameter[2*i+1]+(ppacket->parameter[2*i+2]<<8);
			*/

			pdata=(Uint16 *)(ppacket->parameter);
			len=((ppacket->length-2-1)>>1);
			for(i=len;i>0;i--)
				pdata[i]=(Uint16)(ppacket->parameter[2*i-1])+((Uint16)(ppacket->parameter[2*i])<<8);
			pdata[0]=(Uint16)(ppacket->parameter[0]);

			Write(fs,(Uint16 *)ppacket->parameter,len+1);
			
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_ADD_DATA_PATCH:
		{
			/*
			len=((ppacket->length-2)>>1);
			for(i=0;i<len;i++)
				ppacket->parameter[i]=ppacket->parameter[2*i]+(ppacket->parameter[2*i+1]<<8);
			*/
			
			pdata=(Uint16 *)(ppacket->parameter);
			len=((ppacket->length-2)>>1);
			for(i=0;i<len;i++)
				pdata[i]=(Uint16)(ppacket->parameter[2*i])+((Uint16)(ppacket->parameter[2*i+1])<<8);

			Write(fs,(Uint16 *)ppacket->parameter,len);
			
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_FLASH_PROGRAM:
		{
			Close(fs);
			fs=0xffff;
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_GAIT_COMMAND:
		{
			ExecuteGait(ppacket->parameter[0],ppacket->parameter[1]);
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_GAIT_DIRECTION:
		break;
		case INST_GAIT_DIRECTION_EXT:
		{
			struct GaitEffect *peffect;
			peffect=(struct GaitEffect *)(ppacket->parameter);
			peffect->xOffset=(int16)(ppacket->parameter[0]+(ppacket->parameter[1]<<8));
			peffect->yOffset=(int16)(ppacket->parameter[2]+(ppacket->parameter[3]<<8));
			peffect->thetaOffset=(int16)(ppacket->parameter[4]+(ppacket->parameter[5]<<8));
			ExecuteGaitDirectionExt(peffect);
			
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_START_INCLINOMETER_FEEDBACK:
		{
			EnableADCInterrupt(TRUE);
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_STOP_INCLINOMETER_FEEDBACK:
		{
			EnableADCInterrupt(FALSE);
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_INCLINOMETER_REQUIRED:
		{
			/*
			struct Sensors *ps;
			ps=(struct Sensors *)pret->parameter;
			pret->parameter[0]=90;
			pret->parameter[1]=90;
			GetSensorResult(ps);
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=8;
			ret=ConstructCOMPacket(pret);
			*/
			
			struct SensorsRaw *ps;
			ps=(struct SensorsRaw *)pret->parameter;
			GetSensorRawResult(ps);
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=sizeof(struct SensorsRaw)+2;
			ret=ConstructCOMPacket(pret);			
						
		}
		break;
		case INST_STATE_SWAP:
		{
			struct StateSwapInput *pinput;
			struct StateSwapOutput *poutput;

			pdata=(Uint16 *)(ppacket->parameter);
			len=((ppacket->length-2)>>1);
			for(i=0;i<len;i++)
				pdata[i]=(Uint16)(ppacket->parameter[2*i])+((Uint16)(ppacket->parameter[2*i+1])<<8);

			pinput=(struct StateSwapInput *)(ppacket->parameter);
			poutput=(struct StateSwapOutput *)(ppacket->parameter+MAX_PARAM_NUM/2);
			memset(poutput,0,sizeof(struct StateSwapOutput));
			
			HandleGaitInst(pinput,poutput);
			HandleOdometer(pinput,poutput);
			HandleSensorFeedback(pinput,poutput);
			HandleHeadMoving(pinput,poutput);
			HandleOtherState(pinput,poutput);
			HandleTorsoPose(pinput,poutput);
			
			for(i=0;i<sizeof(struct StateSwapOutput);i++)
			{
				//pret->parameter[2*i]=ppacket->parameter[MAX_PARAM_NUM/2+i]&0xff;
				//pret->parameter[2*i+1]=(ppacket->parameter[MAX_PARAM_NUM/2+i]>>8)&0xff;
				pret->parameter[i]=ppacket->parameter[MAX_PARAM_NUM/2+i];
			}

			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2+sizeof(struct StateSwapOutput);
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_CALIBRATE_AHRS:
		{
			struct SensorCalibration *psc;
			psc=(struct SensorCalibration *)pret->parameter;
			StoreCalibrationToFlash(psc);
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=2;
			ret=ConstructCOMPacket(pret);
		}
		break;
		case INST_GAIT_STABLIZATION_VISUALIZE:
		{
			//send the upper machine about the current pose , supporting leg, as well as the current pose sensor
			/*
			struct Sensors *ps;
			ps=(struct Sensors *)pret->parameter;
			pret->parameter[0]=90;
			pret->parameter[1]=90;
			GetSensorResult(ps);
			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=8;
			ret=ConstructCOMPacket(pret);
			*/
			
			struct Sensors *ps;
			struct RigidBody *pr;
			pdata=(Uint16 *)pret->parameter;
			pr=(struct RigidBody *)(pret->parameter+sizeof(Uint16));
			ps=(struct Sensors *)(pret->parameter+sizeof(Uint16)+sizeof(struct RigidBody));
			
			GetSensorResult(ps);
			GetGeneratedRobotpose(pdata,pr);

			//return result to the upper machine
			pret->infomation=ppacket->instruction|PACKET_TYPE_MASK;
			pret->length=sizeof(Uint16)+sizeof(struct RigidBody)+sizeof(struct Sensors)+2;
			ret=ConstructCOMPacket(pret);
		}
		break;		
		default:
		break;
	}

	return ret;
}





