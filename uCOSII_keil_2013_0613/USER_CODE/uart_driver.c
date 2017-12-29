#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "main.h"

extern OS_EVENT *plckSendPacket;
//extern OS_EVENT *plckSendDxlPacket;
extern OS_EVENT *psemTxEmpty;
extern OS_EVENT *pSemSCIBTX;
extern OS_EVENT *pMbxSciaRx;
extern OS_EVENT *pMbxScibRx;

#define UART0_BPS     115200                                            /* 串口0通信波特率             */
#define UART3_BPS     500000                                            /* 串口2通信波特率             */
#define POINTER_DATA_MASK	0xffffff00

void UART0_IRQHandler(void)
{
	INT32U iir=0;
	INT8U recv=0;
	INT8U lsr=0;

	OSIntEnter();
	iir=U0IIR;
	if(iir&0x00000001)
	{
		OSIntExit();
		return;
	}
	iir=((iir&0x0000000E)>>1);
	switch(iir)
	{
		case 0x01:								 //tx empty interrupt
			OSSemPost(psemTxEmpty);
		break;
		case 0x03:								 //rx line status interrupt
		case 0x02:								 //rx data available
		case 0x06:								 //fifo timeout interrupt	
		{
			lsr=U0LSR;							
			while(lsr&0x01)
			{
				recv=U0RBR;
				OSQPost(pMbxSciaRx,(void *)(recv+POINTER_DATA_MASK));
				lsr=U0LSR;
			}
		}
		break;
		default:
		break;
	}
	OSIntExit();
	return;
}

void UART0_Init (void)
{
	INT16U usFdiv;
    /* UART0 */
  
  	U0LCR  = 0x83;                      /* 允许设置波特率               */
	/* 波特率计算公式 UARTbps = PCLK/(16*DLest*(1+DivAddVal/MulVal))   DLest即usFdiv;U0FDR [3:0]DIVADDVAL  [7:4]MULVAL*/

    /*usFdiv = (FPCLK / 16) / UART0_BPS;        //115384bps   */
	   
	usFdiv = 7;
	U0FDR = 0xfe;		  //110837bps

    U0DLM  = usFdiv / 256;
    U0DLL  = usFdiv % 256;
    U0LCR  = 0x03;                      /* 锁定波特率                   */
    U0FCR  = 0xC7;						 //fifo enable and reset 14 char is set for fullfilling
	U0IER  = 0x07;			   			 //enable tx/rx/ls interrupt
	zyIsrSet(NVIC_UART0,(unsigned long)UART0_IRQHandler,10);
	zyIsrEnable(NVIC_UART0);
}

Uint16 SendPacket(Byte *pbuf,Uint16 len)
{
	//we assume all send is blocking and overlapped sending is forbidden
	Uint16 i=0;
	Uint16 fifoLen=0;
	INT8U err=0;

	if(pbuf==NULL||len==0)
		return 0;

	OSMutexPend(plckSendPacket,0,&err);
	do
	{
		while(!(U0LSR&0x20))
			OSSemPend(psemTxEmpty,0,&err);
		//copy the first part of packet into fifo
		if(i+0x10>len)
			fifoLen=len;
		else
			fifoLen=i+0x10;
		for(;i<fifoLen;i++)
			U0THR=pbuf[i];
	}while(i<len);
	while(!(U0LSR&0x20))
		OSSemPend(psemTxEmpty,0,&err);
	OSMutexPost(plckSendPacket);
	
	return len;
}

Byte GetChar()
{
	INT8U err=0;
	INT32U wTemp=0;

	wTemp=(INT32U)OSQPend(pMbxSciaRx,0,&err);
	if(err!=OS_NO_ERR)
		return (Byte)-1;
	return (wTemp&(~POINTER_DATA_MASK));
}

void UART3_IRQHandler(void)
{
	INT32U iir=0;
	INT8U recv=0;
	INT8U lsr=0;

	OSIntEnter();
	iir=U3IIR;
	if(iir&0x00000001)
	{
		OSIntExit();
		return;
	}

	iir=((iir&0x0000000E)>>1);
	switch(iir)
	{
		case 0x01:								 //tx empty interrupt
			OSSemPost(pSemSCIBTX);
		break;
		case 0x03:								 //rx line status interrupt
		case 0x02:								 //rx data available
		case 0x06:								 //fifo timeout interrupt	
		{
			lsr=U3LSR;
			while((lsr&0x01))
			{
				recv=U3RBR;
				OSQPost(pMbxScibRx,(void *)(recv+POINTER_DATA_MASK));
				lsr=U3LSR;
			}
		}
		break;
		default:
		break;
	}
	OSIntExit();
	return;
}


Uint16 SendDXLPacket(Byte *pbuf,Uint16 len)
{
	//we assume all send is blocking and overlapped sending is forbidden
	Uint16 i=0;
	Uint16 fifoLen=0;
	INT8U err=0;

	if(pbuf==NULL||len==0)
		return 0;

	//OSMutexPend(plckSendDxlPacket,0,&err);
	do
	{
		while(!(U3LSR&0x20))
			OSSemPend(pSemSCIBTX,0,&err);
		//copy the first part of packet into fifo
		if(i+0x10>len)
			fifoLen=len;
		else
			fifoLen=i+0x10;
		for(;i<fifoLen;i++)
			U3THR=pbuf[i];
	}while(i<len);
	while(!(U3LSR&0x20))
		OSSemPend(pSemSCIBTX,0,&err);
	//OSMutexPost(plckSendDxlPacket);
	
	return len;
}

Byte GetDXLChar()
{
	INT8U err=0;
	INT32U wTemp=0;

	wTemp=(INT32U)OSQPend(pMbxScibRx,0,&err);
	if(err!=OS_NO_ERR)
		return (Byte)-1;
	return (wTemp&(~POINTER_DATA_MASK));
}

void ChangeDXLDirection(enum DXLDirectionType type)
{
	Uint16 isTxing=0;
	switch(type)
	{
		case RS485RX:
			isTxing=U3LSR&0x40;
			while(!isTxing)
			{
				OSTimeDly(1);
				isTxing=U3LSR&0x40;
			}
			//ScibRegs.SCICTL1.bit.TXENA=0;
			FIO0CLR0=0x80;
			FIO0CLR1=0x01;
			//GpioDataRegs.GPBCLEAR.bit.GPIOB1=1;
			//GpioDataRegs.GPBCLEAR.bit.GPIOB2=1;
			//ScibRegs.SCICTL1.bit.RXENA=1;
		break;
		case RS485TX:
		case TTLTX:
			//ScibRegs.SCICTL1.bit.RXENA=0;
			FIO0SET0=0x80;
			FIO0SET1=0x01;
			//GpioDataRegs.GPBCLEAR.bit.GPIOB1=1;
			//GpioDataRegs.GPBSET.bit.GPIOB2=1;
			//ScibRegs.SCICTL1.bit.TXENA=1;
		break;
		case TTLRX:
			isTxing=U3LSR&0x40;
			while(!isTxing)
			{
				OSTimeDly(1);
				isTxing=U3LSR&0x40;
			}
			//ScibRegs.SCICTL1.bit.TXENA=0;
			FIO0SET0=0x80;
			FIO0SET1=0x01;
			//GpioDataRegs.GPBSET.bit.GPIOB1=1;
			//GpioDataRegs.GPBCLEAR.bit.GPIOB2=1;
			//ScibRegs.SCICTL1.bit.RXENA=1;		
		break;
		default:
		break;
	}
	return;
}

void UART3_Init (void)
{
  	U3LCR  = 0x83;                      /* 允许设置波特率               */
	U3DLM = 0;							//msb is set to 0
	U3DLL = 2;							//lsb is set 2
	U3FDR = 0x95; 						//fractional baudrate is (1+5/9)
    U3LCR  = 0x03;                      /* 锁定波特率                   */
    U3FCR  = 0xC7;						 //fifo enable and reset 14 char is set for fullfilling
	U3IER 	= 0x07;			   			 //enable tx/rx/ls interrupt

	FIO0DIR0=0x80;
	FIO0DIR1=0x01;
	ChangeDXLDirection(RS485TX);
	zyIsrSet(NVIC_UART3,(unsigned long)UART3_IRQHandler,10);
	zyIsrEnable(NVIC_UART3);
}

void InitSci(void)
{
	UART3_Init();
	UART0_Init();
}
