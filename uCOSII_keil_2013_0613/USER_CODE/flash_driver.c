#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "main.h"

#define CS_Low()  FIO0CLR2=0x01;
#define CS_High() FIO0SET2=0x01;

#define MAIN_MEMORY_PAGE_READ	0x52
#define MAIN_MEMORY_PROGRAM_1	0x82
#define MAIN_MEMORY_PROGRAM_2	0x85
#define PAGE_PROGRAM_BUFFER_1	0x83
#define PAGE_PROGRAM_BUFFER_2	0x86
#define PAGE_ERASE				0x81
#define READ_BUFFER_1			0x54
#define READ_BUFFER_2			0x56
#define WRITE_BUFFER_1			0x84
#define STATUS_REGISTER_READ	0xD7
#define SINGLE_PAGE_SIZE		128
#define WORD_ADDRESS_MASK 		0x0000007f
#define PAGE_ADDRESS_MASK 		0x0007ff80
#define MAX_HEAD_SIZE 			4

#define PAGES_BUFFERED			2
Uint16 flashBuffer[SINGLE_PAGE_SIZE*PAGES_BUFFERED];
extern OS_EVENT *pSemSPIA;

void SSP0_IRQHandler(void)
{
	OSIntEnter();
	SSP0ICR=0x0003;
	SSP0IMSC=0x00000000;
	OSSemPost(pSemSPIA);
	OSIntExit();
	return;
}

void SSP0_init(void)	  //初始化SPI
{
	FIO0DIR2=0x01;
	CS_High();
	//flash_ready is default input pin

	SSP0CR0=0x000000CF;
	SSP0CR1=0x00000000;	
	SSP0CPSR=0x00000004;
	SSP0IMSC=0x00000000;	
	SSP0ICR=0x00000002;
	SSP0CR1=0x00000002;

	zyIsrSet(NVIC_SSP0,(unsigned long)SSP0_IRQHandler,11);
	zyIsrEnable(NVIC_SSP0);
}

Uint16 WriteFlashPage(Uint32 startAddr,Uint16 *pbuf,Uint16 size)
{
	Uint32 wDeAddress=0;
	Uint16 wFlashCmdHead[MAX_HEAD_SIZE];
	Uint16 i=0;
	Uint16 fifoLen=0;
	Uint16 charBuf=0;
	INT8U err=0;
	
	wDeAddress=MAIN_MEMORY_PROGRAM_1;
	wDeAddress<<=24;
	wDeAddress&=0xff000000;
	wDeAddress|=((startAddr&PAGE_ADDRESS_MASK)<<3);
	wDeAddress|=((startAddr&WORD_ADDRESS_MASK)<<1);

	//生成spi flash操作指令
	wFlashCmdHead[0]=(wDeAddress>>16)&0xffff;
	wFlashCmdHead[1]=(wDeAddress&0xffff);
		
	//query whether this is valid without edis
	CS_Low();
	do
	{
		if(size+2-i>8)
			fifoLen=8;
		else
			fifoLen=size+2-i;

		while((SSP0SR&0x02)&&fifoLen>0)
		{
				if(i<2)
					SSP0DR=wFlashCmdHead[i];
				else
					SSP0DR=pbuf[i-2];
				i++;
				fifoLen--;
		}
	
		//wait until semophore and exit
		SSP0ICR=0x0003;
		SSP0IMSC=0x00000002;
		OSSemPend(pSemSPIA,0,&err);	

		while(SSP0SR&0x04)
		{
			charBuf=SSP0DR;
			charBuf=charBuf;
		}
	}while(i<size+2);
	CS_High();	

	//sleep for programming
	OSTimeDly(20);
	
	return size;
}

Uint16 ReadFlashPage(Uint32 startAddr,Uint16 *pbuf,Uint16 size)
{
	Uint32 wDeAddress=0;
	Uint16 wFlashCmdHead[MAX_HEAD_SIZE];
	Uint16 i=0;
	Uint16 fifoLen=0;
	Uint16 readLen=0;
	Uint16 charBuf=0;
	INT8U err=0;
	
	wDeAddress=MAIN_MEMORY_PAGE_READ;
	wDeAddress<<=24;
	wDeAddress&=0xff000000;
	wDeAddress|=((startAddr&PAGE_ADDRESS_MASK)<<3);
	wDeAddress|=((startAddr&WORD_ADDRESS_MASK)<<1);

	//生成spi flash操作指令
	wFlashCmdHead[0]=(wDeAddress>>16)&0xffff;
	wFlashCmdHead[1]=(wDeAddress&0xffff);
	wFlashCmdHead[2]=0xffff;
	wFlashCmdHead[3]=0xffff;

	//query whether this is valid without edis
	CS_Low();
	do
	{
		//copy the first part of packet into fifo
		if(size+4-i>8)
			fifoLen=8;
		else
			fifoLen=size+4-i;

		while((SSP0SR&0x02)&&fifoLen>0)
		{
				if(i<4)
					SSP0DR=wFlashCmdHead[i];
				else
					SSP0DR=0xffff;
				i++;
				fifoLen--;
		}
		
		//wait until semophore and exit
		SSP0ICR=0x0003;
		SSP0IMSC=0x00000002;
		OSSemPend(pSemSPIA,0,&err);	
		//clear all recv buffer of spi
		while(SSP0SR&0x04)
		{
			charBuf=SSP0DR;
			if(readLen>=4)
				pbuf[readLen-4]=charBuf;
			readLen++;
		}
	}while(i<size+4);
	CS_High();	
		
	return readLen-4;
}

void SpiFlashProgram(Uint32 address,Uint16 *buffer,Uint16 size)
{
	Uint16 wPageSize=0;
	Uint16 wOffset=0;
	
	//生成写入首地址
	while(wOffset<size)
	{
		wPageSize=(size-wOffset>SINGLE_PAGE_SIZE)?SINGLE_PAGE_SIZE:size-wOffset;
		wOffset+=WriteFlashPage(address+wOffset,buffer+wOffset,wPageSize);
	}
	
	return;
}

void SpiFlashRead(Uint32 address,Uint16 *buffer,Uint16 size)
{
	Uint16 wPageSize=0;
	Uint16 wOffset=0;
	
	//生成写入首地址
	while(wOffset<size)
	{
		wPageSize=(size-wOffset>SINGLE_PAGE_SIZE)?SINGLE_PAGE_SIZE:size-wOffset;
		wOffset+=ReadFlashPage(address+wOffset,buffer+wOffset,wPageSize);
	}
	
	return;
}


