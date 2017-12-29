/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.c
** Last modified Date:  2009-05-12
** Last Version:        V1.01
** Descriptions:        The main() function example template
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Chengmingji
** Created date:        2009-07-24
** Version:             V1.00
** Descriptions:        添加用户应用程序
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         Li Baihua
** Modified date:       2009-07-27
** Version:
** Descriptions:        编写示例代码
**
** Rechecked by:
*********************************************************************************************************/

#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "COM_packet.h"

/*********************************************************************************************************
  宏定义
*********************************************************************************************************/
#define     BEEP            (0x01 << 1)                                  /*  LED1    P3.25               */
#define     BEEP_INIT()     FIO3DIR3 |= BEEP                             /*  设置P3.25端口为输出                    */
#define     BEEP_OFF()      FIO3SET3 = BEEP                              /*  蜂鸣器关                    */
#define     BEEP_ON()       FIO3CLR3 = BEEP                              /*  蜂鸣器开                    */

#define 	BEEP2			(0x01 << 2)									  /*  LED2   P3.26                */
#define     BEEP2_INIT()     FIO3DIR3 |= BEEP2                            /*  蜂鸣器关                    */
#define     BEEP2_OFF()      FIO3SET3 = BEEP2                             /*  蜂鸣器关                    */
#define     BEEP2_ON()       FIO3CLR3 = BEEP2                             /*  蜂鸣器开                    */
/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
OS_STK stkMainTask[196];
OS_STK stkComTask[196];
OS_STK stkDxlTask[196];
OS_STK stkAdcTask[256];
OS_STK stkGtgTask[384];

OS_EVENT *plckSendPacket;
OS_EVENT *plckSendDxlPacket;
OS_EVENT *psemTxEmpty;
OS_EVENT *pSemSCIBTX;
OS_EVENT *pMbxSciaRx;
OS_EVENT *pMbxScibRx;
OS_EVENT *pSemSPIA;
OS_EVENT *pSemADC;
OS_EVENT *psemDXL;
OS_EVENT *plckSensorAquire;
OS_EVENT *psemGaitGen;
OS_EVENT *psemGaitExe;
OS_EVENT *plckUpdateCommand;
OS_EVENT *plckGaitQueue;
OS_EVENT *psemI2C0TXEmpty;

void *rxArrayOfMsg[16];
void *rxbArrayOfMsg[16];
Byte packet[sizeof(DSP_INST_PACKET)+2];
/*********************************************************************************************************
** Function name:       mainTask
** Descriptions:        主任务
** input parameters:    pvData: 没有使用
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mainTask(void *pvData)
{
	Uint16 cntr=0;
    pvData = pvData;
    
    BEEP_INIT();
    BEEP_OFF();
    
    while (TRUE) 
	{
		OSTimeDly(10);
		HandleInterrupt();
		cntr++;
		if(cntr==50)
			BEEP_ON();
		else if(cntr==100)
		{
			BEEP_OFF();
			cntr=0;
		}
    }
}

void task_adc(void *pvData)
{
	pvData = pvData;
    InitSensor();
    	
    while(TRUE)
    {
    	WaitADCInterrupt();
    	HandleSensor();
    }
}

void task_gaitgen(void *pvData)
{
	pvData = pvData;
    
    InitGaitGeneration();
    while(TRUE)
    {
    	GenerateGaitStep();
    }
}

void task_dxl(void *pvData)
{
	pvData = pvData;

	InitDxlControl();    
    while(TRUE)
    {
    	WaitDXLInterrupt();
    	EnableTimerInterrupt(HandleDXLControl());
    }
}

void task_com(void *pvData)
{
	Uint16 ret=0;
	memset(packet,0,sizeof(packet));
	pvData = pvData;
/*
    BEEP2_INIT();
    BEEP2_OFF();	
*/
	while(TRUE)
	{

		ret=WaitCOMWholePacket(packet,sizeof(packet),0);
		if((ret!=0xffff)&&0<(ret=HandleInstPacket(packet,ret)))
			SendPacket(packet,ret);

/*		strcpy(packet,"zhangjiwen is a proficient embedded system designer\n");
		ret=strlen(packet);
		SendPacket(packet,ret);
		OSTimeDly(OS_TICKS_PER_SEC*2);
	
		OSTimeDly(OS_TICKS_PER_SEC / 3);
        BEEP2_ON();
        OSTimeDly(OS_TICKS_PER_SEC / 3);
        BEEP2_OFF();	
	
*/	
	}


/*	Uint16 i=0;
	Uint16 page=0;
	#define PAGES_BUFFERED			2
	extern Uint16 flashBuffer[SINGLE_PAGE_SIZE*PAGES_BUFFERED];

	while(TRUE)
	{
		page=2;
		SpiFlashRead(page*128,flashBuffer,128);
		i=128;

		while(i--)
			flashBuffer[i]=128-i;		
		SpiFlashProgram(page*128,flashBuffer,128);
		memset(flashBuffer,0,sizeof(flashBuffer));
		SpiFlashRead(page*128,flashBuffer,128);
		OSTimeDly(1000);
	}
*/
/*	Uint16 result[16];
	Uint16 nbytes;
	while(TRUE)
	{
		nbytes=ReadADCResult(result,sizeof(result));
		nbytes=nbytes;
		OSTimeDly(10);
	}
*/	
}

/*********************************************************************************************************
** Function name:       main
** Descriptions:        用户程序入口函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
int main(void)
{
	INT8U err=0;

    targetInit();
    pinInit();
	pconpInit();
	InitSci();
	SSP0_init();
	InitAdc();
	I2CInit();
	InitFileSystem();

    OSInit();
	
	plckSendPacket=OSMutexCreate(16,&err);
	plckSendDxlPacket=OSMutexCreate(11,&err);
	psemTxEmpty=OSSemCreate(0);
	psemI2C0TXEmpty=OSSemCreate(0);
	pSemSCIBTX=OSSemCreate(0);
	pMbxSciaRx=OSQCreate(rxArrayOfMsg,sizeof(rxArrayOfMsg)/sizeof(void *));
	pMbxScibRx=OSQCreate(rxbArrayOfMsg,sizeof(rxbArrayOfMsg)/sizeof(void *));
	pSemSPIA=OSSemCreate(0);
	pSemADC=OSSemCreate(0);
	psemDXL=OSSemCreate(0);
	plckSensorAquire=OSMutexCreate(13,&err);
	psemGaitGen=OSSemCreate(0);
	psemGaitExe=OSSemCreate(0);
	plckUpdateCommand=OSMutexCreate(15,&err);
	plckGaitQueue=OSMutexCreate(10,&err);

    OSTaskCreate(mainTask, (void *)0, &stkMainTask[sizeof(stkMainTask) / sizeof(OS_STK) - 1], 20);
	OSTaskCreate(task_dxl, (void *)0, &stkDxlTask[sizeof(stkDxlTask) / sizeof(OS_STK) - 1], 21);
	OSTaskCreate(task_adc, (void *)0, &stkAdcTask[sizeof(stkAdcTask) / sizeof(OS_STK) - 1], 25);
	OSTaskCreate(task_com, (void *)0, &stkComTask[sizeof(stkComTask) / sizeof(OS_STK) - 1], 23);
	OSTaskCreate(task_gaitgen, (void *)0, &stkGtgTask[sizeof(stkGtgTask) / sizeof(OS_STK) - 1], 24);

    OSStart();
	return 0;
}

/*********************************************************************************************************
**  End Of File
*********************************************************************************************************/
