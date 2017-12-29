#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "DXL_packet.h"
#include "main.h"

Bool isTimerInterruptEnable=FALSE;
Bool isADCInterruptEnable=TRUE;

extern OS_EVENT *pSemADC;
extern OS_EVENT *psemDXL;

void WaitADCInterrupt(void)				
{
	INT8U err=0;
	OSSemPend(pSemADC,0,&err);
}

void WaitDXLInterrupt(void)				
{
	INT8U err=0;
	OSSemPend(psemDXL,0,&err);
}

void HandleInterrupt(void)
{
	if(isADCInterruptEnable)
		OSSemPost(pSemADC);
	if(isTimerInterruptEnable)
		OSSemPost(psemDXL);
}


