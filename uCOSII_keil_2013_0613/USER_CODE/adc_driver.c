#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"

void InitAdc(void)
{
	AD0CR=0x00200130;
	ADINTEN=0x00000000;
}

Uint16 ReadADCResult(Uint16 *pbuf,Uint16 size)
{
	volatile Uint32 *presult=NULL;
	Uint16 i=0;
	//start ADC convertion
	AD0CR=0x00210130;
	OSTimeDly(2);
	presult=&ADDR0;

	while(i<size&&i<8)
	{
		if(presult[i]&0x80000000)
			pbuf[i]=(Uint16)((presult[i]&0xfff0)>>4);
		else
			pbuf[i]=0;
		i++;
	}
	AD0CR=0x00200130;

	return i;
}
