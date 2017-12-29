#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"

void InitRit(void)
{
	AD0CR=0x00200120;
	ADINTEN=0x00000000;
}

