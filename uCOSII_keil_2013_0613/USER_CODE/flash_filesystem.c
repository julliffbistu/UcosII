#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "main.h"

#define MAX_BUFFERED_PAGES		3
#define MAX_FILE_POINTERS		2

#define INVALID_FILE_POINTER	0xffff
#define INVALID_BLOCK_NUMBER	0xffff
#define FILE_FIXED_PAGES		10

struct Block
{
	Uint16 buffer[SINGLE_PAGE_SIZE];
	Uint16 flag;
	Uint16 cntr;
	Uint16 block;
};

struct File
{
	Uint16 startBlock;
	Uint16 current;
	Uint16 offset;
	Uint16 flag;
};

struct Block blocks[MAX_BUFFERED_PAGES];
struct File fp[MAX_FILE_POINTERS];

void InitFileSystem()
{
	Uint16 i=0;
	memset(blocks,0,sizeof(blocks));
	for(i=0;i<MAX_BUFFERED_PAGES;i++)
		blocks[i].block=INVALID_BLOCK_NUMBER;
	memset(fp,0,sizeof(fp));
	for(i=0;i<MAX_FILE_POINTERS;i++)
		fp[i].startBlock=INVALID_FILE_POINTER;
}

int16 Open(Uint16 id,Uint16 flag)
{
	int16 i=0;
	for(i=0;i<MAX_FILE_POINTERS;i++)
	{
		if(fp[i].startBlock==INVALID_FILE_POINTER)
		{
			fp[i].startBlock=id*FILE_FIXED_PAGES;
			fp[i].current=fp[i].startBlock;
			fp[i].offset=0;
			fp[i].flag=flag;
			break;
		}
	}	
	return (i<MAX_FILE_POINTERS?i:-1);
}

int16 Read(Uint16 file,Uint16 *pbuf,Uint16 len)
{
	Uint16 i=0;
	Uint32 addr=0;
	if(fp[file].startBlock==INVALID_FILE_POINTER)
		return -1;
	while(len>0)
	{
		for(i=0;i<MAX_BUFFERED_PAGES;i++)
		{
			if(blocks[i].block==fp[file].current)
				break;
		}
		if(i<MAX_BUFFERED_PAGES)
		{
			if(len<=SINGLE_PAGE_SIZE-fp[file].offset)
			{
				memcpy(pbuf,blocks[i].buffer+fp[file].offset,len*sizeof(Uint16));
				fp[file].offset+=len;
				len=0;
			}else
			{
				memcpy(pbuf,blocks[i].buffer+fp[file].offset,(SINGLE_PAGE_SIZE-fp[file].offset)*sizeof(Uint16));
				pbuf+=(SINGLE_PAGE_SIZE-fp[file].offset);
				len-=(SINGLE_PAGE_SIZE-fp[file].offset);			
				fp[file].offset=0;
				fp[file].current++;
			}
		}else
		{
			Uint16 max=0;
			Uint16 tgt=0;
			for(i=0;i<MAX_BUFFERED_PAGES;i++)
			{
				if(blocks[i].block==INVALID_BLOCK_NUMBER)
					break;
				if(max<blocks[i].cntr)
				{
					max=blocks[i].cntr;
					tgt=i;
				}
			}
			i=(i<MAX_BUFFERED_PAGES?i:tgt);
			blocks[i].block=fp[file].current;
			blocks[i].cntr=0;
			addr=(Uint32)blocks[i].block*SINGLE_PAGE_SIZE;
			ReadFlashPage(addr,blocks[i].buffer,SINGLE_PAGE_SIZE);
			if(len<=SINGLE_PAGE_SIZE-fp[file].offset)
			{
				memcpy(pbuf,blocks[i].buffer+fp[file].offset,len*sizeof(Uint16));
				fp[file].offset+=len;
				len=0;
			}else
			{
				memcpy(pbuf,blocks[i].buffer+fp[file].offset,(SINGLE_PAGE_SIZE-fp[file].offset)*sizeof(Uint16));
				pbuf+=(SINGLE_PAGE_SIZE-fp[file].offset);
				len-=(SINGLE_PAGE_SIZE-fp[file].offset);			
				fp[file].offset=0;
				fp[file].current++;
			}
		}
	}
	
	for(i=0;i<MAX_BUFFERED_PAGES;i++)
	{
		if(blocks[i].block!=INVALID_BLOCK_NUMBER)
			blocks[i].cntr=(blocks[i].cntr>0xfffe?blocks[i].cntr:blocks[i].cntr+1);
	}
	return 0;
}

int16 Write(Uint16 file,Uint16 *pbuf,Uint16 len)
{
	Uint16 i=0;
	Uint32 addr=0;
	if(fp[file].startBlock==INVALID_FILE_POINTER)
		return -1;
	while(len>0)
	{
		for(i=0;i<MAX_BUFFERED_PAGES;i++)
		{
			if(blocks[i].block==fp[file].current)
				break;
		}
		if(i<MAX_BUFFERED_PAGES)
		{
			if(len<=SINGLE_PAGE_SIZE-fp[file].offset)
			{
				memcpy(blocks[i].buffer+fp[file].offset,pbuf,len*sizeof(Uint16));
				fp[file].offset+=len;
				len=0;
			}else
			{
				memcpy(blocks[i].buffer+fp[file].offset,pbuf,(SINGLE_PAGE_SIZE-fp[file].offset)*sizeof(Uint16));
				addr=(Uint32)(blocks[i].block)*SINGLE_PAGE_SIZE;
				WriteFlashPage(addr,blocks[i].buffer,SINGLE_PAGE_SIZE);
				pbuf+=(SINGLE_PAGE_SIZE-fp[file].offset);
				len-=(SINGLE_PAGE_SIZE-fp[file].offset);			
				fp[file].offset=0;
				fp[file].current++;
			}
		}else
		{
			Uint16 max=0;
			Uint16 tgt=0;
			for(i=0;i<MAX_BUFFERED_PAGES;i++)
			{
				if(blocks[i].block==INVALID_BLOCK_NUMBER)
					break;
				if(max<blocks[i].cntr)
				{
					max=blocks[i].cntr;
					tgt=i;
				}
			}
			//i=(i<MAX_BUFFERED_PAGES?i:tgt);
			if(i>=MAX_BUFFERED_PAGES)
				i=tgt;
			blocks[i].block=fp[file].current;
			blocks[i].cntr=0;
			if(len<=SINGLE_PAGE_SIZE-fp[file].offset)
			{
				memcpy(blocks[i].buffer+fp[file].offset,pbuf,len*sizeof(Uint16));
				fp[file].offset+=len;
				len=0;
			}else
			{
				memcpy(pbuf,blocks[i].buffer+fp[file].offset,(SINGLE_PAGE_SIZE-fp[file].offset)*sizeof(Uint16));
				addr=(Uint32)(blocks[i].block)*SINGLE_PAGE_SIZE;
				WriteFlashPage(addr,blocks[i].buffer,SINGLE_PAGE_SIZE);
				pbuf+=(SINGLE_PAGE_SIZE-fp[file].offset);
				len-=(SINGLE_PAGE_SIZE-fp[file].offset);			
				fp[file].offset=0;
				fp[file].current++;
			}
		}
	}
	
	for(i=0;i<MAX_BUFFERED_PAGES;i++)
	{
		if(blocks[i].block!=INVALID_BLOCK_NUMBER)
			blocks[i].cntr=(blocks[i].cntr>0xfffe?blocks[i].cntr:blocks[i].cntr+1);
	}
	return 0;
}

int16 CloseAll()
{
	Uint16 i=0;
	for(i=0;i<MAX_FILE_POINTERS;i++)
		Close(i);
	return 0;
}

int16 Close(Uint16 file)
{
	Uint16 i=0;
	Uint32 addr=0;
	if(fp[file].startBlock==INVALID_FILE_POINTER)
		return -1;
	if(fp[file].flag&FILE_WRITE_ONLY)
	{
		if(fp[file].offset!=0)
		{
			for(i=0;i<MAX_BUFFERED_PAGES;i++)
			{
				if(blocks[i].block==fp[file].current)
					break;
			}
			if(i<MAX_BUFFERED_PAGES)
			{
				addr=(Uint32)(blocks[i].block)*SINGLE_PAGE_SIZE;
				WriteFlashPage(addr,blocks[i].buffer,SINGLE_PAGE_SIZE);
			}
		}
	}
	fp[file].startBlock=INVALID_FILE_POINTER;
	fp[file].current=0;
	fp[file].offset=0;
	fp[file].flag=0;
	return 0;
}



