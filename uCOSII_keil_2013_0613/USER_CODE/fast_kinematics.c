#include <stdlib.h>
#include <string.h>
#include "..\config.h"
#include "transplant.h"
#include "main.h"

struct HomogenousMatrix
{
	int16 rotMatrix[3*3];
	int16 offsetVector[1*3];
};

#define MULTI2(a,b) (int16)(((int32)(a)*(b))>>PRECISION_BIT)
#define MULTI3(a,b,c) MULTI2(MULTI2((a),(b)),(c))

Uint16 FillInHomoMatrixRPY(struct HomogenousMatrix *pmatrix,struct RigidPose *prot,struct RigidOffset *poffset)
{
	int16 alpha=0;
	int16 beta=0;
	int16 theta=0;
	pmatrix->offsetVector[0]=poffset->x;
	pmatrix->offsetVector[1]=poffset->y;
	pmatrix->offsetVector[2]=poffset->z;
	alpha=prot->alpha;
	beta=prot->beta;
	theta=prot->theta;
	pmatrix->rotMatrix[0*3+0]=MULTI2(COS(beta),COS(theta));
	pmatrix->rotMatrix[0*3+1]=-MULTI2(COS(alpha),SIN(theta));
	pmatrix->rotMatrix[0*3+1]+=MULTI3(SIN(alpha),SIN(beta),COS(theta));
	pmatrix->rotMatrix[0*3+2]=MULTI2(SIN(alpha),SIN(theta));
	pmatrix->rotMatrix[0*3+2]+=MULTI3(SIN(beta),COS(theta),COS(alpha));
	pmatrix->rotMatrix[1*3+0]=MULTI2(COS(beta),SIN(theta));
	pmatrix->rotMatrix[1*3+1]=MULTI2(COS(alpha),COS(theta));
	pmatrix->rotMatrix[1*3+1]+=MULTI3(SIN(beta),SIN(alpha),SIN(theta));
	pmatrix->rotMatrix[1*3+2]=-MULTI2(COS(theta),SIN(alpha));
	pmatrix->rotMatrix[1*3+2]+=MULTI3(SIN(beta),SIN(theta),COS(alpha));
	pmatrix->rotMatrix[2*3+0]=-SIN(beta);
	pmatrix->rotMatrix[2*3+1]=MULTI2(COS(beta),SIN(alpha));
	pmatrix->rotMatrix[2*3+2]=MULTI2(COS(beta),COS(alpha));
	return 0;
}

Uint16 FillInHomoMatrixRPYRev(struct HomogenousMatrix *pmatrix,struct RigidPose *prot,struct RigidOffset *poffset)
{
	int16 alpha=0;
	int16 beta=0;
	int16 theta=0;
	pmatrix->offsetVector[0]=poffset->x;
	pmatrix->offsetVector[1]=poffset->y;
	pmatrix->offsetVector[2]=poffset->z;
	alpha=prot->alpha;
	beta=prot->beta;
	theta=prot->theta;
	pmatrix->rotMatrix[0*3+0]=MULTI2(COS(beta),COS(theta));
	pmatrix->rotMatrix[1*3+0]=-MULTI2(COS(alpha),SIN(theta));
	pmatrix->rotMatrix[1*3+0]+=MULTI3(SIN(alpha),SIN(beta),COS(theta));
	pmatrix->rotMatrix[2*3+0]=MULTI2(SIN(alpha),SIN(theta));
	pmatrix->rotMatrix[2*3+0]+=MULTI3(SIN(beta),COS(theta),COS(alpha));
	pmatrix->rotMatrix[0*3+1]=MULTI2(COS(beta),SIN(theta));
	pmatrix->rotMatrix[1*3+1]=MULTI2(COS(alpha),COS(theta));
	pmatrix->rotMatrix[1*3+1]+=MULTI3(SIN(beta),SIN(alpha),SIN(theta));
	pmatrix->rotMatrix[2*3+1]=-MULTI2(COS(theta),SIN(alpha));
	pmatrix->rotMatrix[2*3+1]+=MULTI3(SIN(beta),SIN(theta),COS(alpha));
	pmatrix->rotMatrix[0*3+2]=-SIN(beta);
	pmatrix->rotMatrix[1*3+2]=MULTI2(COS(beta),SIN(alpha));
	pmatrix->rotMatrix[2*3+2]=MULTI2(COS(beta),COS(alpha));
	return 0;
}

Uint16 FillInBaseMatrixRev(int16 *pmatrix,int16 theta1p2,int16 theta0)
{
	pmatrix[0*3+0]=COS(theta1p2);
	pmatrix[0*3+1]=MULTI2(SIN(theta1p2),SIN(theta0));
	pmatrix[0*3+2]=-MULTI2(SIN(theta1p2),COS(theta0));
	pmatrix[1*3+0]=0;
	pmatrix[1*3+1]=COS(theta0);
	pmatrix[1*3+2]=SIN(theta0);
	pmatrix[2*3+0]=SIN(theta1p2);
	pmatrix[2*3+1]=-MULTI2(COS(theta1p2),SIN(theta0));
	pmatrix[2*3+2]=MULTI2(COS(theta1p2),COS(theta0));
	return 0;
}

Uint16 MultiHomoMatrix(struct HomogenousMatrix *pleft,struct HomogenousMatrix *pright,struct HomogenousMatrix *presult)
{
	Uint16 i=0;
	Uint16 j=0;
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			presult->rotMatrix[i*3+j]=MULTI2(pleft->rotMatrix[i*3],pright->rotMatrix[j])+
										MULTI2(pleft->rotMatrix[i*3+1],pright->rotMatrix[3+j])+
										MULTI2(pleft->rotMatrix[i*3+2],pright->rotMatrix[6+j]);
		}
		presult->offsetVector[i]=MULTI2(pleft->rotMatrix[i*3],pright->offsetVector[0])+
										MULTI2(pleft->rotMatrix[i*3+1],pright->offsetVector[1])+
										MULTI2(pleft->rotMatrix[i*3+2],pright->offsetVector[2])+
										pleft->offsetVector[i];
	}
	return 0;
}

Uint16 CalRelativeVector(struct HomogenousMatrix *pleft,int16 *pvector)
{
	Uint16 i=0;
	int16 temp[3];
	for(i=0;i<3;i++)
	{
		temp[i]=MULTI2(pleft->rotMatrix[i*3],pvector[0])+
				MULTI2(pleft->rotMatrix[i*3+1],pvector[1])+
				MULTI2(pleft->rotMatrix[i*3+2],pvector[2])+
				pleft->offsetVector[i];
	}
	memcpy(pvector,temp,sizeof(temp));	
	return 0;
}

Uint16 GetInverseKine(struct RigidBody *pankle, struct RigidBody *phip, struct LegParameter *pleg,int16 *result)
{
	struct HomogenousMatrix matrix1,matrix2,matrix3;
	int16 vector[3];
	int32 temp=0;
	Uint32 interval;
	interval=OSTimeGet();
	
	//swinging leg calculation
	pankle->offset.x=-pankle->offset.x+phip->offset.x;
	pankle->offset.y=-pankle->offset.y+phip->offset.y;
	pankle->offset.z=-pankle->offset.z+phip->offset.z;
	FillInHomoMatrixRPY(&matrix2,&(phip->pose),&(pankle->offset));
	
	pankle->offset.x=0;
	pankle->offset.y=0;
	pankle->offset.z=-pleg->foot;
	FillInHomoMatrixRPYRev(&matrix1,&(pankle->pose),&(pankle->offset));
	
	MultiHomoMatrix(&matrix1,&matrix2,&matrix3);
	vector[0]=0;
	vector[1]=(pleg->legOffset>>1);
	vector[2]=-(pleg->hipHight);
	CalRelativeVector(&matrix3,vector);
	
	temp=(int32)vector[0]*vector[0]+(int32)vector[1]*vector[1]+(int32)vector[2]*vector[2];
	temp=(int32)pleg->crus*pleg->crus+(int32)pleg->thigh*pleg->thigh-temp;
	temp<<=(PRECISION_BIT-1);
	temp=temp/((int32)pleg->thigh*pleg->crus);
	temp=PI-ARCCOS(temp);
	result[9]=(int16)temp;
	
	//temp=ARCTAN2(vector[2],vector[1])-PI/2;
	temp=-ARCTAN2(vector[1],vector[2]);
	result[11]=(int16)temp;
	
	temp=(int32)(pleg->crus)*pleg->crus*SIN(result[9]);
	temp=temp/((int32)vector[0]*vector[0]+(int32)vector[1]*vector[1]+(int32)vector[2]*vector[2]);
	temp=(1<<PRECISION_BIT)-((temp*SIN(result[9]))>>(PRECISION_BIT-1));
	result[10]=(ARCCOS(temp)>>1);
	
	temp=(int32)vector[0]*COS(result[11]);
	temp>>=PRECISION_BIT;
	result[10]+=ARCTAN2(temp,vector[2]);
	result[10]=-result[10];
	
	FillInBaseMatrixRev(matrix1.rotMatrix,-(result[9]+result[10]),result[11]);
	matrix1.offsetVector[0]=0;
	matrix1.offsetVector[1]=0;
	matrix1.offsetVector[2]=0;
	MultiHomoMatrix(&matrix1,&matrix3,&matrix2);
	result[8]=-ARCSIN(matrix2.rotMatrix[2*3+0]);
	result[7]=ARCTAN2(matrix2.rotMatrix[2*3+1],matrix2.rotMatrix[2*3+2]);
	result[6]=ARCTAN2(matrix2.rotMatrix[1*3+0],matrix2.rotMatrix[0*3+0]);
	
	//supporting leg calculation
	pankle->offset.x=phip->offset.x;
	pankle->offset.y=phip->offset.y;
	pankle->offset.z=phip->offset.z-pleg->foot;
	FillInHomoMatrixRPY(&matrix3,&(phip->pose),&(pankle->offset));
	vector[0]=0;
	vector[1]=-(pleg->legOffset>>1);
	vector[2]=-(pleg->hipHight);
	CalRelativeVector(&matrix3,vector);
	
	temp=(int32)vector[0]*vector[0]+(int32)vector[1]*vector[1]+(int32)vector[2]*vector[2];
	temp=(int32)pleg->crus*pleg->crus+(int32)pleg->thigh*pleg->thigh-temp;
	temp<<=(PRECISION_BIT-1);
	temp=temp/((int32)pleg->thigh*pleg->crus);
	temp=PI-ARCCOS(temp);
	result[2]=(int16)temp;
	
	//temp=ARCTAN2(vector[2],vector[1])-PI/2;
	temp=-ARCTAN2(vector[1],vector[2]);
	result[0]=(int16)temp;
	
	temp=(int32)(pleg->crus)*pleg->crus*SIN(result[2]);
	temp=temp/((int32)vector[0]*vector[0]+(int32)vector[1]*vector[1]+(int32)vector[2]*vector[2]);
	temp=(1<<PRECISION_BIT)-((temp*SIN(result[2]))>>(PRECISION_BIT-1));
	result[1]=(ARCCOS(temp)>>1);
	
	temp=(int32)vector[0]*COS(result[0]);
	temp>>=PRECISION_BIT;
	result[1]+=ARCTAN2(temp,vector[2]);
	result[1]=-result[1];
	
	FillInBaseMatrixRev(matrix1.rotMatrix,-(result[1]+result[2]),result[0]);
	matrix1.offsetVector[0]=0;
	matrix1.offsetVector[1]=0;
	matrix1.offsetVector[2]=0;
	MultiHomoMatrix(&matrix1,&matrix3,&matrix2);
	result[3]=-ARCSIN(matrix2.rotMatrix[2*3+0]);
	result[4]=ARCTAN2(matrix2.rotMatrix[2*3+1],matrix2.rotMatrix[2*3+2]);
	result[5]=ARCTAN2(matrix2.rotMatrix[1*3+0],matrix2.rotMatrix[0*3+0]);
	
	//for reverse foot coordinates
	if(pleg->legOffset<0)
	{
		int16 angle=0;
		Uint16 i=0;
		for(i=0;i<6;i++)
		{
			angle=result[i];
			result[i]=result[11-i];
			result[11-i]=angle;
		}
	}
	
	result[1]=-result[1];
	result[2]=-result[2];
	//result[3]=-result[3];
	result[10]=-result[10];
	result[9]=-result[9];
	//result[8]=-result[8];
	interval=OSTimeGet()-interval;
	return 0;
}






