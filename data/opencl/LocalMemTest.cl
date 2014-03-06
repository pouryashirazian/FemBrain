//Comfort Types
typedef unsigned char		U8;
typedef unsigned short		U16;
typedef unsigned int		U32;
typedef			 char		I8;
typedef			 short		I16;
typedef			 int		I32;

#define MAX_NODES 8192

__kernel void LocalMem(__global float4* arrInHeader4,
							   __global float4* arrInOps4,										 
							   __global float4* arrInPrims4,											 
							   __global float4* arrInMtxNodes4,							   
							   __global float4* arrOutFields) 
{
 	float stkOps[MAX_NODES];
	for(int i=0; i<MAX_NODES; i++)
		stkOps[i] = 0;

	int idX = get_global_id(0);	
	stkOps[idX] = 0.5f;
	arrOutFields[idX + 1] = stkOps[idX + 1];
}