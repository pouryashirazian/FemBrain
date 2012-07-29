//Stack
typedef struct{
	U16 arrIndices[MAX_TREE_DEPTH];
	int idxTop;
}STACKU16;

typedef struct{
	U16 arrIndices[MAX_TREE_DEPTH];
	float arrFields[MAX_TREE_DEPTH];
	int idxTop;
}STACKPAIR;

void stkPush(U16 idxOp, STACKU16* stkOps)
{
	stkOps->idxTop++;
	stkOps->arrIndices[stkOps->idxTop] = idxOp;
}

void stkPop(STACKU16* stkOps)
{	
	stkOps->idxTop--;
}

void stkPushPair(U16 idxOp, float field, STACKPAIR* stkOpsField)
{
	stkOpsField->idxTop++;
	stkOpsField->arrIndices[stkOpsField->idxTop] = idxOp;
	stkOpsField->arrFields[stkOpsField->idxTop] = field;
}

void stkPopPair(STACKPAIR* stkOpsField)
{	
	stkOpsField->idxTop--;
}


//Kernel Function to compute dist2ances to sphere at origin 
__kernel void poly(__global float4* arrMeshVertex,
				   __global float4* arrMeshColor,
				   __global float* arrPosX, 
				   __global float* arrPosY, 
				   __global float* arrPosZ,				   
				   const unsigned int count)
{	
	int idx = get_global_id(0);
	if(idx < count)
	{	
		float4 v = (float4)(arrPosX[idx], arrPosY[idx], arrPosZ[idx], 1.0);		
		arrMeshVertex[idx] = v;
		float4 c = 255.0f * normalize(v);
		arrMeshColor[idx] = (float4)(c.x, c.y, c.z, 1.0f); 				
	}	
};

// TODO: Add OpenCL kernel code here.
/*!
* The input BlobTree is arranged as the following: HEADER + PRIMS + OPSS
* Header is 12 floats: 4F Lower BBOX + 4F Upper BBOX + (ctPrims + ctOps + ctMtxNodes + cellsize)
* Prims is 20 floats per each: type + idxMatrix + 4F Pos + 4F Dir + 4F Res + 4F Color
* Ops is 8 floats per each: type + opFlags + opLeftRightChildrenIds + 4F res
* Main Kernel to compute cell configurations.
* @param arrInHeader the header of the BlobTree input model
* @param arrInPrims the primitives input BlobTree
* @param arrInMtxNode the matrices of the input BlobTree
*/
float ComputeOperatorField(int idxOperator,
						  float4 v,						
						  __global float* arrInHeader,
						  __global float* arrInOps,
						  __global float* arrInPrims,						  
						  __global float* arrInMtxNode)
{
	//Access vector4
	__global float4* header4  = (__global float4 *)arrInHeader;
	__global float4* ops4     = (__global float4 *)arrInOps;
	__global float4* prims4   = (__global float4 *)arrInPrims;	
	__global float4* matrix4  = (__global float4 *)arrInMtxNode;

	//Compute Point		
	float4 resColor = (float4)(0.0f, 1.0f, 0.0f, 1.0f);
	float resField  = 0.0f;
	
	//Stack for operators and intermediate field values
	STACKU16 stkOps;
	STACKPAIR stkOpsField;
	stkOps.idxTop = -1;
	stkOpsField.idxTop = -1;
	
	//Push first operator onto stack
	if((int)arrInHeader[OFFSET_HEADER_COUNT_OPS] > 0)
		stkPush(0, &stkOps); 
	else
	{
		int ctPrims = (int)arrInHeader[OFFSET_HEADER_COUNT_PRIMS];		 
		resColor = (float4)(0.0f, 0.0f, 0.0f, 0.0f);		
		for(int i=0;i<ctPrims;i++)
		{
			float curField = ComputePrimitiveField(i, v, ops4, prims4, matrix4);
			resColor += curField * prims4[i * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
			resField += curField; 
		}
	}


	//While Stack is non-empty
	while(stkOps.idxTop != -1)
	{
		//Get top of stack op Index
		int idxOp = stkOps.arrIndices[stkOps.idxTop];
				
		//2. Fetch op flags
		//Check children types
		//bool isUnary = (((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].y) & 0x08) != 0);
		//bool isRange = (((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].y) & 0x04) != 0);		
		bool isLCOp = (((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].y) & 0x02) != 0);
		bool isRCOp = (((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].y) & 0x01) != 0);

		//3. Fetch left, right children indices
		U16 idxLC = ((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].z) >> 16) & 0xFFFF;	
		U16 idxRC = ((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].z)) & 0xFFFF;	
		U8 ctAvailable = 0;
		float lcf, rcf;
		
		//Left Child
		if(isLCOp)
		{
			//If Field Stack is Empty
			if(stkOpsField.idxTop == -1)
				stkPush(idxLC, &stkOps);
			//Else if the top of field stack belongs to this child
			else if(stkOpsField.arrIndices[stkOpsField.idxTop] == idxLC)
			{
				lcf = stkOpsField.arrFields[stkOpsField.idxTop];
				stkPopPair(&stkOpsField);
				ctAvailable++;
			}
			//Else push onto stack
			else
				stkPush(idxLC, &stkOps);
		}
		else
			ctAvailable++;


		//Right Child
		if(isRCOp)
		{
			//If Field Stack is Empty
			if(stkOpsField.idxTop == -1)
				stkPush(idxRC, &stkOps);
			//Else if the top of field stack belongs to this child
			else if(stkOpsField.arrIndices[stkOpsField.idxTop] == idxRC)
			{
				rcf = stkOpsField.arrFields[stkOpsField.idxTop];
				stkPopPair(&stkOpsField);
				ctAvailable++;
			}
			//Else push onto stack
			else
				stkPush(idxRC, &stkOps);
		}
		else
			ctAvailable++;

		//Evaluate the field for this operator and store it in the op field stack
		if(ctAvailable >= 2)
		{
			//Compute Primitive Fields
			if(isLCOp == 0)
				lcf = ComputePrimitiveField(idxLC, v, ops4, prims4, matrix4);
			if(isRCOp == 0)
				rcf = ComputePrimitiveField(idxRC, v, ops4, prims4, matrix4);

			//1. Fetch op type
			U16 opType = (int)(ops4[idxOp * DATASIZE_OPERATOR_F4].x);			
			switch(opType)
			{
			case(opBlend):
				resField = lcf + rcf;
					//	Float_ lcf  = two * (half + leftChildField) - one;
					//	Float_ rcf  = two * (half + rightChildField) - one;
				break;
			}

			//Push the pair on to stack
			stkPushPair(idxOp, resField, &stkOpsField);
			stkPop(&stkOps);
		}
	}

	return resField;
}