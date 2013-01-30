//Directions
const int LBN =	0;  /* left bottom near corner  */
const int LBF =	1;  /* left bottom far corner   */
const int LTN =	2;  /* left top near corner     */
const int LTF =	3;  /* left top far corner      */
const int RBN =	4;  /* right bottom near corner */
const int RBF =	5;  /* right bottom far corner  */
const int RTN =	6;  /* right top near corner    */
const int RTF =	7;  /* right top far corner     */

//Axis
const int AAX = 0;
const int AAY = 1;
const int AAZ = 2;

//edge: LB, LT, LN, LF, RB, RT, RN, RF, BN, BF, TN, TF
//Corner and EdgeAxis
//const int corner1[12]    = {LBN,LTN,LBN,LBF,RBN,RTN,RBN,RBF,LBN,LBF,LTN,LTF};
//const int corner2[12]    = {LBF,LTF,LTN,LTF,RBF,RTF,RTN,RTF,RBN,RBF,RTN,RTF};
//const int edgeaxis[12]   = {AAZ,AAZ,AAY,AAY,AAZ,AAZ,AAY,AAY,AAX,AAX,AAX,AAX};


/*!
 *  Compute the cell configuration by evaluating the field at 8 vertices of the cube.
 */
__kernel void ComputeConfig(__global float4* arrInHeader4,
			 __global float4* arrInOps4,										 
			 __global float4* arrInPrims4,											 
	 		 __global float4* arrInMtxNodes4,
			 __read_only image2d_t texInVertexCountTable,
			 __constant struct CellParam* inCellParams,
			 __global uchar* arrOutCellConfig,
			 __global uchar* arrOutVertexCount)		
{
	//Get XY plane index
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inCellParams->ctNeededCells[0])||(idY >= inCellParams->ctNeededCells[1])||(idZ >= inCellParams->ctNeededCells[2]))
		return;
	uint idxCell = idZ * (inCellParams->ctNeededCells[0] * inCellParams->ctNeededCells[1]) + idY * inCellParams->ctNeededCells[0] + idX;
	if(idxCell > inCellParams->ctTotalCells)
		return;
	
	float cellsize = inCellParams->cellsize;
	//printf("Cellsize: %.2f\n", cellsize);

	//Count : Prims, Ops, Mtx ; CellSize
	U32 ctPrims = (U32)arrInHeader4[OFFSET4_HEADER_PARAMS].x;
	U32 ctOps   = (U32)arrInHeader4[OFFSET4_HEADER_PARAMS].y;
	float4 lower = arrInHeader4[OFFSET4_HEADER_LOWER] + cellsize * (float4)(idX, idY, idZ, 0.0f);

	float arrFields[8];
	float4 arrVertices[8];
	arrVertices[0] = lower;
	arrVertices[1] = lower + cellsize * (float4)(0, 0, 1, 0);
	arrVertices[2] = lower + cellsize * (float4)(0, 1, 0, 0);
  	arrVertices[3] = lower + cellsize * (float4)(0, 1, 1, 0);
	arrVertices[4] = lower + cellsize * (float4)(1, 0, 0, 0);
	arrVertices[5] = lower + cellsize * (float4)(1, 0, 1, 0);
  	arrVertices[6] = lower + cellsize * (float4)(1, 1, 0, 0);
	arrVertices[7] = lower + cellsize * (float4)(1, 1, 1, 0);
	
    //Compute Configuration index
    int idxConfig = 0;
	for(int i=0; i<8; i++)
	{
		//arrFields[i] = ComputePrimitiveField(0, arrVertices[i], arrInOps4, arrInPrims4, arrInMtxNode4);
		arrFields[i] = ComputeField(arrVertices[i], arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		if(isgreaterequal(arrFields[i], ISO_VALUE))
			idxConfig += (1 << i);
	}
	
    //read number of vertices that output from this cell
    U8 ctVertices = read_imageui(texInVertexCountTable, tableSampler, (int2)(idxConfig,0)).x;

	//Cell-based output											
	arrOutCellConfig[idxCell] = idxConfig;
	arrOutVertexCount[idxCell] = ctVertices;
}


/*
 * Extract mesh by computing its vertices
 */
__kernel void ComputeMesh(__global float4* arrInHeader4,
						  __global float4* arrInOps4,										 
						  __global float4* arrInPrims4,											 
						  __global float4* arrInMtxNode4,					      
						  __read_only image2d_t texInTriangleTable,
						  __constant struct CellParam* inCellParams,
						  __global U8* arrInCellConfig,						
						  __global U32* arrInVertexBufferOffset,
						  
						  __global U32* arrInTetMeshBufferOffset,
						  __global float* arrOutTetMeshVertices,
						  __global U32* arrOutTetMeshIndices,
						  
						  __global float4* arrOutMeshVertex,
						  __global float4* arrOutMeshColor,
						  __global float* arrOutMeshNormal)						  
{
	//Get XY plane index
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inCellParams->ctNeededCells[0])||(idY >= inCellParams->ctNeededCells[1])||(idZ >= inCellParams->ctNeededCells[2]))
		return;
	uint idxCell = idZ * (inCellParams->ctNeededCells[0] * inCellParams->ctNeededCells[1]) + idY * inCellParams->ctNeededCells[0] + idX;
	if(idxCell > inCellParams->ctTotalCells)
		return;
	
	//We need complete insiders too
	if(arrInCellConfig[idxCell] == 0)
		return;

	float cellsize = inCellParams->cellsize;
	float4 lower = arrInHeader4[OFFSET4_HEADER_LOWER] + cellsize * (float4)(idX, idY, idZ, 0.0f);
	float arrFields[8];
	float4 arrVertices[8];
	arrVertices[0] = lower;
	arrVertices[1] = lower + cellsize * (float4)(0, 0, 1, 0);
	arrVertices[2] = lower + cellsize * (float4)(0, 1, 0, 0);
  	arrVertices[3] = lower + cellsize * (float4)(0, 1, 1, 0);
	arrVertices[4] = lower + cellsize * (float4)(1, 0, 0, 0);
	arrVertices[5] = lower + cellsize * (float4)(1, 0, 1, 0);
  	arrVertices[6] = lower + cellsize * (float4)(1, 1, 0, 0);
	arrVertices[7] = lower + cellsize * (float4)(1, 1, 1, 0);
	

    //Compute Configuration index
	for(int i=0; i<8; i++)
	{
		arrFields[i] = ComputeField(arrVertices[i], arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNode4);
	}

	//Tetrahedralize
	//(x,y,z) * number of cubes * 8 cube corners
	//cl_mem inoutMemTetMeshVertices = m_lpGPU->createMemBuffer(sizeof(float) * 3 * ctCrossedOrInside * 8, ComputeDevice::memReadWrite);

	//(a,b,c,d) * number of cubes * 6 Tetrahedra per cube
	//cl_mem inoutMemTetMeshIndices = m_lpGPU->createMemBuffer(sizeof(U32) * 4 * ctCrossedOrInside * 6, ComputeDevice::memReadWrite);
	U32 idxTet = arrInTetMeshBufferOffset[idxCell];	
	U32 offsetVertex = idxTet * 8 * 3;
	for(int i=0; i<8; i++)
	{
		arrOutTetMeshVertices[offsetVertex + i*3 + 0] = arrVertices[i].x; 
		arrOutTetMeshVertices[offsetVertex + i*3 + 1] = arrVertices[i].y;
		arrOutTetMeshVertices[offsetVertex + i*3 + 2] = arrVertices[i].z;
	}
	
	//6 Tets per each cube
	U32 offsetTetVertex = idxTet*8;
	U32 offsetTet = idxTet * 6 * 4;
	arrOutTetMeshIndices[offsetTet + 0] = offsetTetVertex + LBN;
	arrOutTetMeshIndices[offsetTet + 1] = offsetTetVertex + LTN;
	arrOutTetMeshIndices[offsetTet + 2] = offsetTetVertex + RBN;
	arrOutTetMeshIndices[offsetTet + 3] = offsetTetVertex + LBF;
	
	arrOutTetMeshIndices[offsetTet + 4] = offsetTetVertex + RTN;
	arrOutTetMeshIndices[offsetTet + 5] = offsetTetVertex + LTN;
	arrOutTetMeshIndices[offsetTet + 6] = offsetTetVertex + LBF;
	arrOutTetMeshIndices[offsetTet + 7] = offsetTetVertex + RBN;

	arrOutTetMeshIndices[offsetTet + 8] = offsetTetVertex + RTN;
	arrOutTetMeshIndices[offsetTet + 9] = offsetTetVertex + LTN;
	arrOutTetMeshIndices[offsetTet + 10] = offsetTetVertex + LTF;
	arrOutTetMeshIndices[offsetTet + 11] = offsetTetVertex + LBF;

	arrOutTetMeshIndices[offsetTet + 12] = offsetTetVertex + RTN;
	arrOutTetMeshIndices[offsetTet + 13] = offsetTetVertex + RBN;
	arrOutTetMeshIndices[offsetTet + 14] = offsetTetVertex + LBF;
	arrOutTetMeshIndices[offsetTet + 15] = offsetTetVertex + RBF;

	arrOutTetMeshIndices[offsetTet + 16] = offsetTetVertex + RTN;
	arrOutTetMeshIndices[offsetTet + 17] = offsetTetVertex + LBF;
	arrOutTetMeshIndices[offsetTet + 18] = offsetTetVertex + LTF;
	arrOutTetMeshIndices[offsetTet + 19] = offsetTetVertex + RBF;

	arrOutTetMeshIndices[offsetTet + 20] = offsetTetVertex + RTN;
	arrOutTetMeshIndices[offsetTet + 21] = offsetTetVertex + LTF;
	arrOutTetMeshIndices[offsetTet + 22] = offsetTetVertex + RTF;
	arrOutTetMeshIndices[offsetTet + 23] = offsetTetVertex + RBF;

	

	//Variables
	U32 idxEdge;
	float4 e1;
	float4 e2;
	float4 v;
	float3 n;
	float scale;
	U32 idxMeshAttrib;
	int idxEdgeStart, idxEdgeEnd, idxEdgeAxis;
	int idxConfig = arrInCellConfig[idxCell];
	int voffset = arrInVertexBufferOffset[idxCell];

	//Break loop if the idxEdge is 255
	for(int i=0; i<16; i++)
	{
		idxEdge = read_imageui(texInTriangleTable, tableSampler, (int2)(i, idxConfig)).x;
		if(idxEdge == 255)
			break;
		idxEdgeStart = inCellParams->corner1[idxEdge];
		idxEdgeEnd   = inCellParams->corner2[idxEdge];
		idxEdgeAxis  = inCellParams->edgeaxis[idxEdge];	

		e1 = lower + cellsize * (float4)((int)((idxEdgeStart & 0x04) >> 2), (int)((idxEdgeStart & 0x02) >> 1), (int)(idxEdgeStart & 0x01), 0.0f);
		e2 = e1;
		if(idxEdgeAxis == 0)
			e2.x += cellsize;
		else if(idxEdgeAxis == 1)
			e2.y += cellsize;
		else
			e2.z += cellsize;
		scale = (ISO_VALUE - arrFields[idxEdgeStart])/(arrFields[idxEdgeEnd] - arrFields[idxEdgeStart]);
		
		//Use Linear Interpolation for now. Upgrade to Newton-Raphson (Gradient Marching)
		v = e1 + scale * (e2 - e1);

		//Compute Normal
		n = ComputeNormal(v, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNode4);

		//Compute Field and Color
		float4 color;
		ComputeFieldAndColor(v,	&color, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNode4);

		//MeshAttrib index
		idxMeshAttrib = voffset + i;
		 
		arrOutMeshVertex[idxMeshAttrib] = v;
		arrOutMeshColor[idxMeshAttrib] = color;
		arrOutMeshNormal[idxMeshAttrib * 3] = n.x;
		arrOutMeshNormal[idxMeshAttrib * 3 + 1] = n.y;
		arrOutMeshNormal[idxMeshAttrib * 3 + 2] = n.z;
	//	printf("n = [%.2f, %.2f, %.2f] \n", n.x, n.y, n.z);  
	}
}

