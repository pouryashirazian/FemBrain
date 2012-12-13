/* Please Write the OpenCL Kernel(s) code here or load it from a text file *///OpenCL BlobTree Polygonizer: Pourya Shirazian
#define DATASIZE_HEADER		12
#define DATASIZE_OPERATOR	8
#define DATASIZE_PRIMITIVE	20

#define DATASIZE_OPERATOR_F4	DATASIZE_OPERATOR/4
#define DATASIZE_PRIMITIVE_F4	DATASIZE_PRIMITIVE/4

//OFFSETS in PRIMITIVES
#define OFFSET4_PRIM_TYPE		0
#define OFFSET4_PRIM_POS		1
#define OFFSET4_PRIM_DIR		2
#define OFFSET4_PRIM_RES		3
#define OFFSET4_PRIM_COLOR		4

//OFFSETS in OPERATORS
#define OFFSET4_OP_TYPE			0
#define OFFSET4_OP_RES			1

#define OFFSET_OP_TYPE			0
#define OFFSET_OP_FLAGS			1
#define OFFSET_OP_LCRC			2
#define OFFSET_OP_PARENTLINK	3


//OFFSETS in HEADER
#define OFFSET4_HEADER_LOWER	0
#define OFFSET4_HEADER_UPPER	1
#define OFFSET4_HEADER_PARAMS	2

#define OFFSET_HEADER_BBOX_LO_X 	0
#define OFFSET_HEADER_BBOX_LO_Y 	1
#define OFFSET_HEADER_BBOX_LO_Z 	2
#define OFFSET_HEADER_BBOX_HI_X 	4
#define OFFSET_HEADER_BBOX_HI_Y 	5
#define OFFSET_HEADER_BBOX_HI_Z 	6
#define OFFSET_HEADER_COUNT_PRIMS 	8
#define OFFSET_HEADER_COUNT_OPS 	9
#define OFFSET_HEADER_COUNT_MTX		10
#define OFFSET_HEADER_CELLSIZE 		11

//BlobTree
#define MAX_TREE_DEPTH	 64
#define MAX_TREE_NODES   1024
#define MAX_MTX_NODES MAX_TREE_NODES * 2
#define PRIM_MATRIX_STRIDE 12
#define BOX_MATRIX_STRIDE 16

#define ISO_VALUE 0.5f
#define NORMAL_DELTA 0.0001f

#define ZERO4 {0.0f, 0.0f, 0.0f, 0.0f}
#define AXISX {1.0f, 0.0f, 0.0f, 0.0f}
#define AXISY {0.0f, 1.0f, 0.0f, 0.0f}
#define AXISZ {0.0f, 0.0f, 1.0f, 0.0f}

#define MAX_VERTICES_COUNT_PER_CELL		15
#define MAX_TRIANGLES_COUNT_PER_CELL	5


//Comfort Types
typedef unsigned char		U8;
typedef unsigned short		U16;
typedef unsigned int		U32;
typedef			 char		I8;
typedef			 short		I16;
typedef			 int		I32;


//Types
enum PrimitiveType {primPoint, primLine, primCylinder, primDisc, primRing, primCube, 
					primTriangle, primQuadricPoint, primNULL, primInstance};
					
enum OperatorType {opUnion, opIntersect, opDif, opSmoothDif, opBlend, opRicciBlend, 
				   opGradientBlend, opFastQuadricPointSet, opCache, opWarpTwist, 
				   opWarpTaper, opWarpBend, opWarpShear};
				   
enum OPFLAGS {ofRightChildIsOp = 1, ofLeftChildIsOp = 2, ofChildIndexIsRange = 4, ofIsUnaryOp = 8};

//Sampler
const sampler_t tableSampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;


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

typedef struct CellParam{
	U8 corner1[12];
	U8 corner2[12];
	U8 edgeaxis[12];
	U32 ctNeededCells[3];
	U32 ctTotalCells;
}CellParam;

//Computes Wyvill Field-Function
float ComputeWyvillField(float dd)
{
	if(isgreater(dd, 1.0f))
		return 0.0f;
	float t = (1.0f - dd);
	return t*t*t;
}

/*!
 * Compute field due to a primitive.
 */
float ComputePrimitiveField(int idxPrimitive,
							float4 v,			
							__global float4* arrOps4,
							__global float4* arrPrims4, 
							__global float4* arrMtxNodes4)
{
	//Transform
	int primType  = (int)(arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4].x);		
	float4 vt = v;
	{		
		int idxMatrix = (int)(arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4].y) & 0xFFFF;		
		float4 row0 = arrMtxNodes4[idxMatrix * 3];
		float4 row1 = arrMtxNodes4[idxMatrix * 3 + 1];
		float4 row2 = arrMtxNodes4[idxMatrix * 3 + 2];
		//printf("MTX row0: %.2f\n", row0.x);
		vt = (float4)(dot(row0, v), dot(row1, v), dot(row2, v), 0.0f);
	}
		

	float dist2;

	//Compute distance to the skeletal primitive
	switch(primType)
	{
	case(primPoint):
	{
		float4 d2 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS] - vt;
		d2.w = 0;
		dist2 = dot(d2, d2);		
	}	
	break;
	case(primLine):
	{
		float4 line0 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float4 lineDelta = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR] - line0;
		float lineDeltaDot = dot(lineDelta, lineDelta);
		float delta = dot(vt - line0, lineDelta) / lineDeltaDot;
		float4 tt = vt - (line0 + delta * lineDelta);
		dist2 = dot(tt, tt);
	}
	break;
	case(primCylinder):
	{
		float4 pos = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float4 dir = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR];
		float radius = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		float height = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].y;
		float y = dot(pos, dir);
		float x = fmax(0.0f, sqrt(dot(pos, pos) - y*y) - radius);
		if(isgreater(y, 0.0f))
			y = fmax(0.0f, y - height);
		dist2 = x*x + y*y;			
	}	
	break;	
	case(primTriangle):
	{
		dist2  = 10.0f;
	}
	break;	

	case(primCube):
	{
		float4 dif = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float side = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
				
		float projected;
		float delta;
			
		//Along X
		float4 axis = (float4)(1.0f, 0.0f, 0.0f, 0.0f);		
		projected = dot(dif, axis);
		if(isless(projected, -1.0f * side))
		{
		    delta = projected + side;
		    dist2 += delta*delta;
		}
		else if (isgreater(projected, side))
		{
		    delta = projected - side;
		    dist2 += delta*delta;
		}

		//Along Y
		axis = (float4)(0.0f, 1.0f, 0.0f, 0.0f);
		projected = dot(dif, axis);
		if(isless(projected, -1.0f * side))
		{
		    delta = projected + side;
		    dist2 += delta*delta;
		}
		else if (isgreater(projected, side))
		{
		    delta = projected - side;
		    dist2 += delta*delta;
		}

		//Along Z
		axis = (float4)(0.0f, 0.0f, 1.0f, 0.0f);
		projected = dot(dif, axis);
		if(isless(projected, -1.0f * side))
		{
		    delta = projected + side;
		    dist2 += delta*delta;
		}
		else if (isgreater(projected, side))
		{
		    delta = projected - side;
		    dist2 += delta*delta;
		}		
	}
	break;
	
	case(primDisc):
	{
		float4 delta = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float4 n = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR];
		float r = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		float4 dir = delta - dot(delta, n) * n;
		float len2 = dot(dir, dir);
		if(islessequal(len2, r*r))
			dist2 = fabs(dot(delta, delta) - len2);
		else
		{	
			dir = normalize(dir);
			float4 x = r*dir - delta;
			dist2 = dot(x, x);
		}			
	}
	break;
	case(primRing):
	{
		float4 delta = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float4 n   = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR];
		float r    = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		float4 dir = delta - dot(delta, n) * n;

		if(isequal(length(dir), 0.0f))
			dist2 = r*r + dot(delta, delta);
		else
		{	
			dir = normalize(dir);				
			float4 x = r*dir - delta;
			dist2 = dot(x, x);
		}			
	}
	break;
	
	case(primQuadricPoint):
	{
		float4 dt = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float rs  = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR].z;
		float cf1 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		float cf2 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].y;
		float cf3 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].z;
		dist2 = dot(dt, dt);
		if(isgreater(rs, dist2))
			return dist2*dist2*cf1 + dist2*cf2 + cf3;
	}
	break;
	case(primNULL):
	{
		dist2 = 10.0f;
	}
	break;
	
	case(primInstance):
	{
		int idxOrigin = (int)arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		int isOriginOp = (int)arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].z;
		//return ComputeOperatorField(idxOrigin, vt, arrOps4, arrPrims4, arrMtxNodes4);		
	}
	break;
		
	}

 	return ComputeWyvillField(dist2); 
}


float3 ComputeNormal(int idxPrimitive, float4 v, 
		   __global float4* arrOps4,
		   __global float4* arrPrims4,
		   __global float4* arrMtxNodes4)
{
	const float deltaInv = -1.0f / NORMAL_DELTA;	
	float inFieldValue = ComputePrimitiveField(0, v, arrOps4, arrPrims4, arrMtxNodes4);
	//printf("FieldValue= %.2f \n", inFieldValue); 
	
	
	float3 n;
	n.x = ComputePrimitiveField(0, v + (float4)(NORMAL_DELTA,0,0,0), arrOps4, arrPrims4, arrMtxNodes4);
	n.y = ComputePrimitiveField(0, v + (float4)(0, NORMAL_DELTA, 0, 0), arrOps4, arrPrims4, arrMtxNodes4);
	n.z = ComputePrimitiveField(0, v + (float4)(0, 0, NORMAL_DELTA, 0), arrOps4, arrPrims4, arrMtxNodes4);
	//n.w = 0.0f;

	n = deltaInv * (n - (float3)(inFieldValue, inFieldValue, inFieldValue));
	n = normalize(n);
	//printf("Prenormalized N= [%.2f, %.2f, %.2f, %.2f] \n", n.x, n.y, n.z, n.w); 
	return n;
}

float4 ComputePrimitiveColor(int idxPrimitive, __global float4* arrPrims4)
{
	return arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
}




/*!
* Compute Cell Configuration index and Number Vertices Output for all cells.
* @param ctTotalCells total number of cells
* @param arrOutCellConfig array containing the configuration of cubic cells
* @param arrOutVertexCount array containing the number of vertices output 
* per each cell
* @param color of each vertex
*/
__kernel void ComputeConfigIndexVertexCount(__global float4* arrInHeader4,
											__global float4* arrInOps4,										 
											__global float4* arrInPrims4,											 
					 					    __global float4* arrInMtxNode4,
											__constant struct CellParam* inCellParams,
											__read_only image2d_t texInVertexCountTable,
											__read_only image2d_t texInTriangleTable,
											__global uchar* arrOutCellConfig,
											__global uchar* arrOutVertexCount,	
											__global float4* arrOutMeshVertex,
											__global float4* arrOutMeshNormal,
											__global float4* arrOutMeshColor)
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
	
	//float cellsize = arrInHeader4[OFFSET_HEADER_CELLSIZE];	
	float cellsize = arrInHeader4[OFFSET4_HEADER_PARAMS].w;
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
		//arrFields[i] = ComputePrimitiveField(0, arrVertices[i], ops4, prims4, matrix4);
		arrFields[i] = ComputePrimitiveField(0, arrVertices[i], arrInOps4, arrInPrims4, arrInMtxNode4);
		if(isgreaterequal(arrFields[i], ISO_VALUE))
			idxConfig += (1 << i);
	}
	
	//printf("idxConfig: %d\n", idxConfig);
	
    // read number of vertices from texture
    U8 ctVertices = read_imageui(texInVertexCountTable, tableSampler, (int2)(idxConfig,0)).x;

	//Cell-based output											
	arrOutCellConfig[idxCell] = idxConfig;
	arrOutVertexCount[idxCell] = ctVertices;
	

	//Variables
	U8 idxEdge;
	float4 e1;
	float4 e2;
	float4 v;
	float scale;
	U32 idxMeshAttrib;
	int idxEdgeStart, idxEdgeEnd, idxEdgeAxis;
	
	//Read configurations from texture memory
	for(int i=0; i<ctVertices; i++)
	{
		idxEdge = read_imageui(texInTriangleTable, tableSampler, (int2)(idxConfig, i)).x;
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
		//e2[idxEdgeAxis] = e2[idxEdgeAxis] + cellsize;
		scale = (ISO_VALUE - arrFields[idxEdgeStart])/(arrFields[idxEdgeEnd] - arrFields[idxEdgeStart]);
		v = e1 + scale * (e2 - e1);


		idxMeshAttrib = idxCell * MAX_VERTICES_COUNT_PER_CELL + i; 
		arrOutMeshVertex[idxMeshAttrib] = v;
		//arrOutMeshNormal[idxMeshAttrib] = ComputeNormal(0, v, arrInOps4, arrInPrims4, arrInMtxNode4);
		arrOutMeshColor[idxMeshAttrib] = ComputePrimitiveColor(0, arrInPrims4);
	}
}


/*!
 *  Compute the cell configuration by evaluating the field at 8 vertices of the cube.
 */
__kernel void ComputeConfig(__global float4* arrInHeader4,
							__global float4* arrInOps4,										 
							__global float4* arrInPrims4,											 
	 					    __global float4* arrInMtxNode4,
							__constant struct CellParam* inCellParams,
							__read_only image2d_t texInVertexCountTable,
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
	
	//float cellsize = arrInHeader4[OFFSET_HEADER_CELLSIZE];	
	//Count : Prims, Ops, Mtx ; CellSize
	U32 ctPrims = (U32)arrInHeader4[OFFSET4_HEADER_PARAMS].x;
	U32 ctOps   = (U32)arrInHeader4[OFFSET4_HEADER_PARAMS].y;
	float cellsize = arrInHeader4[OFFSET4_HEADER_PARAMS].w;
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
		//arrFields[i] = ComputePrimitiveField(0, arrVertices[i], ops4, prims4, matrix4);
		arrFields[i] = ComputePrimitiveField(0, arrVertices[i], arrInOps4, arrInPrims4, arrInMtxNode4);
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
					      __constant struct CellParam* inCellParams,
						  __read_only image2d_t texInTriangleTable,
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
	//if(arrInVertexCount[idxCell] == 0)
		//return;
	
	//float cellsize = arrInHeader4[OFFSET_HEADER_CELLSIZE];	
	float cellsize = arrInHeader4[OFFSET4_HEADER_PARAMS].w;
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
		arrFields[i] = ComputePrimitiveField(0, arrVertices[i], arrInOps4, arrInPrims4, arrInMtxNode4);
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
		n = ComputeNormal(0, v, arrInOps4, arrInPrims4, arrInMtxNode4);

		//MeshAttrib index
		idxMeshAttrib = voffset + i;
		 
		arrOutMeshVertex[idxMeshAttrib] = v;
		arrOutMeshColor[idxMeshAttrib] = ComputePrimitiveColor(0, arrInPrims4);
		arrOutMeshNormal[idxMeshAttrib * 3] = n.x;
		arrOutMeshNormal[idxMeshAttrib * 3 + 1] = n.y;
		arrOutMeshNormal[idxMeshAttrib * 3 + 2] = n.z;
	//	printf("n = [%.2f, %.2f, %.2f] \n", n.x, n.y, n.z);  
	}
}