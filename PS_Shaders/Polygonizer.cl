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

#define ZERO4 {0.0f, 0.0f, 0.0f, 0.0f}
#define AXISX {1.0f, 0.0f, 0.0f, 0.0f}
#define AXISY {0.0f, 1.0f, 0.0f, 0.0f}
#define AXISZ {0.0f, 0.0f, 1.0f, 0.0f}

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




/*!
* Compute Cell Configuration index and Number Vertices Output for all cells.
*/
__kernel void ComputeConfigIndexVertexCount(const uint ctTotalCells,
											__global uchar* arrOutCellConfig,
											__global uchar* arrOutVertexCount,	
											__global float4* arrOutMeshVertex,
											__global float4* arrOutMeshColor,
											__global float* arrInHeader,
											__global float* arrInOps,										 
											__global float* arrInPrims,											 
											 __global float* arrInMtxNode,
											 const __global uint* arrInNeededCells,
											 __read_only image2d_t texInVertexCountTable,
											 __read_only image2d_t texInTriangleTable)
{
	//Get XY plane index
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= arrInNeededCells[0])||(idY >= arrInNeededCells[1])||(idZ >= arrInNeededCells[2]))
		return;
	uint idxCell = idZ * (arrInNeededCells[0] * arrInNeededCells[1]) + idY * arrInNeededCells[0] + idX;
	if(idxCell > ctTotalCells)
		return;

	__global float4* header4  = (__global float4 *)arrInHeader;
	__global float4* ops4     = (__global float4 *)arrInOps;
	__global float4* prims4   = (__global float4 *)arrInPrims;	
	__global float4* matrix4  = (__global float4 *)arrInMtxNode;

	//printf("Box Lo X = %.2f\n", arrInHeader[0]);
	//printf("Box Hi X = %.2f\n", header4[1].x);
	printf("Matrix 0,0 = %.2f \n", matrix4[0].x);
	//printf("Total Nodes = %d \n", ctTotalCells);

	float cellsize = arrInHeader[OFFSET_HEADER_CELLSIZE];	
	float4 lower = header4[0] + cellsize * (float4)(idX, idY, idZ, 0.0f);
	float arrFields[8];
	float4 arrVertices[8];

	arrVertices[0] = lower;
	arrVertices[1] = lower + cellsize * (float4)(1, 0, 0, 0);
	arrVertices[2] = lower + cellsize * (float4)(1, 1, 0, 0);
  	arrVertices[3] = lower + cellsize * (float4)(0, 1, 0, 0);
	arrVertices[4] = lower + cellsize * (float4)(0, 0, 1, 0);
	arrVertices[5] = lower + cellsize * (float4)(1, 0, 1, 0);
  	arrVertices[6] = lower + cellsize * (float4)(1, 1, 1, 0);
	arrVertices[7] = lower + cellsize * (float4)(0, 1, 1, 0);


	//Compute Fields
	for(int i=0; i<8; i++)
	{
		arrFields[i] = ComputePrimitiveField(0, arrVertices[i], ops4, prims4, matrix4);
		//arrFields[i] = ComputeOperatorField(0, arrVertices[i], arrInHeader, arrInOps, arrInPrims, arrInMtxNode);
	}
	
    //Compute Configuration index
    int idxConfig = 0;
	for(int i=0; i<8; i++)
	{
		if(isgreaterequal(arrFields[i], ISO_VALUE))
			idxConfig += (1 << i);
	}

    // read number of vertices from texture
    U8 ctVertices = read_imageui(texInVertexCountTable, tableSampler, (int2)(idxConfig,0)).x;
	U8 arrConfig[16];
	
	//Read configurations from texture memory
	for(int i=0; i<ctVertices; i++)
		arrConfig[i] = read_imageui(texInTriangleTable, tableSample, (int2)(idxConfig, i)).x;



	arrOutCellConfig[idxCell] = idxConfig;
	arrOutVertexCount[idxCell] = ctVertices;
	
}