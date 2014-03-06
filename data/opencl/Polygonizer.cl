//OpenCL BlobTree Polygonizer: Pourya Shirazian


//Enabling Newton-Raphson will take forever to compile cl file 
//#define USE_NEWTONRAPHSON

//DATASIZES
#define DATASIZE_HEADER		12
#define DATASIZE_OPERATOR	16
#define DATASIZE_PRIMITIVE	20
#define PRIM_MATRIX_STRIDE  12
#define BOX_MATRIX_STRIDE   16

#define DATASIZE_OPERATOR_F4	DATASIZE_OPERATOR/4
#define DATASIZE_PRIMITIVE_F4	DATASIZE_PRIMITIVE/4
#define PRIM_MATRIX_STRIDE_F4 	PRIM_MATRIX_STRIDE/4
#define BOX_MATRIX_STRIDE_F4 	BOX_MATRIX_STRIDE/4


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
#define OFFSET_HEADER_START 		11

//OFFSETS in OPERATORS
#define OFFSET4_OP_TYPE			0
#define OFFSET4_OP_RES			1
#define OFFSET4_AABB_LO			2
#define OFFSET4_AABB_HI			3

#define OFFSET_OP_TYPE			0
#define OFFSET_OP_LC			1
#define OFFSET_OP_RC			2
#define OFFSET_OP_NEXT			3
#define OFFSET_OP_RES			4
#define OFFSET_OP_FLAGS			7
#define OFFSET_OP_AABB_LO		8
#define OFFSET_OP_AABB_HI		12

//OFFSETS in PRIMITIVES
#define OFFSET4_PRIM_TYPE		0
#define OFFSET4_PRIM_POS		1
#define OFFSET4_PRIM_DIR		2
#define OFFSET4_PRIM_RES		3
#define OFFSET4_PRIM_COLOR		4

#define OFFSET_PRIM_TYPE		0
#define OFFSET_PRIM_IDX_MATRIX 	1
#define OFFSET_PRIM_PARENT 		2
#define OFFSET_PRIM_SIBLING 	3
#define OFFSET_PRIM_POS		 	4
#define OFFSET_PRIM_DIR	 		8
#define OFFSET_PRIM_RES		 	12
#define OFFSET_PRIM_COLOR	 	16

//Constants
#define ISO_VALUE 0.5f
#define NORMAL_DELTA 0.0001f
#define FIELD_VALUE_EPSILON 0.001f

#define ZERO4 {0.0f, 0.0f, 0.0f, 0.0f}
#define AXISX {1.0f, 0.0f, 0.0f, 0.0f}
#define AXISY {0.0f, 1.0f, 0.0f, 0.0f}
#define AXISZ {0.0f, 0.0f, 1.0f, 0.0f}

#define MAX_TREE_DEPTH 32
#define MAX_VERTICES_COUNT_PER_CELL		15
#define MAX_TRIANGLES_COUNT_PER_CELL	5


//Defines an empty index to jump out of branch
#define NULL_BLOB 0xFFFF

//MPU GRID
#define GRID_DIM_8

#ifdef GRID_DIM_32
#define GRID_DIM 32
#define CELLID_SHIFT_X 0
#define CELLID_SHIFT_Y 5
#define CELLID_SHIFT_Z 10
#define CELLID_BITMASK 0x1F
#endif

#ifdef GRID_DIM_16
#define GRID_DIM 16
#define CELLID_SHIFT_X 0
#define CELLID_SHIFT_Y 4
#define CELLID_SHIFT_Z 8
#define CELLID_BITMASK 0x0F
#endif

#ifdef GRID_DIM_8
#define GRID_DIM 8
#define CELLID_SHIFT_X 0
#define CELLID_SHIFT_Y 3
#define CELLID_SHIFT_Z 6
#define CELLID_BITMASK 0x07
#endif

#define CELLID_HASHSIZE (size_t)(1<<(3*CELLID_SHIFT_Y))
#define CELLID_FROM_IDX(i,j,k) ((((k) & CELLID_BITMASK) << CELLID_SHIFT_Z) | (((j) & CELLID_BITMASK) << CELLID_SHIFT_Y) | ((i) & CELLID_BITMASK))
#define EDGETABLE_DEPTH 8
#define NORMALIZE_FIELD(x) (2.0f * (0.5f + (x)) - 1.0f)


//Return codes
#define RET_PARAM_ERROR -1
#define RET_NOT_ENOUGH_MEM -2
#define RET_INVALID_BVH -3
#define RET_SUCCESS 1

//Comfort Types
typedef unsigned char		U8;
typedef unsigned short		U16;
typedef unsigned int		U32;
typedef			 char		I8;
typedef			 short		I16;
typedef			 int		I32;

//CellCorners
enum CellCorners {LBN, LBF, LTN, LTF, RBN, RBF, RTN, RTF};

//Types
enum PrimitiveType {primPoint, primLine, primCylinder, primDisc, primRing, primCube, 
					primTriangle, primQuadricPoint, primNULL, primInstance, primRBF};
					
enum OperatorType {opUnion, opIntersect, opDif, opSmoothDif, opBlend, opRicciBlend, 
				   opGradientBlend, opFastQuadricPointSet, opCache, opWarpTwist, 
				   opWarpTaper, opWarpBend, opWarpShear};
				   
enum OPFLAGS {ofRightChildIsOp = 1, ofLeftChildIsOp = 2, ofChildIndexIsRange = 4, ofIsUnaryOp = 8, ofIsRightOp = 16, ofBreak = 32};

//Sampler
__constant sampler_t tableSampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

//CellParam
typedef struct CellParam{
	U8 corner1[12];
	U8 corner2[12];
	U8 edgeaxis[12];
	U32 ctNeededCells[3];
	U32 ctTotalCells;
	float cellsize;
}CellParam;

//GridParam
typedef struct GridParam{
	U32 ctGridPoints[3];
	U32 ctTotalPoints;
	float cellsize;
}GridParam;


typedef struct StackU32{
	U32 values[MAX_TREE_DEPTH];
	U32 count;
}StackU32;

typedef struct StackF{
	float values[MAX_TREE_DEPTH];
	U32 count;
}StackF;

//Prototypes
void StackPush(struct StackU32* pstk, U32 val);
void StackPop(struct StackU32* pstk);
bool IsStackEmpty(struct StackU32* pstk);
U32 StackTop(struct StackU32* pstk);

void StackPushF(struct StackF* pstk, U32 val);
void StackPopF(struct StackF* pstk);
bool IsStackEmptyF(struct StackF* pstk);
float StackTopF(struct StackF* pstk);

float ComputeWyvillField(float dd);
bool IsOutsideOp(float4 v, U32 idxOp, __global float4* arrOps4);

float ComputeRangeField(float4 v, U32 idxOp, 
					    __global float4* arrOps4,
						__global float4* arrPrims4, 
						__global float4* arrMtxNodes4);
						
float ComputePrimitiveField(float4 v,
							U32 idxPrimitive,									
							__global float4* arrOps4,
							__global float4* arrPrims4, 
							__global float4* arrMtxNodes4);
						
float ComputeBranchField(U32 idxBranchOp,
						 U32* lpNextOp,		
						 bool* lpIsRightOp, 
						 float4 v,
						 float lf,
						 float rf,
						 __global float4* arrHeader4,
						 __global float4* arrOps4,
						 __global float4* arrPrims4, 
						 __global float4* arrMtxNodes4);

float3 ComputeNormal(float4 v,
		   __global float4* arrHeader4, 
		   __global float4* arrOps4,
		   __global float4* arrPrims4,
		   __global float4* arrMtxNodes4);			

float ComputeFieldStackBased(float4 v,			
						   __global float4* arrHeader4,
						   __global float4* arrOps4,
						   __global float4* arrPrims4, 
						   __global float4* arrMtxNodes4);

						
float4 ComputeGradientWithPrevField(float4 v,
								   __global float4* arrHeader4, 
								   __global float4* arrOps4,
								   __global float4* arrPrims4,
								   __global float4* arrMtxNodes4);
								   
//////////////////////////////////////////////////////////////////////
//Stack Operations
void StackPush(struct StackU32* pstk, U32 val)
{
	pstk->values[pstk->count] = val;
	pstk->count++;			
}

void StackPop(struct StackU32* pstk)
{
	pstk->count--;
}

bool IsStackEmpty(struct StackU32* pstk)
{
	return (pstk->count == 0);
}

U32 StackTop(struct StackU32* pstk)
{
	return pstk->values[pstk->count - 1];
}


//Float Stack
void StackPushF(struct StackF* pstk, U32 val)
{
	pstk->values[pstk->count] = val;
	pstk->count++;			
}

void StackPopF(struct StackF* pstk)
{
	pstk->count--;
}

bool IsStackEmptyF(struct StackF* pstk)
{
	return (pstk->count == 0);
}

float StackTopF(struct StackF* pstk)
{
	return pstk->values[pstk->count - 1];
}

//Computes Wyvill Field-Function
float ComputeWyvillField(float dd)
{
	if(isgreater(dd, 1.0f))
		return 0.0f;
	float t = (1.0f - dd);
	return t*t*t;
}


//Check weather a given vertex is outside the specified op
bool IsOutsideOp(float4 v, U32 idxOp, __global float4* arrOps4)
{
	float4 aabbLo = arrOps4[idxOp * DATASIZE_OPERATOR + OFFSET4_AABB_LO];
	float4 aabbHi = arrOps4[idxOp * DATASIZE_OPERATOR + OFFSET4_AABB_HI];
	return !(isgreaterequal(v.x, aabbLo.x) && isgreaterequal(aabbHi.x, v.x) &&
			isgreaterequal(v.y, aabbLo.y) && isgreaterequal(aabbHi.y, v.y) &&
			isgreaterequal(v.z, aabbLo.z) && isgreaterequal(aabbHi.z, v.z));
}



/*
float ComputeTriangleSquareDist(float4 v, float4 vertices[3],
								 float4* outClosestPoint, float4* outBaryCoords) {
	float4 dif = vertices[0] - v;
	float4 edge0 = vertices[1] - vertices[0];
	float4 edge1 = vertices[2] - vertices[0];
	float a00 = dot(edge0, edge0);
	float a01 = dot(edge0, edge1);
	float a11 = dot(edge1, edge1);

	float b0 = dot(dif, edge0);
	float b1 = dot(dif, edge1);
	float c = dot(dif, dif);
	float det = fabs(a00 * a11 - a01 * a01);
	float s = a01 * b1 - a11 * b0;
	float t = a01 * b0 - a00 * b1;
	float sqrDistance;

	//Conditions
	if (s + t <= det) {
		if (s < (float) 0) {
			if (t < (float) 0) // region 4
					{
				if (b0 < (float) 0) {
					t = (float) 0;
					if (-b0 >= a00) {
						s = (float) 1;
						sqrDistance = a00 + ((float) 2) * b0 + c;
					} else {
						s = -b0 / a00;
						sqrDistance = b0 * s + c;
					}
				} else {
					s = (float) 0;
					if (b1 >= (float) 0) {
						t = (float) 0;
						sqrDistance = c;
					} else if (-b1 >= a11) {
						t = (float) 1;
						sqrDistance = a11 + ((float) 2) * b1 + c;
					} else {
						t = -b1 / a11;
						sqrDistance = b1 * t + c;
					}
				}
			} else // region 3
			{
				s = (float) 0;
				if (b1 >= (float) 0) {
					t = (float) 0;
					sqrDistance = c;
				} else if (-b1 >= a11) {
					t = (float) 1;
					sqrDistance = a11 + ((float) 2) * b1 + c;
				} else {
					t = -b1 / a11;
					sqrDistance = b1 * t + c;
				}
			}
		} else if (t < (float) 0) // region 5
				{
			t = (float) 0;
			if (b0 >= (float) 0) {
				s = (float) 0;
				sqrDistance = c;
			} else if (-b0 >= a00) {
				s = (float) 1;
				sqrDistance = a00 + ((float) 2) * b0 + c;
			} else {
				s = -b0 / a00;
				sqrDistance = b0 * s + c;
			}
		} else // region 0
		{
			// minimum at interior point
			float invDet = ((float) 1) / det;
			s *= invDet;
			t *= invDet;
			sqrDistance = s * (a00 * s + a01 * t + ((float) 2) * b0)
					+ t * (a01 * s + a11 * t + ((float) 2) * b1) + c;
		}
	} else {
		float tmp0, tmp1, numer, denom;

		if (s < (float) 0) // region 2
				{
			tmp0 = a01 + b0;
			tmp1 = a11 + b1;
			if (tmp1 > tmp0) {
				numer = tmp1 - tmp0;
				denom = a00 - ((float) 2) * a01 + a11;
				if (numer >= denom) {
					s = (float) 1;
					t = (float) 0;
					sqrDistance = a00 + ((float) 2) * b0 + c;
				} else {
					s = numer / denom;
					t = (float) 1 - s;
					sqrDistance = s * (a00 * s + a01 * t + ((float) 2) * b0)
							+ t * (a01 * s + a11 * t + ((float) 2) * b1) + c;
				}
			} else {
				s = (float) 0;
				if (tmp1 <= (float) 0) {
					t = (float) 1;
					sqrDistance = a11 + ((float) 2) * b1 + c;
				} else if (b1 >= (float) 0) {
					t = (float) 0;
					sqrDistance = c;
				} else {
					t = -b1 / a11;
					sqrDistance = b1 * t + c;
				}
			}
		} else if (t < (float) 0) // region 6
				{
			tmp0 = a01 + b1;
			tmp1 = a00 + b0;
			if (tmp1 > tmp0) {
				numer = tmp1 - tmp0;
				denom = a00 - ((float) 2) * a01 + a11;
				if (numer >= denom) {
					t = (float) 1;
					s = (float) 0;
					sqrDistance = a11 + ((float) 2) * b1 + c;
				} else {
					t = numer / denom;
					s = (float) 1 - t;
					sqrDistance = s * (a00 * s + a01 * t + ((float) 2) * b0)
							+ t * (a01 * s + a11 * t + ((float) 2) * b1) + c;
				}
			} else {
				t = (float) 0;
				if (tmp1 <= (float) 0) {
					s = (float) 1;
					sqrDistance = a00 + ((float) 2) * b0 + c;
				} else if (b0 >= (float) 0) {
					s = (float) 0;
					sqrDistance = c;
				} else {
					s = -b0 / a00;
					sqrDistance = b0 * s + c;
				}
			}
		} else // region 1
		{
			numer = a11 + b1 - a01 - b0;
			if (numer <= (float) 0) {
				s = (float) 0;
				t = (float) 1;
				sqrDistance = a11 + ((float) 2) * b1 + c;
			} else {
				denom = a00 - ((float) 2) * a01 + a11;
				if (numer >= denom) {
					s = (float) 1;
					t = (float) 0;
					sqrDistance = a00 + ((float) 2) * b0 + c;
				} else {
					s = numer / denom;
					t = (float) 1 - s;
					sqrDistance = s * (a00 * s + a01 * t + ((float) 2) * b0)
							+ t * (a01 * s + a11 * t + ((float) 2) * b1) + c;
				}
			}
		}
	}

	// Account for numerical round-off error.
	if (sqrDistance < (float) 0) {
		sqrDistance = (float) 0;
	}

	//mClosestPoint0 = *mPoint;
	*outClosestPoint = vertices[0] + s * edge0 + t * edge1;
	(*outBaryCoords).y = s;
	(*outBaryCoords).z = t;
	(*outBaryCoords).x = (float) 1 - s - t;
	return sqrDistance;
}
*/

/*!
 * Compute field due to a primitive.
 */
float ComputePrimitiveField(float4 v,
							U32 idxPrimitive,
							__global float4* arrOps4,
							__global float4* arrPrims4, 
							__global float4* arrMtxNodes4)
{
	//Transform
	int primType  = (int)(arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4].x);		
	float4 vt = v;
	{		 
		int idxMatrix = (int)(arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4].y);		
		//printf("idxMatrix = %d, \n", idxMatrix);
		//printf("V.W: %.2f \n", v.w);
		
		float4 row0 = arrMtxNodes4[idxMatrix * PRIM_MATRIX_STRIDE_F4];
		float4 row1 = arrMtxNodes4[idxMatrix * PRIM_MATRIX_STRIDE_F4 + 1];
		float4 row2 = arrMtxNodes4[idxMatrix * PRIM_MATRIX_STRIDE_F4 + 2];
		//printf("idxPrim: %d, idxMatrix: %d, MTX row0: [%.2f, %.2f, %.2f, %.2f]\n", idxPrimitive, idxMatrix, row0.x, row0.y, row0.z, row0.w);
		vt = (float4)(dot(row0, v), dot(row1, v), dot(row2, v), 0.0f);
	}
	
	//Handle Instanced Node First
	if(primType == primInstance)
	{
		//resX: idxArray, resY: idxScript, resZ: isOriginOp, dirX: originNodeType
		U32 idxOrigin = (U32)arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		bool isOriginOp = (((int)arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].z) == 1);
		U32 originType = (U32)arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR].x;

		return 0.0f;
		/*
		if(isOriginOp) {
			if(IsOutsideOp(vt, idxOrigin, arrOps4))
				return 0.0f;
			else 
			{
				//Instanced nodes can only be range for now.
				U32 flags = (U32)arrOps4[idxOrigin * DATASIZE_OPERATOR_F4 + OFFSET4_OP_RES].w;
				bool bRange  = (flags & ofChildIndexIsRange) >> 2;
				if(bRange)
					return ComputeRangeField(vt, idxOrigin, arrOps4, arrPrims4, arrMtxNodes4);
				else 
					return 0.0f;
			}
		}
		else
			return ComputePrimitiveField(idxOrigin, vt, arrOps4, arrPrims4, arrMtxNodes4);
		 */
	}
		

	//All Other primitives
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
		/*
        float4 vertices[3];
        vertices[0] = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
        vertices[1] = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR];
        vertices[2] = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES];
        float4 outClosest;
        float4 outBaryCoords;
        dist2 = ComputeTriangleSquareDist(vt, vertices, &outClosest, &outBaryCoords);
        */
		dist2 = 10.0f;
	}
	break;	

	case(primCube):
	{
		float4 dif = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float side = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
				
		float projected;
		float delta;
			
		//Along X
		dist2 = 0.0f;
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
	
		
	}

 	return ComputeWyvillField(dist2); 
}

float ComputeOpField(int opType, float lf, float rf, float4 params) {
	float field = 0.0f;
	//Compute All Operators			
	switch(opType) {
		case(opUnion): {
			field = max(lf, rf);
			break;
		}
		case(opIntersect): {
			field = min(lf, rf);
			break;
		}										
		case(opBlend): {
			field = lf + rf;
			break;
		}					
		case(opRicciBlend): {
			field = pow(pow(lf, params.x) + pow(rf, params.x), params.y);
			break;
		}		

		case(opDif): {
			field = min(lf, 1.0f - rf); 
			break;
		}
		
		case(opSmoothDif): {
			field = lf * (1.0f - rf);
			break;
		}
	}
	return field;
}

float ComputeRangeField(float4 v, U32 idxOp, 						
					    __global float4* arrOps4,
						__global float4* arrPrims4, 
						__global float4* arrMtxNodes4)
{
	//Homogenous coords needed
	v.w = 1.0f;
	U32 opType  = (U32)arrOps4[idxOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].x;
	U32 idxFrom = (U32)arrOps4[idxOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].y;
	U32 idxTo   = (U32)arrOps4[idxOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].z;
	float4 params = arrOps4[idxOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_RES];

	
	float field = 0.0f;
	switch(opType) {
		case(opUnion): {
			for(U32 i=idxFrom; i <= idxTo; i++)									
				field = max(field, ComputePrimitiveField(v, i, arrOps4, arrPrims4, arrMtxNodes4));
		}
		break;
		case(opIntersect): {
			for(U32 i=idxFrom; i <= idxTo; i++)									
				field = min(field, ComputePrimitiveField(v, i, arrOps4, arrPrims4, arrMtxNodes4));
		}
		break;
	
		case(opBlend): {
			for(U32 i=idxFrom; i <= idxTo; i++)									
				field += ComputePrimitiveField(v, i, arrOps4, arrPrims4, arrMtxNodes4);				
		}
		break;
		case(opRicciBlend): {
			for(U32 i=idxFrom; i <= idxTo; i++)									
				field = pow(pow(field, params.x) + pow(ComputePrimitiveField(v, i, arrOps4, arrPrims4, arrMtxNodes4), params.x), params.y); 				
		}
		break;
	}
	
	return field;
}



/*!
 * Compute field due to a branch in the tree.
 * At Ops: 1. Evaluate Left Prim and Right Prim
 * 		   2. Use Left Field or Right Field if the child is op
 * 		   3. Jump to next op or return
 * 	    
 */
float ComputeBranchField(U32 idxBranchOp,
						 U32* lpNextOp,		
						 bool* lpIsRightOp, 
						 float4 v,
						 float lf,
						 float rf,
						 __global float4* arrHeader4,
						 __global float4* arrOps4,
						 __global float4* arrPrims4, 
						 __global float4* arrMtxNodes4)
{	
	float field = 0.0f;
	while(idxBranchOp != NULL_BLOB)
	{		
		U32 opType = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].x;
		U32 opLeftChild = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].y;
		U32 opRightChild = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].z;

		//Next Node
		*lpNextOp = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].w;
	
		float4 params = arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_RES];
		U32 flags = (U32)params.w;
		
		//enum OPFLAGS {ofRightChildIsOp = 1, ofLeftChildIsOp = 2, ofChildIndexIsRange = 4, ofIsUnaryOp = 8, ofIsRightOp = 16, ofBreak = 32};
		bool isBreak = (flags & ofBreak) >> 5;
		*lpIsRightOp = (flags & ofIsRightOp) >> 4;
		bool bUnary  = (flags & ofIsUnaryOp) >> 3;
		bool bRange  = (flags & ofChildIndexIsRange) >> 2;
		bool bLeftChildOp  = (flags & ofLeftChildIsOp) >> 1;
		bool bRightChildOp = (flags & ofRightChildIsOp);
		field = 0.0f;

		//Range
		if(bRange)
			field = ComputeRangeField(v, idxBranchOp, arrOps4, arrPrims4, arrMtxNodes4);					
		else
		{
			if(!bLeftChildOp)
				lf = ComputePrimitiveField(v, opLeftChild, arrOps4, arrPrims4, arrMtxNodes4);
			
			if(!bUnary && !bRightChildOp)
				rf = ComputePrimitiveField(v, opRightChild, arrOps4, arrPrims4, arrMtxNodes4);
			
			field = ComputeOpField(idxBranchOp, lf, rf, params);
		}	
		
		//If we processed a right op
		if((*lpIsRightOp) == true)
			rf = field;
		else
			lf = field;
		
		//If we processed a break op
		if(isBreak)
			break;

		//idxBranchOp
		idxBranchOp = *lpNextOp;
	}
	
	return field;
}

/*!
 * Stackless Non-Recursive BlobTree Traversal
 */
float ComputeField(float4 v,			
				   __global float4* arrHeader4,
		  		   __global float4* arrOps4,
		   	 	   __global float4* arrPrims4, 
		   		   __global float4* arrMtxNodes4)
{
	//Count : Prims, Ops, Mtx ; CellSize
	//U32 ctPrims = (U32)arrHeader4[OFFSET4_HEADER_PARAMS].x;
	U32 ctOps   = (U32)arrHeader4[OFFSET4_HEADER_PARAMS].y;

	//Should be in homogenous coordinates
	v.w = 1.0f;

	if(ctOps > 0)
	{
		float lf = 0.0f;
		float rf = 0.0f;
		float field;
		bool isRightOp;
		
		U32 idxBranchOp = arrHeader4[OFFSET4_HEADER_PARAMS].w;
		U32 idxNextOp = NULL_BLOB;
		while(idxBranchOp != NULL_BLOB)
		{
			//Compute Right Branch
			field = ComputeBranchField(idxBranchOp, &idxNextOp, &isRightOp, v, lf, rf, arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);			
			if(isRightOp)
				rf = field;
			else
				lf = field;
			
			idxBranchOp = idxNextOp;
		}
		
		return field;
	}
	else
		return ComputePrimitiveField(v, 0, arrOps4, arrPrims4, arrMtxNodes4);
}

/*!
 * Compute field and color due to a branch in the tree.
 * At Ops: 1. Evaluate Left Prim and Right Prim
 * 		   2. Use Left Field or Right Field if the child is op
 * 		   3. Jump to next op or return
 * 	    
 */
float ComputeBranchFieldAndColor(U32 idxBranchOp,
								 U32* lpNextOp,		
								 bool* lpIsRightOp, 
								 float4 v,
								 float4* lpOutColor,
								 float lf,
								 float4 lcolor,
								 float rf,
								 float4 rcolor,
								 __global float4* arrHeader4,
								 __global float4* arrOps4,
								 __global float4* arrPrims4, 
								 __global float4* arrMtxNodes4)
{	
	float field = 0.0f;
	while(idxBranchOp != NULL_BLOB)
	{			
		U32 opType = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].x;
		U32 opLeftChild = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].y;
		U32 opRightChild = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].z;

		//Next Node
		*lpNextOp = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].w;

		//Params
		float4 params = arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_RES];
		U32 flags = params.w;		
		
		//enum OPFLAGS {ofRightChildIsOp = 1, ofLeftChildIsOp = 2, ofChildIndexIsRange = 4, ofIsUnaryOp = 8, ofIsRightOp = 16, ofBreak = 32};
		bool isBreak = (flags & ofBreak) >> 5;
		*lpIsRightOp = (flags & ofIsRightOp) >> 4;
		bool bUnary = (flags & ofIsUnaryOp) >> 3;
		bool bRange = (flags & ofChildIndexIsRange) >> 2;
		bool bLeftChildOp = (flags & ofLeftChildIsOp) >> 1;
		bool bRightChildOp = (flags & ofRightChildIsOp);
		field = 0.0f;

		//Range
		if(bRange)
		{				
			float primF;
			
			switch(opType) {
				case(opUnion): {				
						U32 idxMaxField = 0;
						field = ComputePrimitiveField(v, opLeftChild, arrOps4, arrPrims4, arrMtxNodes4);						
						for(U32 i=opLeftChild+1; i <= opRightChild; i++)
						{
							primF = ComputePrimitiveField(v, i, arrOps4, arrPrims4, arrMtxNodes4);
							if(isgreaterequal(primF, field))
							{
								field = primF;
								idxMaxField = i;
							}
						}					
						(*lpOutColor) = NORMALIZE_FIELD(field) * arrPrims4[idxMaxField * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
				}
				break;
				case(opIntersect): {
					U32 idxMinField = 0;
					field = ComputePrimitiveField(v, opLeftChild, arrOps4, arrPrims4, arrMtxNodes4);
					for(U32 i=opLeftChild+1; i <= opRightChild; i++)
					{
						primF = ComputePrimitiveField(v, i, arrOps4, arrPrims4, arrMtxNodes4);
						if(islessequal(primF, field))
						{
							field = primF;
							idxMinField = i;
						}
					}					
					(*lpOutColor) = NORMALIZE_FIELD(field) * arrPrims4[idxMinField * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
				}
				break;
	
				case(opBlend): {								
					for(U32 i=opLeftChild; i <= opRightChild; i++)	
					{				
						primF = ComputePrimitiveField(v, i, arrOps4, arrPrims4, arrMtxNodes4);
						(*lpOutColor) += NORMALIZE_FIELD(primF) * arrPrims4[i * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
						field += primF;
					}				
				}
				break;
				case(opRicciBlend): {
					for(U32 i=opLeftChild; i <= opRightChild; i++)	
					{				
						primF = ComputePrimitiveField(v, i, arrOps4, arrPrims4, arrMtxNodes4);
						(*lpOutColor) += NORMALIZE_FIELD(primF) * arrPrims4[i * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
						field = pow(pow(field, params.x) + pow(primF, params.x), params.y);
					}				
				}
				break;				
			}
		}
		else
		{
			if(!bLeftChildOp) {
				lf = ComputePrimitiveField(v, opLeftChild, arrOps4, arrPrims4, arrMtxNodes4);
				lcolor = arrPrims4[opLeftChild * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
			}
			
			if(!bUnary && !bRightChildOp) {
				rf = ComputePrimitiveField(v, opRightChild, arrOps4, arrPrims4, arrMtxNodes4);
				rcolor = arrPrims4[opRightChild * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
			}
		
			//Normalized fields from (0 - 0.5) to (0 - 1)
			float lfn = NORMALIZE_FIELD(lf);
			float rfn = NORMALIZE_FIELD(rf);
			field = ComputeOpField(opType, lf, rf, params);

			//Compute All Operators			
			switch(opType) {
				case(opUnion): {
					if(isgreaterequal(lfn, rfn))
						(*lpOutColor) = lfn * lcolor;
					else
						(*lpOutColor) = rfn * rcolor;
				}
				break;
				case(opIntersect): {
					if(islessequal(lfn, rfn))
						(*lpOutColor) = lcolor;
					else
						(*lpOutColor) = rcolor;
				}					
				break;
					
				case(opBlend): {
					(*lpOutColor) = lfn * lcolor + rfn * rcolor; 									
				}					
				break;

				case(opRicciBlend): {
					(*lpOutColor) = lfn * lcolor + rfn * rcolor;
				}
				break;
				
				case(opDif): { 
					if(islessequal(lf, 1.0f - rf))
						(*lpOutColor) = lcolor;
					else
						(*lpOutColor) = rcolor;
				}
				break;
				
				case(opSmoothDif): {
					if(islessequal(lf, 1.0f - rf))
						(*lpOutColor) = lcolor;
					else
						(*lpOutColor) = rcolor;
				}
				break;

			}
		}	
		
		//If we processed a right op
		if((*lpIsRightOp) == true) {
			rcolor = (*lpOutColor);
			rf = field;
		}
		else 
		{
			lcolor = (*lpOutColor);
			lf = field;
		}
		
		//If we processed a break op
		if(isBreak)
			break;

		//idxBranchOp
		idxBranchOp = *lpNextOp;
	} //END WHILE
	
	return field;
}

/*!
 * Compute Field-Value and Color
 */
float ComputeFieldAndColor(float4 v,	
							float4* lpOutColor,
						   __global float4* arrHeader4,
						   __global float4* arrOps4,
						   __global float4* arrPrims4, 
						   __global float4* arrMtxNodes4)
{
	//Count : Prims, Ops, Mtx ; CellSize
	//U32 ctPrims = (U32)arrHeader4[OFFSET4_HEADER_PARAMS].x;
	U32 ctOps   = (U32)arrHeader4[OFFSET4_HEADER_PARAMS].y;
	(*lpOutColor) = (float4)(0,0,0,0);

	//Should be in homogenous coordinates
	v.w = 1.0f;

	if(ctOps > 0)
	{
		float lf = 0.0f;
		float rf = 0.0f;
		float4 lcolor = (float4)(0,0,0,0);
		float4 rcolor = (float4)(0,0,0,0);
		float field;
		bool isRightOp;
		
		U32 idxBranchOp = arrHeader4[OFFSET4_HEADER_PARAMS].w;
		U32 idxNextOp = NULL_BLOB;
		while(idxBranchOp != NULL_BLOB)
		{
			//enum OPFLAGS {ofRightChildIsOp = 1, ofLeftChildIsOp = 2, ofChildIndexIsRange = 4, ofIsUnaryOp = 8, ofIsRightOp = 16};
			//U32 flags = (U32)arrOps4[idxBranchOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].z;							
			field = ComputeBranchFieldAndColor(idxBranchOp, &idxNextOp, &isRightOp, v, lpOutColor, 
											   lf, lcolor, rf, rcolor, arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);
			if(isRightOp) {
				rf = field;
				rcolor = *lpOutColor;
			}
			else {				
				lf = field;
				lcolor = *lpOutColor;
			}
			
			idxBranchOp = idxNextOp;
		}
		
		return field;
	}
	else
	{
		(*lpOutColor) = arrPrims4[OFFSET4_PRIM_COLOR];
		return ComputePrimitiveField(v, 0, arrOps4, arrPrims4, arrMtxNodes4);
	}
}


/*!
 * Compute Gradient and FieldValue, Uses 4 evals
 */
float3 ComputeGradientAndField(float4 v,
							   float* outField,
							   __global float4* arrHeader4, 
							   __global float4* arrOps4,
							   __global float4* arrPrims4,
							   __global float4* arrMtxNodes4)
{
	const float deltaInv = 1.0f / NORMAL_DELTA;	
	*outField = ComputeField(v, arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);	
	float3 grad;
	grad.x = ComputeField(v + (float4)(NORMAL_DELTA,0,0,0), arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);
	grad.y = ComputeField(v + (float4)(0, NORMAL_DELTA, 0, 0), arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);
	grad.z = ComputeField(v + (float4)(0, 0, NORMAL_DELTA, 0), arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);

	grad = deltaInv * (grad - (float3)(*outField, *outField, *outField)); 
	return grad;
}


/*!
 * Compute Gradient and FieldValue, Uses 4 evals
 */
float4 ComputeGradientWithPrevField(float4 v,
								   __global float4* arrHeader4, 
								   __global float4* arrOps4,
								   __global float4* arrPrims4,
								   __global float4* arrMtxNodes4)
{
	float inField = v.w;
	v.w = 1.0f;
	
	float4 grad;
	grad.x = ComputeField(v + (float4)(NORMAL_DELTA,0,0,0), arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);
	grad.y = ComputeField(v + (float4)(0, NORMAL_DELTA, 0, 0), arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);
	grad.z = ComputeField(v + (float4)(0, 0, NORMAL_DELTA, 0), arrHeader4, arrOps4, arrPrims4, arrMtxNodes4);
	grad.w = 0;

	grad = (1.0f / NORMAL_DELTA) * (grad - (float4)(inField, inField, inField, 0)); 
	return grad;
}

/*!
 * Computes the root using shrink-wrap method and the root marching 
 * Algorithm.
 */
float4 ComputeRootNewtonRaphson(float4 a, float4 b, 
								__global float4* arrInHeader4,
								__global float4* arrInOps4,										 
								__global float4* arrInPrims4,											 
								__global float4* arrInMtxNodes4) {
	
	//Number of iterations to get to the root is 8	
	float4 x;
	float4 grad;
	
	if(isless(fabs(a.w - ISO_VALUE), fabs(b.w - ISO_VALUE))) {
		x = a;
	}
	else {
		x = b;
	}

	//March toward the iso-surface
	for(int i=0; i<8; i++)
	{
		grad = ComputeGradientWithPrevField(x, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		
		//Uses shrink-wrap method to converge to surface
		x += ((ISO_VALUE - x.w) * grad) / dot(grad, grad);
		
		//Compute field for the displaced vertex. x.w will be set to 1 in ComputeField to be in homogenous
		x.w = ComputeField(x, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		
		if(isless(fabs(x.w - ISO_VALUE), FIELD_VALUE_EPSILON))
			break;
	}

	return x;
}


__kernel void ComputeAllFields(__global float4* arrInHeader4,
							   __global float4* arrInOps4,										 
							   __global float4* arrInPrims4,											 
							   __global float4* arrInMtxNodes4,
							   __constant struct GridParam* inGridParam,
							   __global float4* arrOutFields)			
{
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inGridParam->ctGridPoints[0])||(idY >= inGridParam->ctGridPoints[1])||(idZ >= inGridParam->ctGridPoints[2]))
		return;
	U32 idxVertex = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + idX;
	if(idxVertex >= inGridParam->ctTotalPoints)
		return;
	//printf("IDXVertex = %d\n", idxVertex);
	
	float cellsize = inGridParam->cellsize;		
	float4 v = arrInHeader4[OFFSET4_HEADER_LOWER] + cellsize * (float4)(idX, idY, idZ, 0.0f);
	v.w = ComputeField(v, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);	
	arrOutFields[idxVertex] = v;
}

__kernel void ComputeAllFieldsStackBased(__global float4* arrInHeader4,
							   __global float4* arrInOps4,										 
							   __global float4* arrInPrims4,											 
							   __global float4* arrInMtxNodes4,
							   __constant struct GridParam* inGridParam,
							   __global float4* arrOutFields)			
{
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inGridParam->ctGridPoints[0])||(idY >= inGridParam->ctGridPoints[1])||(idZ >= inGridParam->ctGridPoints[2]))
		return;
	U32 idxVertex = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + idX;
	if(idxVertex >= inGridParam->ctTotalPoints)
		return;
	//printf("IDXVertex = %d\n", idxVertex);
	
	float cellsize = inGridParam->cellsize;		
	float4 v = arrInHeader4[OFFSET4_HEADER_LOWER] + cellsize * (float4)(idX, idY, idZ, 0.0f);
	v.w	= ComputeFieldStackBased(v, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
	arrOutFields[idxVertex] = v;
}

//Computes field value for an array of vertices
__kernel void ComputeFieldArray(U32 ctVertices,
							   __global float4* arrInHeader4,
							   __global float4* arrInOps4,										 
							   __global float4* arrInPrims4,											 
							   __global float4* arrInMtxNodes4,							
							   __global float4* arrInOutVertexFields)			
{
	int idX = get_global_id(0);
	if(idX >= ctVertices)
		return;
	arrInOutVertexFields[idX].w	= ComputeField(arrInOutVertexFields[idX], arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
}

__kernel void ComputeFieldArrayStackBased(U32 ctVertices,
							   __global float4* arrInHeader4,
							   __global float4* arrInOps4,										 
							   __global float4* arrInPrims4,											 
							   __global float4* arrInMtxNodes4,							
							   __global float4* arrInOutVertexFields)			
{
	int idX = get_global_id(0);
	if(idX >= ctVertices)
		return;
	arrInOutVertexFields[idX].w	= ComputeFieldStackBased(arrInOutVertexFields[idX], arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
}

__kernel void ComputeFieldImage(float4 lo, float4 hi, 
							    int dimx, int dimy,
								__global float4* arrInHeader4,
							   __global float4* arrInOps4,										 
							   __global float4* arrInPrims4,											 
							   __global float4* arrInMtxNodes4,							
							   __write_only image2d_t fields) {

	int2 coords = (int2){ get_global_id(0), get_global_id(1)};
	if(coords.x >= dimx || coords.y >= dimy)
		return;
	
	float dx = (float)(coords.x) / (float)dimx;
	float dy = (float)(coords.y) / (float)dimy;
	
	bool xset = 0;
	float4 d = (hi - lo);
	d.w = 0.0f;
	if(isnotequal(d.x, 0.0f)) {
		d.x = dx;
		xset = 1;
	}
	
	if(isnotequal(d.y, 0.0f))
		d.y = (xset == 1) ? dy : dx;
	if(isnotequal(d.z, 0.0f))
		d.z = dy;
	
				
	float4 p = lo + d * (hi - lo);
	float f = ComputeField(p, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
	float4 pixel = (float4)(f, f, f, 1.0f);

	//write_imagef(fields, coords, pixel);	
	
	uint4 color = (uint4)(0, 255, 0, 255);					
	write_imageui(fields, coords, color);						
}

//Per each vertex computes 2 offscreen points and their associated field-values
//Normals determine the direction of extrusion and intrusion
__kernel void ComputeOffSurfacePointsAndFields(float len,
											  U32 ctVertices,											 											
											  __global float4* arrInMeshVertex,
											  __global float3* arrInMeshNormal,						  								   
											  __global float4* arrInHeader4,
											  __global float4* arrInOps4,										 
											  __global float4* arrInPrims4,											 
											  __global float4* arrInMtxNodes4,									
											  __global float4* arrOutOffSurfaceXYZF) 
{
	int idX = get_global_id(0);
	if(idX >= ctVertices)
		return;
	
	float4 ptOut = arrInMeshVertex[idX] + len * (float4)(arrInMeshNormal[idX], 0);
	float4 ptIn = arrInMeshVertex[idX] - len * (float4)(arrInMeshNormal[idX], 0);
	ptOut.w		= ComputeField(ptOut, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
	ptIn.w		= ComputeField(ptIn, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
	
	arrOutOffSurfaceXYZF[idX*2] = ptOut;
	arrOutOffSurfaceXYZF[idX*2+1] = ptIn;
}

//Computes the edgetable for polygonizer
__kernel void ComputeEdgeTable(__global float4* arrInFields,
							   __constant struct GridParam* inGridParam,
							   __global U32* arrOutHighEdgesCount,
							   __global U8* arrOutHighEdgesFlags)
{
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inGridParam->ctGridPoints[0])||(idY >= inGridParam->ctGridPoints[1])||(idZ >= inGridParam->ctGridPoints[2]))
		return;
	U32 idxVertex = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + idX;
	if(idxVertex >= inGridParam->ctTotalPoints)
		return;

	bool bHot = isgreaterequal(arrInFields[idxVertex].w, ISO_VALUE); 
	bool bHotNbor = false;
	U32 idxVertexNbor = 0;
	
	//high edges
	U8 ctCrossed = 0;
	U8 flag = 0;
	
	//XYZ: 421
	//X
	idxVertexNbor = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + (idX+1);
	if((idxVertexNbor < inGridParam->ctTotalPoints) && ((idX+1) < inGridParam->ctGridPoints[0]))
	{
		bHotNbor = isgreaterequal(arrInFields[idxVertexNbor].w, ISO_VALUE); 
		if(bHot ^ bHotNbor)
		{
			ctCrossed ++;
			flag |= 4;
		}
	}

	//Y
	idxVertexNbor = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + (idY+1) * inGridParam->ctGridPoints[0] + idX;
	if((idxVertexNbor < inGridParam->ctTotalPoints) && ((idY+1) < inGridParam->ctGridPoints[1]))
	{
		bHotNbor = isgreaterequal(arrInFields[idxVertexNbor].w, ISO_VALUE); 
		if(bHot ^ bHotNbor)
		{
			ctCrossed ++;
			flag |= 2;
		}
	}

	//Z
	idxVertexNbor = (idZ+1) * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + idX;
	if((idxVertexNbor < inGridParam->ctTotalPoints) && ((idZ+1) < inGridParam->ctGridPoints[2]))
	{
		bHotNbor = isgreaterequal(arrInFields[idxVertexNbor].w, ISO_VALUE); 
		if(bHot ^ bHotNbor)
		{
			ctCrossed ++;
			flag |= 1;
		}
	}

	//Write to output
	arrOutHighEdgesCount[idxVertex] = ctCrossed;
	arrOutHighEdgesFlags[idxVertex] = flag; 
}

__kernel void ApplyVertexDeformations(U32 ctVertices,
									  __global float4* arrInRestPos,
									  __global float4* arrInDeformation,
									  __global float4* arrOutMeshVertex) {
	int idX = get_global_id(0);
	if(idX >= ctVertices)
		return;
	
	arrOutMeshVertex[idX] = arrInRestPos[idX] + arrInDeformation[idX];
}

//Compute Vertex Attribs
__kernel void ComputeVertexAttribs(__global float4* arrInHeader4,
								   __global float4* arrInOps4,										 
								   __global float4* arrInPrims4,											 
								   __global float4* arrInMtxNodes4,
								   __global float4* arrInFields,
								   __global U32* arrInHighEdgesCount,
								   __global U32* arrInHighEdgesOffset,
								   __global U8* arrInHighEdgesFlags,
								   __constant struct GridParam* inGridParam,
								   __global float4* arrOutMeshVertex,
								   __global float4* arrOutMeshColor,
								   __global float* arrOutMeshNormal)						  								   
{
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inGridParam->ctGridPoints[0])||(idY >= inGridParam->ctGridPoints[1])||(idZ >= inGridParam->ctGridPoints[2]))
		return;
	U32 idxVertex = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + idX;
	if(idxVertex >= inGridParam->ctTotalPoints)
		return;
	
	//IF This vertex has no hot edges then return
	U32 ctVertices = arrInHighEdgesCount[idxVertex]; 
	if(ctVertices == 0)
		return;

	float4 va = arrInFields[idxVertex];
	float4 vb;

	U8 flag = arrInHighEdgesFlags[idxVertex];
	U32 idxVertexNbor = 0;
	U32 idxStore = arrInHighEdgesOffset[idxVertex];
	
	//XYZ: 421
	//Compute Left Right Edge
	if(flag & 0x04)
	{
		idxVertexNbor = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + (idX+1);
		vb = arrInFields[idxVertexNbor];

		//Compute Root vertex and normal
#ifdef USE_NEWTONRAPHSON				
		float4 v = ComputeRootNewtonRaphson(va, vb, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		float4 n = ComputeGradientWithPrevField(v, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		n = normalize(-1 * n);
#else
		float field;
		float4 v = va + ((ISO_VALUE - va.w)/(vb.w - va.w)) * (vb - va);
		float3 n = ComputeGradientAndField(v, &field, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		n = normalize(-1 * n);
#endif
	
		//Compute Field and Color
		float4 color;
		ComputeFieldAndColor(v, &color, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);

		//Save 
		v.w = 1;
		arrOutMeshVertex[idxStore] = v;
		arrOutMeshColor[idxStore] = color;
		arrOutMeshNormal[idxStore * 3] = n.x;
		arrOutMeshNormal[idxStore * 3 + 1] = n.y;
		arrOutMeshNormal[idxStore * 3 + 2] = n.z;

		idxStore++;
	}

	//Compute Bottom Top Edge
	if(flag & 0x02)
	{
		idxVertexNbor = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + (idY+1) * inGridParam->ctGridPoints[0] + idX;
		vb = arrInFields[idxVertexNbor];

		//Compute Root vertex and normal
#ifdef USE_NEWTONRAPHSON		
		float4 v = ComputeRootNewtonRaphson(va, vb, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		float4 n = ComputeGradientWithPrevField(v, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		n = normalize(-1 * n);
#else
		float field;
		float4 v = va + ((ISO_VALUE - va.w)/(vb.w - va.w)) * (vb - va);
		float3 n = ComputeGradientAndField(v, &field, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		n = normalize(-1 * n);
#endif

		//Compute Field and Color
		float4 color;
		ComputeFieldAndColor(v, &color, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);

		//Save 
		v.w = 1;
		arrOutMeshVertex[idxStore] = v;
		arrOutMeshColor[idxStore] = color;
		arrOutMeshNormal[idxStore * 3] = n.x;
		arrOutMeshNormal[idxStore * 3 + 1] = n.y;
		arrOutMeshNormal[idxStore * 3 + 2] = n.z;

		idxStore++;
	}

	//Compute Near Far Edge
	if(flag & 0x01)
	{
		idxVertexNbor = (idZ+1) * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + idX;
		vb = arrInFields[idxVertexNbor];

		//Compute Root vertex and normal
#ifdef USE_NEWTONRAPHSON		
		float4 v = ComputeRootNewtonRaphson(va, vb, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		float4 n = ComputeGradientWithPrevField(v, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		n = normalize(-1 * n);
#else
		float field;
		float4 v = va + ((ISO_VALUE - va.w)/(vb.w - va.w)) * (vb - va);
		float3 n = ComputeGradientAndField(v, &field, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);
		n = normalize(-1 * n);
#endif

		//Compute Field and Color
		float4 color;
		ComputeFieldAndColor(v, &color, arrInHeader4, arrInOps4, arrInPrims4, arrInMtxNodes4);

		//Save 
		v.w = 1;
		arrOutMeshVertex[idxStore] = v;
		arrOutMeshColor[idxStore] = color;
		arrOutMeshNormal[idxStore * 3] = n.x;
		arrOutMeshNormal[idxStore * 3 + 1] = n.y;
		arrOutMeshNormal[idxStore * 3 + 2] = n.z;
		idxStore++;
	}
} 

//Compute Cell Configs
__kernel void ComputeCellConfigs(__global float4* arrInFields,
								 __read_only image2d_t texInVertexCountTable,
								 __constant struct GridParam* inGridParam,
								 __constant struct CellParam* inCellParam,
								 __global U32* arrOutCellElementsCount,
								 __global U8* arrOutCellConfig)
{
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inCellParam->ctNeededCells[0])||(idY >= inCellParam->ctNeededCells[1])||(idZ >= inCellParam->ctNeededCells[2]))
		return;
	U32 idxCell = idZ * (inCellParam->ctNeededCells[0] * inCellParam->ctNeededCells[1]) + idY * inCellParam->ctNeededCells[0] + idX;
	if(idxCell >= inCellParam->ctTotalCells)
		return;

	U32 dx = inGridParam->ctGridPoints[0];	
	U32 dxdy = inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1];
	
	//Compute Configuration index
    U32 idxCorners[8];
    idxCorners[0] = idZ * dxdy + idY * dx + idX;
    idxCorners[1] = (idZ+1) * dxdy + idY * dx + idX;
    idxCorners[2] = idZ * dxdy + (idY+1) * dx + idX;
    idxCorners[3] = (idZ+1) * dxdy + (idY+1) * dx + idX;
    idxCorners[4] = idZ * dxdy + idY * dx + idX + 1;
    idxCorners[5] = (idZ+1) * dxdy + idY * dx + idX + 1;
    idxCorners[6] = idZ * dxdy + (idY+1) * dx + idX + 1;
    idxCorners[7] = (idZ+1) * dxdy + (idY+1) * dx + idX + 1;
    
    int idxConfig = 0;    
    for(int i=0; i<8; i++)
	{
    	if(idxCorners[i] < inGridParam->ctTotalPoints)
    	{
    	   if(isgreaterequal(arrInFields[idxCorners[i]].w, ISO_VALUE))
    		   idxConfig += (1 << i);
    	}
	}
	
    //read number of vertices that output from this cell
	arrOutCellElementsCount[idxCell] = read_imageui(texInVertexCountTable, tableSampler, (int2)(idxConfig,0)).x;
	arrOutCellConfig[idxCell] = idxConfig;
}

//Compute Elements Kernel
__kernel void ComputeElements(__global U32* arrInHighEdgesOffset,
		   	   	   	   	   	  __global U8* arrInHighEdgesFlags,
		   	   	   	   	   	  __global U8* arrInCellConfig,
							  __global U32* arrInCellElementCount,
							  __global U32* arrInCellElementOffset,
							  __read_only image2d_t texInTriangleTable,
							  __constant struct GridParam* inGridParam,
							  __constant struct CellParam* inCellParam,							  
							  __global U32* arrOutElements)
{
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inCellParam->ctNeededCells[0])||(idY >= inCellParam->ctNeededCells[1])||(idZ >= inCellParam->ctNeededCells[2]))
		return;
	uint idxCell = idZ * (inCellParam->ctNeededCells[0] * inCellParam->ctNeededCells[1]) + idY * inCellParam->ctNeededCells[0] + idX;
	if(idxCell >= inCellParam->ctTotalCells)
		return;
	int ctElements = arrInCellElementCount[idxCell];
	if(ctElements == 0)
		return;
	
	U32 dx = inGridParam->ctGridPoints[0];	
	U32 dxdy = inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1];

	//Compute Configuration index
    U32 idxCorners[8];
    idxCorners[0] = idZ * dxdy + idY * dx + idX;
    idxCorners[1] = (idZ+1) * dxdy + idY * dx + idX;
    idxCorners[2] = idZ * dxdy + (idY+1) * dx + idX;
    idxCorners[3] = (idZ+1) * dxdy + (idY+1) * dx + idX;
    idxCorners[4] = idZ * dxdy + idY * dx + idX + 1;
    idxCorners[5] = (idZ+1) * dxdy + idY * dx + idX + 1;
    idxCorners[6] = idZ * dxdy + (idY+1) * dx + idX + 1;
    idxCorners[7] = (idZ+1) * dxdy + (idY+1) * dx + idX + 1;
	
	U32 idxEdge;
	int idxEdgeStart, idxEdgeAxis;
	U32 idxStore = arrInCellElementOffset[idxCell];
	
	//Break loop if the idxEdge is 255
	for(int i=0; i<ctElements; i++)
	{
		idxEdge = read_imageui(texInTriangleTable, tableSampler, (int2)(i, arrInCellConfig[idxCell])).x;
		if(idxEdge == 255)
			break;
		idxEdgeStart = inCellParam->corner1[idxEdge];
		//idxEdgeEnd   = inCellParam->corner2[idxEdge];
		idxEdgeAxis  = inCellParam->edgeaxis[idxEdge];	

		U8 flag = arrInHighEdgesFlags[idxCorners[idxEdgeStart]];		
		U8 incr[3];
		//XYZ: 421
		incr[0] = 0;
		incr[1] = (flag & 0x04) >> 2;
		incr[2] = incr[1] + ((flag & 0x02) >> 1);			
		
		//XYZ = 012
		arrOutElements[idxStore + i] = arrInHighEdgesOffset[idxCorners[idxEdgeStart]] + incr[idxEdgeAxis]; 
	}
}


/*!
 * Stackless Non-Recursive BlobTree Traversal
 */
float ComputeFieldStackBased(float4 v,			
						   __global float4* arrHeader4,
						   __global float4* arrOps4,
						   __global float4* arrPrims4, 
						   __global float4* arrMtxNodes4)
{
	//Index Stack
	StackU32 stk;
	stk.count = 0;

	//Field Stack
	StackU32 stkFieldOp;
	stkFieldOp.count = 0;
	
	StackF stkField;
	stkField.count = 0;

	//Count : Prims, Ops, Mtx ; CellSize
	//U32 ctPrims = (U32)arrHeader4[OFFSET4_HEADER_PARAMS].x;
	U32 ctOps   = (U32)arrHeader4[OFFSET4_HEADER_PARAMS].y;
	if(ctOps == 0) 
		return ComputePrimitiveField(v, 0, arrOps4, arrPrims4, arrMtxNodes4);
	
	
	//Push the first operator onto stack
	StackPush(&stk, 0);
	while(!IsStackEmpty(&stk)) {
		U32 idxOp = StackTop(&stk);
		StackPop(&stk);
		
		U32 opType = (U32)arrOps4[idxOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].x;
		U32 opLeftChild = (U32)arrOps4[idxOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].y;
		U32 opRightChild = (U32)arrOps4[idxOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_TYPE].z;	
		float4 params = arrOps4[idxOp * DATASIZE_OPERATOR_F4 + OFFSET4_OP_RES];
		U32 flags = (U32)params.w;
		
		//enum OPFLAGS {ofRightChildIsOp = 1, ofLeftChildIsOp = 2, ofChildIndexIsRange = 4, ofIsUnaryOp = 8, ofIsRightOp = 16, ofBreak = 32};
		//bool isBreak = (flags & ofBreak) >> 5;
		//bool bUnary  = (flags & ofIsUnaryOp) >> 3;
		bool bRange  = (flags & ofChildIndexIsRange) >> 2;
		bool bLeftChildOp  = (flags & ofLeftChildIsOp) >> 1;
		bool bRightChildOp = (flags & ofRightChildIsOp);
		
		if(bRange) {
			StackPushF(&stkField, ComputeRangeField(v, idxOp, arrOps4, arrPrims4, arrMtxNodes4));
			continue;
		}
		
		bool isLFReady = !bLeftChildOp;
		if(!IsStackEmpty(&stkFieldOp)) {
			isLFReady |= (StackTop(&stkFieldOp) == opLeftChild);
		}

		bool isRFReady = !bRightChildOp;
		if(!IsStackEmpty(&stkFieldOp)) {
			isRFReady |= (StackTop(&stkFieldOp) == opRightChild);
		}

		
		if(isLFReady && isRFReady) {
			float lf = 0.0f;
			float rf = 0.0f;
			
			//Left Node
			if(bLeftChildOp) {
				lf = StackTopF(&stkField);
				StackPopF(&stkField);
			}
			else
				lf = ComputePrimitiveField(v, opLeftChild, arrOps4, arrPrims4, arrMtxNodes4);

			
			//Right Node
			if(bRightChildOp) {
				rf = StackTopF(&stkField);
				StackPopF(&stkField);
			}
			else
				rf = ComputePrimitiveField(v, opRightChild, arrOps4, arrPrims4, arrMtxNodes4);

			//Push Field
			StackPushF(&stkField, ComputeOpField(opType, lf, rf, params));
		}
		else {		
			if(bLeftChildOp && !isLFReady)
				StackPush(&stk, opLeftChild);
			if(bRightChildOp && !isRFReady)
				StackPush(&stk, opRightChild);
		}
	}
	
	if(IsStackEmptyF(&stkField))
		return 0.0f;
	else
		return StackTopF(&stkField);
}









