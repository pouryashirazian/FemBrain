//Constants
#define ISO_VALUE 0.5f
#define NORMAL_DELTA 0.0001f
#define FIELD_VALUE_EPSILON 0.001f

#define ZERO4 {0.0f, 0.0f, 0.0f, 0.0f}
#define AXISX {1.0f, 0.0f, 0.0f, 0.0f}
#define AXISY {0.0f, 1.0f, 0.0f, 0.0f}
#define AXISZ {0.0f, 0.0f, 1.0f, 0.0f}

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


//Applies displacements to the mesh buffer
__kernel void ApplyVertexDeformations(U32 ctVertices,
									  __global float4* arrInRestPos,
									  __global float4* arrInDeformation,
									  __global float4* arrOutMeshVertex) {
	int idX = get_global_id(0);
	if(idX >= ctVertices)
		return;
	
	arrOutMeshVertex[idX] = arrInRestPos[idX] + arrInDeformation[idX];
}

//Compute Fields for radial basis functions
__kernel void ComputeRBFPrimFieldArray(U32 ctCenters, 
									  __global float4* arrInterpolationNodesLambda,
									  U32 ctVertices,
									  __global float4* arrInOutVertexFields) {
	int idX = get_global_id(0);
	if(idX >= ctVertices)
		return;

	float field = 0.0f;
	for(int i=0; i < ctCenters; i++)
		field += arrInterpolationNodesLambda[i].w * distance(arrInterpolationNodesLambda[i].xyz, arrInOutVertexFields[idX].xyz);
	
	arrInOutVertexFields[idX].w = field;
}