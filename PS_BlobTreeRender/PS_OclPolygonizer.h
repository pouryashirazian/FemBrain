/*
 * PS_OclPolygonizer.h
 *
 *  Created on: 2011-12-04
 *      Author: pourya
 */
#ifndef PS_OCLPOLYGONIZER_H
#define PS_OCLPOLYGONIZER_H

#include "PS_PolyMemManager.h"
#include "../PS_Graphics/PS_ComputeDevice.h"
#include "../PS_Graphics/PS_VectorMath.h"
#include "../PS_Graphics/PS_GLMeshBuffer.h"
#include "../PS_Graphics/PS_OclScan.h"
#include "../PS_Graphics/PS_OclSumScan.h"

using namespace PS::SIMDPOLY;
using namespace PS::HPC;
using namespace PS::FUNCTIONALMATH;

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
#define OFFSET_OP_RES_X			4
#define OFFSET_OP_RES_Y			5
#define OFFSET_OP_RES_Z			6
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

//Defines an empty index to jump out of branch
#define NULL_BLOB 0xFFFF


namespace PS{
namespace HPC{

/*!
 * GPU polygonizer for iso-surface extraction of the animated BlobTree on the GPU
 */
class GPUPoly : public GLMeshBuffer
{
public:
	GPUPoly();
	explicit GPUPoly(const char* lpFilePath);
	virtual ~GPUPoly();

	//Produce vertices count from the table
	static void ProduceNumVerticesTable(const char* chrOutput);
	static void DrawBox(const svec3f& lo, const svec3f& hi, const svec3f& color, float lineWidth);

	/*!
	 * Read a model from disk and converts it into a proper format for GPU
	 * @param lpFilePath the model file path
	 */
	bool readModel(const char* lpFilePath);


	void testScan();


	//Multi-Pass polygonization
	int runMultiPass(float cellsize = DEFAULT_CELL_SIZE, bool outputTetMesh = true);

	//Draws the mesh using accelerated memory buffer objects
	void drawBBox();

	//Read-back Mesh
	bool readBackMesh(U32& ctVertices, vector<float>& vertices,
						U32& ctFaceElements, vector<U32>& elements);

	/*!
	 * Applies computed displacements from FEM Integrator on the polygonized mesh vertices
	 * @dof degrees of freedom in displacements
	 * @displacement vertex displacements
	 * @return true if displacements applied successfully.
	 */
	bool applyFemDisplacements(U32 dof, double* displacements);
private:
	struct NODE {
		U32 index;
		U32 depth;
		U8 isOp;

		NODE() :
				index(0), depth(0), isOp(1) {
		}
		NODE(U32 index_, U32 depth_, U8 isOp_) {
			index = index_;
			depth = depth_;
			isOp = isOp_;
		}
	};
	/*!
	 * Init shaders and opencl kernels
	 */
	int init();

	/*!
	 * Run the algorithm in tandem
	 */
	int runTandem(float cellsize = DEFAULT_CELL_SIZE);

	//Polygonizer Passes
	int computeAllFields(float cellsize);
	int computeEdgeTable();
	int computeVertexAttribs(U32 ctVertices);
	int computeCellConfigs();
	int computeElements(U32 ctElements);

	//Tetrahedralizer Passes for FEM
	int computeTetMeshCellsInsideOrCrossed();
	int computeTetMeshVertices(U32 ctVertices);
	int computeTetMeshElements(U32 ctElements);


	bool storeTetMeshInVegaFormat(const char* chrFilePath);

	//Set traversal route
	bool setTraversalRoute();
	void printBlobTree(const char* chrFilePath) const;

private:

    struct CellParam{
		U8 corner1[12];
		U8 corner2[12];
		U8 edgeaxis[12];
		U32 ctNeededCells[3];
		U32 ctTotalCells;
		float cellsize;
	};

    //Grid Points
    struct GridParam{
    	U32 ctGridPoints[3];
    	U32 ctTotalPoints;
    	float cellsize;
    };

    //Model
    string m_strModelFilePath;

    //Grid and Cell
	CellParam m_cellParam;
	GridParam m_gridParam;

	//SumScan
    SumScan* m_lpOclSumScan;

	//GPU Device
	ComputeDevice* m_lpGPU;

	//Compute Kernels
	ComputeKernel* m_lpKernelCellConfig;
	ComputeKernel* m_lpKernelComputeConfig;
	ComputeKernel* m_lpKernelComputeMesh;


	//Kernels for the multipass polygonizer
	ComputeKernel* m_lpKernelComputeAllFields;
	ComputeKernel* m_lpKernelComputeEdgeTable;
	ComputeKernel* m_lpKernelComputeVertexAttribs;
	ComputeKernel* m_lpKernelComputeCellConfigs;
	ComputeKernel* m_lpKernelComputeElements;

	//Kernels for Tetrahedralization
	ComputeKernel* m_lpKernelTetMeshEdgeTable;
	ComputeKernel* m_lpKernelTetMeshVertices;
	ComputeKernel* m_lpKernelTetMeshCountCells;
	ComputeKernel* m_lpKernelTetMeshElements;

	//Finite Element
	ComputeKernel* m_lpKernelApplyDeformations;


	//Reusable vars
	cl_mem m_inMemVertexCountTable;
	cl_mem m_inMemTriangleTable;

	cl_mem m_inMemHeader;
	cl_mem m_inMemOps;
	cl_mem m_inMemPrims;
	cl_mem m_inMemMtx;
	cl_mem m_inMemCellParam;
	cl_mem m_inMemGridParam;

	//Vertex
	cl_mem m_inoutMemAllFields;
	cl_mem m_inoutMemHighEdgesCount;
	cl_mem m_inoutMemHighEdgesFlags;
	cl_mem m_inMemHighEdgesOffset;

	//Cell
	cl_mem m_inoutMemCellConfig;
	cl_mem m_inoutMemCellElementsCount;
	cl_mem m_inMemCellElementsOffset;

	//FEM
	cl_mem m_inoutMemRestPos;


	//TetMesh
	U32 m_ctTetMeshVertices;
	U32 m_ctTetMeshElements;
	cl_mem m_outMemTetMeshVertices;
	cl_mem m_outMemTetMeshElements;


	//Model
	bool m_bModelLoaded;

	//Inputs:
	//BlobNode Matrix
	SOABlobNodeMatrices m_mtxNode;
	SOABlobBoxMatrices m_mtxBox;

	//Data Header
	float m_arrHeader[DATASIZE_HEADER];

	//Primitives Data
	float PS_SIMD_ALIGN(m_arrPrims[PS_SIMD_PADSIZE(MAX_TREE_NODES*DATASIZE_PRIMITIVE)]);

	//Operators Data
	float PS_SIMD_ALIGN(m_arrOps[PS_SIMD_PADSIZE(MAX_TREE_NODES*DATASIZE_OPERATOR)]);

	//AABB
	svec3f m_bboxLo;
	svec3f m_bboxHi;

	//Count of primitives and ops
	U32 m_ctPrims;
	U32 m_ctOps;
};



int Run_SphereDistKernel();
int Run_ArrayTestKernel();

}
}




#endif // PS_OCLPOLYGONIZER_H
