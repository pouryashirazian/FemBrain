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
#define DATASIZE_HEADER			12
#define DATASIZE_OPERATOR		8
#define DATASIZE_PRIMITIVE		20

//Operator
#define OFFSET_OP_TYPE			0
#define OFFSET_OP_CHILDREN 		1
#define OFFSET_OP_LINK_FLAGS 	2
#define OFFSET_OP_PARENT_LINK	3

//Primitive
#define OFFSET_PRIM_TYPE		0
#define OFFSET_PRIM_IDX_MATRIX 	1
#define OFFSET_PRIM_LINK_FLAGS 	2
#define OFFSET_PRIM_PARENT_LINK 3

//HEADER
#define OFFSET_COUNT_PRIMS 		8
#define OFFSET_COUNT_OPS 		9
#define OFFSET_COUNT_MTXNODES	10
#define OFFSET_COUNT_INSTANCE 	11

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
	/*!
	 * Run the algorithm in tandem
	 */
	int runTandem(float cellsize = DEFAULT_CELL_SIZE);

	//Multi-Pass polygonization
	int runMultiPass(float cellsize = DEFAULT_CELL_SIZE);

	//Draws the mesh using accelerated memory buffer objects
	void drawBBox();

private:
	/*!
	 * Init shaders and opencl kernels
	 */
	int init();


	//Polygonizer Passes
	int computeAllFields(float cellsize);
	int computeEdgeTable();
	int computeVertexAttribs(U32 ctVertices);
	int computeCellConfigs();
	int computeElements(U32 ctElements);

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
