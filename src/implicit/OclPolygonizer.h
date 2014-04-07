/*
 * PS_OclPolygonizer.h
 *
 *  Created on: 2011-12-04
 *      Author: pourya
 */
#ifndef PS_OCLPOLYGONIZER_H
#define PS_OCLPOLYGONIZER_H


#include "LinearBlobTree.h"
#include "base/Vec.h"
#include "graphics/ComputeDevice.h"
#include "graphics/SGMesh.h"
#include "graphics/OclScan.h"
#include "graphics/OclSumScan.h"
#include "graphics/GLTexture.h"

using namespace PS::SIMDPOLY;
using namespace PS::CL;
using namespace PS::GL;
using namespace PS::MATH;
using namespace PS::SKETCH;

//Error codes
#define ERR_GPUPOLY_KERNEL_NOT_BUILT -1
#define ERR_GPUPOLY_BUFFER_NOT_WRITTEN -2
#define ERR_GPUPOLY_BUFFER_NOT_READ -3
#define ERR_GPUPOLY_TRITABLE_NOT_READ -4
#define ERR_GPUPOLY_VERTEXTABLE_NOT_READ -5
#define ERR_GPUPOLY_FIELDSTABLE		-6
#define ERR_NO_CROSSED_CELLS 		-7
#define ERR_INVALID_INPUT_PARAM 	-8


#define MAX_VERTICES_COUNT_PER_CELL		15
#define MAX_TRIANGLES_COUNT_PER_CELL	5

namespace PS {
namespace SKETCH {

/*!
 * GPU polygonizer for iso-surface extraction of the animated BlobTree on the GPU
 */
class GPUPoly : public SG::SGMesh
{
public:
	GPUPoly();
	explicit GPUPoly(const LinearBlobTree& blob);
	virtual ~GPUPoly();

	//Produce vertices count from the table
	static void ProduceNumVerticesTable(const char* chrOutput);
	static void DrawBox(const vec3f& lo, const vec3f& hi, const vec3f& color, float lineWidth);

	void testScan();

	//Multi-Pass polygonization
	int run();

	//Run tetrahedralization process
	int runTetrahedralizer();

	//Buffer to draw normals
	GLMeshBuffer* prepareMeshBufferForDrawingNormals(float len = 0.3f);

	//Draws the mesh normals
	void drawNormals();

	//Read-back Mesh
	bool readbackMeshV3T3(U32& ctVertices, vector<float>& vertices,
						U32& ctTriangles, vector<U32>& elements) const;

	//Reads normals
	bool readBackNormals(U32& ctVertices, vector<float>& verticesXYZ, vector<float>& normals) const;

	//Read voxel grid vertex fields
	bool readBackVoxelGridSamples(vec4u& dim, vector<float>& arrXYZF) const;

	/*
	 * Computes fieldvalues for an array of points. The fourth component will be assigned
	 * the field associated with the volume model
	 */
	int computeFieldArray(U32 ctVertices, U32 step, vector<float>& vertices);
    int computeFieldImage(const vec3f& lo, const vec3f& hi, GLTexture* lpOutTex);
	int computeOffSurfacePointsAndFields(U32 interval, float len, U32& ctOutVertices, vector<float>& outOffSurfacePoints);

	/*!
	 * Applies computed displacements from FEM Integrator on the polygonized mesh vertices
	 * @dof degrees of freedom in displacements
	 * @displacement vertex displacements
	 * @return true if displacements applied successfully.
	 */
	bool applyFemDisplacements(U32 dof, double* displacements);


	//Set BlobTree
	bool setBlob(const LinearBlobTree& blob);

	//Cellsize
	void setCellSize(float cellsize) { m_cellsize = cellsize;}
	float cellsize() const {return m_cellsize;}

	ComputeDevice* computeDevice() const {return m_lpGPU;}
	vec3i voxelGridDim() const;
	U32 countSurfaceVoxels() const {return m_voxels.size();}
	const vector<vec3f>& surfaceVoxels() const {return m_voxels;}

private:
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

	void clearBlobBuffer();
private:
	vector<vec3f> m_voxels;

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
	ComputeKernel* m_lpKernelComputeFieldArray;
    ComputeKernel* m_lpKernelComputeFieldImage;
	ComputeKernel* m_lpKernelComputeOffSurfacePointsAndFields;


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
	LinearBlobTree m_blob;
	float m_cellsize;
};



int Run_SphereDistKernel();
int Run_ArrayTestKernel();

}
}




#endif // PS_OCLPOLYGONIZER_H
