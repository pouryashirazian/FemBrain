/*
 * Cutting.h
 *
 *  Created on: Sep 25, 2013
 *      Author: pourya
 */

#ifndef CUTTING_H_
#define CUTTING_H_

#include <vector>
#include "base/Vec.h"
#include "graphics/SGMesh.h"
#include "graphics/CLManager.h"
#include "graphics/OclSumScan.h"
#include "deformable/Deformable.h"

#include "tetMesh.h"

using namespace PS;
using namespace PS::SG;
using namespace PS::FEM;
using namespace PS::MATH;
using namespace PS::CL;

namespace PS {
namespace FEM {


/*!
 * Cutting and Subdivision of tetrahedral meshes requires several steps that are compute intensive.
 */
class Cutting : public SGMesh {
public:
	Cutting(Deformable* lpDeformableModel);
	virtual ~Cutting();

	void setup();
	void cleanup();
	void cleanupMemBuffers();

	bool createMemBuffers();

	int performCut(const vec3d& edge0, const vec3d& edge1);
	int performCut(const vec3d& edge0, const vec3d& edge1, vec3d sweptQuad[4]);

	int computeFaceIntersections(const vec3d& s0, const vec3d& s1);
	int computeEdgeIntersections(const vec3d sweptQuad[4]);
	int computeFaceCentroids();
	int computeFaceSegmentIntersectionTest();


	/*!
	 * Drawing debug information such as: Swept path, swept Quad
	 */
	void draw();
private:

	struct MeshInfo  {
		U32 ctVertices;
		U32 ctTets;
	};

	//Mesh
	vector<U32> m_vMeshTets;
	vector<double> m_vMeshVertices;

	//Draw FacePoints
	GLMemoryBuffer* m_lpDrawFacePoints;
	GLMemoryBuffer* m_lpDrawEdgePoints;

	//Stats
	U32 m_ctFacePoints;
	U32 m_ctEdgePoints;

	//Deformable
	SumScan* m_lpOclSumScan;
	Deformable* m_lpDeformable;

	//MeshInfo
	MeshInfo m_meshInfo;
	bool m_isMemBuffersLoaded;


	//Cutting
	bool m_isSweptQuadValid;
	vec3d m_sweptQuad[4];
	vector<vec3d> m_vCuttingPathEdge0;
	vector<vec3d> m_vCuttingPathEdge1;


	//CL
	ComputeDevice* m_lpDevice;
	ComputeKernel* m_lpKernelFacePoints;
	ComputeKernel* m_lpKernelEdgePoints;
	ComputeKernel* m_lpKernelCentroids;
	ComputeKernel* m_lpKernelTest;

	cl_mem m_inMemTetMeshVertices;
	cl_mem m_inMemTetMeshIndices;
	cl_mem m_inMemMeshInfo;

	//Face
	cl_mem m_inoutMemFaceFlags;
	cl_mem m_inoutMemFacePoints;

	//Edge
	cl_mem m_inoutMemEdgeFlags;
	cl_mem m_inoutMemEdgePoints;



};



}
}


#endif /* CUTTING_H_ */
