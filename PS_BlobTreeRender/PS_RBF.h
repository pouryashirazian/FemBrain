/*
 * PS_RBF.h
 *
 *  Created on: Apr 2, 2013
 *      Author: pourya
 */

#ifndef PS_RBF_H_
#define PS_RBF_H_

#include "../PS_Graphics/PS_ComputeDevice.h"
#include "../PS_Graphics/PS_Vector.h"
#include "PS_Graphics/PS_Mesh.h"
#include "PS_OclPolygonizer.h"

using namespace PS;
using namespace PS::HPC;
using namespace PS::MATH;
using namespace PS::MESH;

#define DEFAULT_OFFSURFACE_LEN 0.05
#define DEFAULT_START_CENTERS_COUNT 1000
#define DEFAULT_GREEDY_STEP 50
#define DEFAULT_FITTING_ACCURACY 0.01f
#define MAX_GREEDY_ATTEMPTS 1

/*
 * A BlobTree model is defined using implicit primitives and operators.
 * The model is polygonized with our high performance GPU polygonizer and
 * the resulting polygon mesh represents the model's boundary surface or
 * iso-surface.
 * For our physically-based animation system volumetric model has to be
 * descretized to small tetrahedral elements which uses the brep mesh vertices
 * but will possibly add more surface and internal vertices to produce tetrahedral
 * connnectivity.
 *
 * When animation starts the tetrahedral elements may undergo deformations which will
 * displace their associated vertices and deform the original brep mesh. To keep an uptodate
 * implicit definition of the model we need to approximate the implicit function which defines
 * the model. Radial-basis functions will help us to compute a fast, accurate implicit definition.
 *
 * Benefits of implicit definition:
 * 1. To compute fast collision detection between Avatar and the model
 * 2. To compute the haptic feedback force in realtime.
 */

namespace PS {
namespace HPC {


class FastRBF : public GLMeshBuffer {
public:
	FastRBF();
	explicit FastRBF(GPUPoly* lpGPUPoly);
	explicit FastRBF(Mesh* lpMesh);
	virtual ~FastRBF();

	int init();

	//Polygonize to visualize the RBF
	int polygonize(float cellsize = DEFAULT_CELL_SIZE);

	//Intersection functions
	bool intersects(const vec3f& v, float& penetration) const;
	int intersects(const vector<vec3f>& vertices,
					  vector<bool>& collisions,
					  vector<float>& penetrations,
					  int& idxMaxPenetrated);

	//bool intersects(const vector<vec3f>& v, const vector<bool>& crossed) const;
	bool intersects(const AABB& box);
	void resetCollision();


	//Shuffle interpolation nodes
	bool randomizeInterpolationNodes();

	//Setup
	bool setupFromImplicit(GPUPoly* lpGPUPoly);
	bool setupFromMesh(Mesh* lpMesh);
	bool computeInterpolationFunction(U32 start_count,
										  U32 iAttempt = 1,
										  U32 step = DEFAULT_GREEDY_STEP,
										  float fitting_accuracy = DEFAULT_FITTING_ACCURACY);

	float computeMaxResidualError() const;
	bool testWithVoxelGridSamples(GPUPoly* lpGPUPoly);


	float fieldRBF(const vec3f& v, float iso = 0.0f) const;
	vec3f gradientRBF(const vec3f& v) const;

	float getOffSurfaceLen() const {return m_offSurfaceLen;}
	void setOffSurfaceLen(float len) { m_offSurfaceLen = len;}

	/*!
	 * Displaces vertices based on computed deformations
	 */
	bool applyFemDisplacements(U32 dof, double* displacements);
	bool computeFieldArray();

	//Create Normals
	bool readbackMeshV3T3(U32& ctVertices, vector<float>& vertices, U32& ctElements, vector<U32>& elements);
	GLMeshBuffer* prepareMeshBufferNormals();


	void drawCollision() const;
private:
	bool copyVertexBufferToRestPos();

private:
	vector<vec3f> m_collision;
	vector<float> m_penetration;

	U32 m_ctCenters;
	U32 m_ctTotalInterpolationNodes;
	vector<float> m_interpolationNodes;
	vector<float> m_fields;
	vector<float> m_lambda;
	float m_offSurfaceLen;
	double m_solutionError;

	//SumScan
    SumScan* m_lpOclSumScan;
	ComputeDevice* m_lpGPU;
	ComputeKernel* m_lpKernelApplyDeformations;
	ComputeKernel* m_lpKernelComputeFieldArray;

	//Reusable vars
	cl_mem m_inMemVertexCountTable;
	cl_mem m_inMemTriangleTable;

	//FEM
	cl_mem m_inoutMemRestPos;

};

}
}
#endif /* PS_RBF_H_ */
