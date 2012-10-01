/*
 * PS_Deformable.h
 *
 *  Created on: Sep 29, 2012
 *      Author: pourya
 */

#ifndef PS_DEFORMABLE_H_
#define PS_DEFORMABLE_H_

#include <vector>
#include "../PS_Base/PS_MathBase.h"
#include "../PS_Graphics/PS_MeshGLBuffer.h"

#include "corotationalLinearFEM.h"
#include "corotationalLinearFEMForceModel.h"
#include "generateMassMatrix.h"
#include "implicitBackwardEulerSparse.h"
#include "sceneObjectDeformable.h"

using namespace std;

/*!
 *	Deformable model
 */
class Deformable{
public:
	Deformable();
	Deformable(const char* lpVegFilePath, const char* lpObjFilePath);
	virtual ~Deformable();

	void setup(const char* lpVegFilePath, const char* lpObjFilePath);
	void cleanup();

	void draw();
	void timestep();

	//Toggles a vertex state
	void toggleVertex(double worldX, double worldY, double worldZ);

private:
	void computeConstrainedDof(std::vector<int>& vArrOutputDof);
private:
	GLMeshBuffer m_meshBuffer;

	//Tetrahedra input mesh
	TetMesh* m_lpTetMesh;

	//Deformable Model
	CorotationalLinearFEM* m_lpDeformable;

	//ForceModel
	ForceModel* m_lpDeformableForceModel;

	//Integrator
	ImplicitBackwardEulerSparse* m_lpIntegrator;

	//
	SceneObjectDeformable * m_lpDeformableMesh;

	//Mass Matrix
	SparseMatrix* m_lpMassMatrix;

	//Time Step Counter
	U32 m_ctTimeStep;

	U32 m_dof;
	double* m_arrDisplacements;
	double* m_arrExtForces;

	//Fixed Vertices
	vector<int> m_vFixedVertices;
	int m_idxPulledVertex;

	bool m_bRenderFixedVertices;

};

#endif /* PS_DEFORMABLE_H_ */
