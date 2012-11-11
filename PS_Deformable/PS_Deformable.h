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
#include "../PS_Graphics/PS_GLMeshBuffer.h"
#include "../PS_Graphics/PS_Vector.h"
#include "../PS_Graphics/PS_Box.h"

#include "corotationalLinearFEM.h"
#include "corotationalLinearFEMForceModel.h"
#include "generateMassMatrix.h"
#include "implicitBackwardEulerSparse.h"
#include "sceneObjectDeformable.h"
#include "graph.h"

using namespace std;
using namespace PS::MATH;

/*!
 *	Deformable model
 */
class Deformable{
public:
	Deformable();
	Deformable(const char* lpVegFilePath,
				const char* lpObjFilePath,
				std::vector<int>& vFixedVertices);

	virtual ~Deformable();


	void draw();
	void timestep();

	//Pick a vertex
	int pickVertex(const vec3d& wpos, vec3d& vertex);
	int pickVertices(const vec3d& boxLo, const vec3d& boxHi,
					   vector<vec3d>& arrFoundCoords, vector<int>& arrFoundIndices) const;
	int pickVertices(const vec3d& boxLo, const vec3d& boxHi);

	//Haptic Interaction
	void setPulledVertex(int index) { m_idxPulledVertex = index;}
	bool hapticStart(int index);
	bool hapticStart(const vec3d& wpos);
	void hapticEnd();
	bool hapticUpdateForce();
	bool hapticUpdateDisplace();
	void hapticSetCurrentForce(const vec3d& extForce);
	void hapticSetCurrentDisplacement(const vec3d& displacement);
	void hapticSetCurrentDisplacements(const vector<int>& indices,
											const vector<vec3d>& displacements);
	bool isHapticInProgress() const {return m_bHapticInProgress;}
	int getHapticForceRadius() const {return m_hapticForceNeighorhoodSize;}
	void setHapticForceRadius(int radius) { m_hapticForceNeighorhoodSize = radius;}


	//Render
	bool renderVertices() const {return m_bRenderVertices;}
	void setRenderVertices(bool bRender) { m_bRenderVertices = bRender;}

	//Damping stiffness
	void setDampingStiffnessCoeff(double s);
	double getDampingStiffnessCoeff() const {return m_dampingStiffnessCoeff;}

	//Damping Mass
	void setDampingMassCoeff(double m);
	double getDampingMassCoeff() const {return m_dampingMassCoeff;}

	bool addFixedVertex(int index);
	bool removeFixedVertex(int index);

	/*!
	 * Return: Outputs number of dofs
	 */
	static int FixedVerticesToFixedDOF(std::vector<int>& arrInFixedVertices,
										    std::vector<int>& arrOutFixedDOF);
private:
	/*!
	 * Setup procedure builds deformable model with the specified params
	 */
	void setup(const char* lpVegFilePath,
			    const char* lpObjFilePath,
			    std::vector<int>& vFixedVertices);

	void cleanup();

private:
	bool m_bHapticInProgress;
	double m_hapticCompliance;
	double m_hapticExtForce[3];
	double m_hapticExtDisplacements[3];
	int m_hapticForceNeighorhoodSize;

	double m_dampingStiffnessCoeff;
	double m_dampingMassCoeff;
	double m_timeStep;

	Graph* m_lpMeshGraph;
	//double* m_lpHapticForces;
	//GLMeshBuffer m_meshBuffer;

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
	vector<int> m_vFixedDofs;

	vector<vec3d> m_vHapticDisplacements;
	vector<int> m_vHapticIndices;
	int m_idxPulledVertex;

	int m_positiveDefiniteSolver;

	bool m_bRenderFixedVertices;
	bool m_bRenderVertices;
};

#endif /* PS_DEFORMABLE_H_ */
