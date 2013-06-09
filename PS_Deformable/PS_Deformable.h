/*
 * PS_Deformable.h
 *
 *  Created on: Sep 29, 2012
 *      Author: pourya
 */

#ifndef PS_DEFORMABLE_H_
#define PS_DEFORMABLE_H_

#include <vector>
#include "loki/Functor.h"
#include "../PS_Base/PS_MathBase.h"
#include "../PS_Base/PS_String.h"
#include "../PS_Graphics/PS_GLMeshBuffer.h"
#include "../PS_Graphics/PS_Vector.h"
#include "../PS_Graphics/PS_Box.h"

#include "corotationalLinearFEM.h"
#include "corotationalLinearFEMForceModel.h"
#include "generateMassMatrix.h"
//#include "implicitBackwardEulerSparse.h"
#include "PS_VolumeConservingIntegrator.h"
#include "OclVolConservedIntegrator.h"
#include "sceneObjectDeformable.h"
#include "graph.h"
#include "DBLogger.h"

using namespace std;
using namespace Loki;
using namespace PS::MATH;

#define DEFAULT_FORCE_NEIGHBORHOOD_SIZE 5


//ApplyDeformations
typedef void (*FOnApplyDeformations)(U32 dof, double* displacements);
//typedef Functor<void,  U32, double*> FOnDeformations;

/*!
 *	Deformable model
 */
class Deformable {
public:
	Deformable();

	/*!
	 * Constructs a deformable model from the veg file and the rendereable mesh
	 * from the obj file. Fixed vertices are setup in the ini file.
	 */
	explicit Deformable(const char* lpVegFilePath,
						   const char* lpObjFilePath,
						   std::vector<int>& vFixedVertices,
						   int ctThreads = 0,
						   const char* lpModelTitle = NULL);

	/*!
	 * Constructs a deformable model from the BlobTree by performing a round of
	 * polygonization.
	 */
	explicit Deformable(U32 ctVertices, double* lpVertices,
						   U32 ctElements, int* lpElements,
						   std::vector<int>& vFixedVertices,
						   int ctThreads = 0);

	virtual ~Deformable();


	//Draw
	void draw();

	//TimeStep
	void timestep();

	//Pick a vertex
	int pickVertex(const vec3d& wpos, vec3d& vertex);
	int pickVertices(const vec3d& boxLo, const vec3d& boxHi,
					   vector<vec3d>& arrFoundCoords, vector<int>& arrFoundIndices) const;
	int pickVertices(const vec3d& boxLo, const vec3d& boxHi);

	//Fill Record for
	void statFillRecord(DBLogger::Record& rec) const;

	//Haptic Interaction
	void setPulledVertex(int index) { m_idxPulledVertex = index;}
	bool hapticStart(int index);
	bool hapticStart(const vec3d& wpos);
	void hapticEnd();
	bool hapticUpdateDisplace();
	void hapticSetCurrentForces(const vector<int>& indices,
									const vector<vec3d>& forces);

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

	AABB aabb() const { return m_aabb;}

	double computeVolume() const;
	bool isVolumeChanged() const { return  !EssentiallyEquald(this->computeVolume(), m_restVolume, 0.0001);}

	//Access TetMesh for stats
	TetMesh* getTetMesh() const {return m_lpTetMesh;}

	//ModelName
	string getModelName() const {return m_strModelName;}
	void setModelName(const string& strModelName) {m_strModelName = strModelName;}

	//Gravity
	void setGravity(bool bGravity) {m_bApplyGravity = bGravity;}
	bool getGravity() const {return m_bApplyGravity;}

	//Apply External Forces
	bool applyAllExternalForces();
	bool applyHapticForces();

	//Set callbacks
	void setDeformCallback(FOnApplyDeformations fOnDeform) {
		m_fOnDeform = fOnDeform;
	}

	double getSolverTime() const { return m_lpIntegrator->GetSystemSolveTime();}
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
			    std::vector<int>& vFixedVertices,
			    int ctThreads = 0,
			    const char* lpModelTitle = NULL);

	void setup(U32 ctVertices, double* lpVertices,
				U32 ctElements, int* lpElements,
				std::vector<int>& vFixedVertices,
				int ctThreads = 0);

	void setupIntegrator(int ctThreads = 8);

	void cleanup();

private:
	//Callbacks
	FOnApplyDeformations m_fOnDeform;


	AABB m_aabb;

	double m_dampingStiffnessCoeff;
	double m_dampingMassCoeff;
	double m_timeStep;
	double m_restVolume;

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
	ImplicitNewmarkSparse* m_lpIntegrator;

	//Deformable Mesh
	SceneObjectDeformable * m_lpDeformableMesh;

	//Mass Matrix
	SparseMatrix* m_lpMassMatrix;

	//Time Step Counter
	U32 m_ctTimeStep;

	U32 m_dof;
	double* m_arrDisplacements;
	double* m_arrExtForces;
	double* m_arrElementVolumes;

	//Fixed Vertices
	vector<int> m_vFixedVertices;
	vector<int> m_vFixedDofs;

	//Haptic forces
	bool m_bHapticInProgress;
	int m_hapticForceNeighorhoodSize;
	double m_hapticCompliance;

	//vector<vec3d> m_vHaptic
	vector<vec3d> m_vHapticForces;
	vector<vec3d> m_vHapticDisplacements;
	vector<int> m_vHapticIndices;
	int m_idxPulledVertex;
	int m_positiveDefiniteSolver;

	bool m_bRenderFixedVertices;
	bool m_bRenderVertices;
	bool m_bApplyGravity;

	string m_strModelName;
};

#endif /* PS_DEFORMABLE_H_ */
