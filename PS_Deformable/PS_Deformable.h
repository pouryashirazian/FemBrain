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
#include "../PS_Base/PS_String.h"
#include "../PS_Graphics/PS_GLMeshBuffer.h"
#include "../PS_Graphics/PS_Vector.h"
#include "../PS_Graphics/PS_Box.h"

#include "SurfaceMesh.h"
#include "VolumeMesh.h"
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
using namespace PS::FEM;

#define DEFAULT_FORCE_NEIGHBORHOOD_SIZE 5


//ApplyDeformations
typedef void (*FOnApplyDeformations)(U32 dof, double* displacements);
//typedef Functor<void,  U32, double*> FOnDeformations;

struct FaceIntersection {
	int face;
	vec3d xyz;
	vec3d uvw;
};

struct EdgeIntersection {
	int edge;
	vec3d xyz;
};

/*!
 *	Deformable model
 */
class Deformable : public SceneNode {
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
	virtual ~Deformable();


	//Draw
	void draw();
	void drawCuttingArea();
	void drawTetElement(U32 el, vec4f& color);

	//TimeStep
	void timestep();

	//Pick a vertex
	int pickVertex(const vec3d& wpos, vec3d& vertex);
	int pickVertices(const vec3d& boxLo, const vec3d& boxHi,
					   vector<vec3d>& arrFoundCoords, vector<int>& arrFoundIndices) const;
	int pickVertices(const vec3d& boxLo, const vec3d& boxHi);

	//Fill Record for
	void statFillRecord(DBLogger::Record& rec) const;

	//Cutting and Haptics
	void cleanupCuttingStructures();
	int  performCuts(const vec3d& s0, const vec3d& s1);

	void setPulledVertex(int index);
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

	double m_dampingStiffnessCoeff;
	double m_dampingMassCoeff;
	double m_timeStep;
	double m_restVolume;

	//Cutting
	vector<pair<vec3d, vec3d> > m_vCuttingPath;
	vector<U32> m_vCrossedFaces;
	vector<U32> m_vCrossedTets;
	vec3d m_sweptQuad[4];
	bool m_isSweptQuadValid;

	typedef std::tr1::unordered_map<U32, FaceIntersection*>::iterator MAPFACE_ITER;
	typedef std::tr1::unordered_map<U32, EdgeIntersection*>::iterator MAPEDGE_ITER;
	std::tr1::unordered_map<U32, FaceIntersection*> m_mapElementFaceIntersection;
	std::tr1::unordered_map<U32, EdgeIntersection*> m_mapElementEdgeIntersection;
	/*
	vector<vec3d> m_vFaceIntersections;
	vector<vec3d> m_vFaceIntersectionsBarycoords;
	vector<vec3d> m_vEdgeIntersections;
	vector<vec3d> m_vEdgeIntersectionsBarycoords;
	*/

	//Surface Mesh
	SurfaceMesh* m_lpSurfaceMesh;
	//SceneObjectDeformable * m_lpDeformableMesh;


	//Mesh Graph
	Graph* m_lpMeshGraph;

	//Tetrahedra input mesh
	TetMesh* m_lpTetMesh;

	//Deformable Model
	CorotationalLinearFEM* m_lpDeformable;

	//ForceModel
	ForceModel* m_lpDeformableForceModel;

	//Integrator
	ImplicitNewmarkSparse* m_lpIntegrator;


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
