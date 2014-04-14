//#include <tr1/unordered_map.h>
#include "Deformable.h"
#include "base/Logger.h"
#include "base/Profiler.h"
#include "base/FileDirectory.h"
#include "base/MathBase.h"
#include "graphics/AABB.h"
#include "graphics/Intersections.h"

#include "volumetricMeshLoader.h"
#include "volumetricMeshENuMaterial.h"
#include "generateMeshGraph.h"
#include <algorithm>
#include "tbb/task_scheduler_init.h"

using namespace __gnu_cxx;
using namespace std;

#define DEFAULT_TIME_STEP 0.0333


using namespace PS::INTERSECTIONS;

CuttableMesh* g_lpModifier = NULL;

Deformable::Deformable()
{
	init();
}

Deformable::Deformable(const char* lpVegFilePath,
						  const char* lpObjFilePath,
						  std::vector<int>& vFixedVertices,
						  int ctThreads,
						  const char* lpModelTitle)
{
	init();
	setup(lpVegFilePath, lpObjFilePath, vFixedVertices, ctThreads, lpModelTitle);
}

Deformable::Deformable(const vector<double>& inTetVertices,
				 	 const vector<U32>& inTetElements,
				 	 const vector<int>& vFixedVertices) {
	init();
	setupTetMesh(inTetVertices, inTetElements, vFixedVertices);
}


Deformable::~Deformable()
{
	this->cleanup();
	this->cleanupCuttingStructures();
}

void Deformable::cleanup()
{
	LogInfo("Cleaning up all objects for deformable.");

	//Cleanup all objects
	SAFE_DELETE(m_lpIntegrator);

	SAFE_DELETE(m_lpMassMatrix);

	SAFE_DELETE(m_lpMeshGraph);

	SAFE_DELETE(m_lpSurfaceMesh);

	//SAFE_DELETE(m_lpDeformableMesh);
	SAFE_DELETE(m_lpDeformableForceModel);
	SAFE_DELETE(m_lpDeformable);

	SAFE_DELETE(m_lpTetMesh);

	//Double Arrays
	SAFE_DELETE_ARRAY(m_q);
	SAFE_DELETE_ARRAY(m_qVel);
	SAFE_DELETE_ARRAY(m_qAcc);
	SAFE_DELETE_ARRAY(m_arrExtForces);
	SAFE_DELETE_ARRAY(m_arrElementVolumes);
}

void Deformable::init() {
	m_ctCollided = 0;
	m_strModelName = "FEMBRAIN";
	m_collisionObj = NULL;
	m_lpSurfaceMesh = NULL;
	m_lpTetMesh = NULL;

	//Init Vars
	m_fOnDeform = NULL;
	m_hapticCompliance = 1.0;
	m_idxPulledVertex = -1;
	m_hapticForceNeighorhoodSize = DEFAULT_FORCE_NEIGHBORHOOD_SIZE;
	m_bRenderFixedVertices = true;
	m_bRenderVertices = false;
	m_bHapticInProgress = false;

	// This option only affects PARDISO and SPOOLES solvers, where it is best
	// to keep it at 0, which implies a symmetric, non-PD solve.
	// With CG, this option is ignored.
	m_positiveDefiniteSolver = 0;

	// (tangential) Rayleigh damping
	// "underwater"-like damping
	m_dampingMassCoeff = 0.0;

	// (primarily) high-frequency damping
	m_dampingStiffnessCoeff = 0.01;

	//Time Step the model
	m_timeStep = 0.0333;
	m_ctTimeStep = 0;
	m_lpIntegrator = NULL;
}

//Setup
void Deformable::setup(const char* lpVegFilePath,
						  const char* lpObjFilePath,
						  std::vector<int>& vFixedVertices,
						  int ctThreads,
						  const char* lpModelTitle)
{
	//Set Model Name
	if(lpModelTitle == NULL)
		m_strModelName = PS::FILESTRINGUTILS::ExtractFileTitleOnly(AnsiStr(lpVegFilePath)).cptr();
	else
		m_strModelName = string(lpModelTitle);

	//Adapting to VegaFem input
	vector<int> arrFixed;
	arrFixed.resize(vFixedVertices.size());
	for(U32 i=0; i<vFixedVertices.size(); i++)
		arrFixed[i] = (int)vFixedVertices[i];

	m_lpSurfaceMesh = new SurfaceMesh(lpObjFilePath);
	m_lpSurfaceMesh->resetToRest();
	m_lpSurfaceMesh->setFixedVertices(vFixedVertices);

	//Setup Volumetric Mesh
	VolumetricMesh * lpVolMesh = VolumetricMeshLoader::load(const_cast<char*>(lpVegFilePath));
	if (lpVolMesh == NULL)
		LogError("Failed to load veg format mesh.");
	else
		LogInfoArg2("Success. Number of vertices: %d. Number of elements: %d.",
				    lpVolMesh->getNumVertices(),
				    lpVolMesh->getNumElements());

	if(lpVolMesh->getElementType() == VolumetricMesh::TET)
	{
		m_lpTetMesh = reinterpret_cast<TetMesh*>(lpVolMesh);
		LogInfo("TETMesh loaded successfully.");
	}
	else
		LogError("Loading mesh failed.");

	//MeshGraph
	m_lpMeshGraph = GenerateMeshGraph::Generate(m_lpTetMesh);

	//Setup Deformable Model
	m_lpDeformable = new CorotationalLinearFEM(m_lpTetMesh);

	//Setup Force Model
	m_lpDeformableForceModel = new CorotationalLinearFEMForceModel(m_lpDeformable);

	//Compute Mass Matrix
	GenerateMassMatrix::computeMassMatrix(m_lpTetMesh, &m_lpMassMatrix, true);

	//Copy from the input fixed vertices
	m_vFixedVertices.assign(vFixedVertices.begin(), vFixedVertices.end());

	// total number of DOFs
	m_dof = 3 * m_lpTetMesh->getNumVertices();
	m_q = new double[m_dof];
	m_qVel = new double[m_dof];
	m_qAcc = new double[m_dof];
	m_arrExtForces = new double[m_dof];

	//Create the integrator
	this->setupIntegrator(ctThreads);


	//Compute AABB
	this->setAABB(m_lpSurfaceMesh->aabb());

	//Compute Volume
	U32 ctElems = m_lpTetMesh->getNumElements();
	m_arrElementVolumes = new double[ctElems];
	m_restVolume = this->computeVolume(m_arrElementVolumes, ctElems);
}


int Deformable::setupTetMesh(const vector<double>& inTetVertices,
		 	 	 	 	 	 const vector<U32>& inTetElements,
		 	 	 	 	 	 const vector<int>& vFixedVertices) {

	vector<int> arrElements;
	arrElements.resize(inTetElements.size());
	for(U32 i=0; i<inTetElements.size(); i++)
		arrElements[i] = (int)inTetElements[i];

	//Topology
	//m_lpModifier = new CuttableMesh(inTetVertices, inTetElements);

	//Setup Boundary Mesh
	if(m_lpSurfaceMesh == NULL)
		m_lpSurfaceMesh = new SurfaceMesh();

	//Extract from tetmesh
	m_lpSurfaceMesh->setupFromTetMesh(inTetVertices, inTetElements);
	m_lpSurfaceMesh->resetToRest();
	m_lpSurfaceMesh->setFixedVertices(vFixedVertices);

	//Copy from the input fixed vertices
	m_vFixedVertices.assign(vFixedVertices.begin(), vFixedVertices.end());


	//Setup Volumetric Mesh
	int ctVertices = inTetVertices.size() / 3;
	int ctElements = inTetElements.size() / 4;
	SAFE_DELETE(m_lpTetMesh);
	m_lpTetMesh = new TetMesh(ctVertices, const_cast<double*>(&inTetVertices[0]),
							  ctElements, &arrElements[0]);

	//MeshGraph
	m_lpMeshGraph = GenerateMeshGraph::Generate(m_lpTetMesh);

	//Setup Deformable Model
	m_lpDeformable = new CorotationalLinearFEM(m_lpTetMesh);

	//Setup Force Model
	m_lpDeformableForceModel = new CorotationalLinearFEMForceModel(m_lpDeformable);

	//Compute Mass Matrix
	GenerateMassMatrix::computeMassMatrix(m_lpTetMesh, &m_lpMassMatrix, true);

	// total number of DOFs
	m_dof = 3 * m_lpTetMesh->getNumVertices();
	m_q = new double[m_dof];
	m_qVel = new double[m_dof];
	m_qAcc = new double[m_dof];

	m_arrExtForces = new double[m_dof];


	//Create the integrator
	this->setupIntegrator(tbb::task_scheduler_init::default_num_threads());


	//Compute AABB
	this->setAABB(m_lpSurfaceMesh->aabb());

	//Compute Volume
	U32 ctElems = m_lpTetMesh->getNumElements();
	m_arrElementVolumes = new double[ctElems];
	m_restVolume = this->computeVolume(m_arrElementVolumes, ctElems);

	return 1;
}

/*!
 * Fill the record for reporting stats.
 */
void Deformable::statFillRecord(DBLogger::Record& rec) const
{
	rec.ctElements = m_lpTetMesh->getNumElements();
	rec.ctVertices = m_lpTetMesh->getNumVertices();
	rec.restVolume = m_restVolume;
	rec.totalVolume = this->computeVolume();

	rec.youngModulo = 0.0;
	rec.poissonRatio = 0.0;
	if(m_lpTetMesh->getNumMaterials() > 0)
	{
		VolumetricMesh::Material* lpMaterial = m_lpTetMesh->getMaterial(0);
		if(lpMaterial->getType() == VolumetricMesh::Material::ENU)
		{
			VolumetricMesh::ENuMaterial* lpENuMaterial = reinterpret_cast<VolumetricMesh::ENuMaterial*>(lpMaterial);
			rec.youngModulo = lpENuMaterial->getE();
			rec.poissonRatio = lpENuMaterial->getNu();
		}
	}

	rec.xpElementType = "TET";
	rec.xpForceModel = "COROTATIONAL LINEAR FEM";

#ifdef PARDISO
	rec.xpIntegrator = "PARDISO DIRECT SOLVER";
#else
	rec.xpIntegrator = "JACOBI PRECONDITIONED CG";
#endif
	rec.xpModelName = m_strModelName;
	rec.xpTime = DBLogger::timestamp();
}

double Deformable::computeVolume(double* arrStore, U32 count) const
{
	if(!m_lpTetMesh)
		return 0.0;

	U32 ctElements = m_lpTetMesh->getNumElements();
	double vol = 0.0;
	bool store = false;
	if(arrStore != NULL && count == ctElements)
		store = true;

	for(U32 i=0; i<ctElements; i++)
	{
		double cur = m_lpTetMesh->getElementVolume(i);
		if(store)
			arrStore[i] = cur;
		vol += cur;
	}
	return vol;
}

void Deformable::setDampingStiffnessCoeff(double s)
{
	m_dampingStiffnessCoeff = s;
	m_lpIntegrator->SetDampingStiffnessCoef(s);
}

void Deformable::setDampingMassCoeff(double m)
{
	m_dampingMassCoeff = m;
	m_lpIntegrator->SetDampingMassCoef(m);
}

//Computes constrained dofs based on selected fixed vertices
int Deformable::FixedVerticesToFixedDOF(std::vector<int>& arrInFixedVertices,
	  								 	     std::vector<int>& arrOutFixedDOF)

{
	if(arrInFixedVertices.size() == 0)
		return 0;

	//Fixed Vertices
	std::sort(arrInFixedVertices.begin(), arrInFixedVertices.end());

	arrOutFixedDOF.resize(arrInFixedVertices.size() * 3);
	for(U32 i=0; i < arrInFixedVertices.size(); i++)
	{
		int start = arrInFixedVertices[i] * 3;
		arrOutFixedDOF[i*3] = start;
		arrOutFixedDOF[i*3 + 1] = start + 1;
		arrOutFixedDOF[i*3 + 2] = start + 2;
	}

	return arrOutFixedDOF.size();
}


//TimeStep the animation
void Deformable::timestep()
{
	//Apply forces
	if(m_lpIntegrator == NULL)
		return;

	// important: must always clear forces, as they remain in effect unless changed
	m_lpIntegrator->SetExternalForcesToZero();

	//Apply external forces
	memset(m_arrExtForces, 0, sizeof(double) * m_dof);

	if(m_bApplyGravity && (m_ctCollided == 0)) {
		for(U32 i=0; i < m_dof; i++) {
			if(i % 3 == 1) {
				m_arrExtForces[i] += -10.0;
			}
		}
	}


	//Haptic
	applyHapticForces();
	m_lpIntegrator->SetExternalForces(m_arrExtForces);

	//Time Step
	m_lpIntegrator->DoTimestep();
	m_lpIntegrator->GetqState(m_q, m_qVel, m_qAcc);


	//Collision
	vec3f c = m_collisionObj->transform()->forward().map(vec3f(0,0,0));
	vec3d n = vec3d(0.0, 1.0, 0.0);

	//p current and rest
	vec3d pc, pr;

	//displacement and velocity
	vec3d q, v;

	m_ctCollided = 0;
	for (U32 i = 0; i < m_lpSurfaceMesh->countVertices(); i++) {
		pr = m_lpSurfaceMesh->vertexRestPosAt(i);
		q = vec3d(&m_q[i*3]);

		pc = pr + q;
		if(pc.y <= c.y) {
			m_ctCollided++;
		}
	}

	//response
	if(m_ctCollided > 0) {
		U32 dof = 0;
		for (U32 i = 0; i < m_lpSurfaceMesh->countVertices(); i++) {
			dof = i*3;

			pr = m_lpSurfaceMesh->vertexRestPosAt(i);
			q = vec3d(&m_q[dof]);
			pc = pr + q;


			v = vec3d(&m_qVel[dof]);
			vec3d vn = n * vec3d::dot(v, n);
			vec3d vp = v - vn;
			vec3d vr = vp - vn * 0.8;

			//set
			m_qAcc[dof] = 0.0;
			m_qAcc[dof+1] = 0.0;
			m_qAcc[dof+2] = 0.0;

			m_qVel[dof] = vr.x;
			m_qVel[dof+1] = vr.y;
			m_qVel[dof+2] = vr.z;

			//Collision
			if(pc.y <= c.y) {
				m_q[dof] = q.x;
				m_q[dof+1] = c.y - pr.y;
				m_q[dof+2] = q.z;
			}
		}


		m_lpIntegrator->SetqState(m_q, m_qVel, m_qAcc);
	}


	//Update AABB
	setAABB(m_lpSurfaceMesh->aabb());

	//Apply displacements
	m_lpSurfaceMesh->applyDisplacements(m_q);
	if(m_fOnDeform)
		m_fOnDeform(m_dof, m_q);

	//Increment time step
	m_ctTimeStep++;
}

int Deformable::pickVertex(const vec3d& wpos, vec3d& vertex)
{
	double dist;
	int index = m_lpSurfaceMesh->findClosestVertex(wpos, dist, vertex);
	LogInfoArg2("Clicked on vertex: %d (0-indexed). Dist: %.2f", index, dist);
	return index;
}

int Deformable::pickVertices(const AABB& box,
							 vector<vec3f>& arrPickedVertices,
				 	 	 	 vector<int>& arrPickedIndices) {
	arrPickedVertices.resize(0);
	arrPickedIndices.resize(0);
	U32 ctVertices = m_lpSurfaceMesh->countVertices();
	for(U32 i=0; i<ctVertices;i++)
	{
		vec3d v = m_lpSurfaceMesh->vertexAt(i);
		vec3f vf = vec3f(v.x, v.y, v.z);
		if(Contains<float>(box.lower(), box.upper(), vf)) {
			arrPickedVertices.push_back(vf);
			arrPickedIndices.push_back(i);
		}
	}

	return (int)arrPickedVertices.size();
}

int Deformable::pickVertices(const vec3d& boxLo, const vec3d& boxHi,
								 vector<vec3d>& arrFoundCoords, vector<int>& arrFoundIndices) const
{
	arrFoundCoords.resize(0);
	arrFoundIndices.resize(0);
	U32 ctVertices = m_lpSurfaceMesh->countVertices();
	for(U32 i=0; i<ctVertices;i++)
	{
		vec3d v = m_lpSurfaceMesh->vertexAt(i);

		if(Contains<double>(boxLo, boxHi, v))
		{
			arrFoundCoords.push_back(v);
			arrFoundIndices.push_back(i);
		}
	}

	return (int)arrFoundCoords.size();
}

int Deformable::pickVertices(const vec3d& boxLo, const vec3d& boxHi)
{
	m_vHapticIndices.resize(0);
	m_vHapticDisplacements.resize(0);

	U32 ctVertices = m_lpSurfaceMesh->countVertices();
	for(U32 i=0; i<ctVertices;i++)
	{
		vec3d vv = m_lpSurfaceMesh->vertexAt(i);

		if(Contains<double>(boxLo, boxHi, vv))
		{
			m_vHapticDisplacements.push_back(vv);
			m_vHapticIndices.push_back(i);
		}
	}

	return (int)m_vHapticDisplacements.size();
}

bool Deformable::addFixedVertex(int index)
{
	for(U32 i=0; i< m_vFixedVertices.size(); i++)
	{
		if(m_vFixedVertices[i] == index)
			return false;
	}

	m_vFixedVertices.push_back(index);
	updateFixedVertices();

	return true;
}

bool Deformable::removeFixedVertex(int index){

	bool bChanged = false;
	for(U32 i=0; i< m_vFixedVertices.size(); i++)
	{
		if(m_vFixedVertices[i] == index)
		{
			m_vFixedVertices.erase(m_vFixedVertices.begin() + i);
			bChanged = true;
			break;
		}
	}

	if(!bChanged)
		return false;

	updateFixedVertices();
	return true;
}

bool Deformable::setFixedVertices(const std::vector<int>& vFixedVertices) {

	m_vFixedVertices.assign(vFixedVertices.begin(), vFixedVertices.end());
	return updateFixedVertices();
}

int  Deformable::getFixedVertices(vector<int>& vFixedVertices) {
	vFixedVertices.assign(m_vFixedVertices.begin(), m_vFixedVertices.end());
	return (int)m_vFixedVertices.size();
}

bool Deformable::updateFixedVertices() {
	//Update DOFS
	FixedVerticesToFixedDOF(m_vFixedVertices, m_vFixedDofs);

	//Set fixed dofs
	m_lpIntegrator->setConstrainedDOF(m_vFixedDofs.size(), &m_vFixedDofs[0]);

	return true;
}

void Deformable::setupIntegrator(int ctThreads)
{
	//Update DOFS
	FixedVerticesToFixedDOF(m_vFixedVertices, m_vFixedDofs);

	//Rebuilt Integrator
	SAFE_DELETE(m_lpIntegrator);
	LogInfoArg1("Setup Integrator with %d threads.", ctThreads);

	// initialize the Integrator
	m_lpIntegrator = new VolumeConservingIntegrator(m_dof, m_timeStep,
													 m_lpMassMatrix,
													 m_lpDeformableForceModel,
													 m_positiveDefiniteSolver,
													 m_vFixedDofs.size(),
													 &m_vFixedDofs[0],
													 m_dampingMassCoeff,
													 m_dampingStiffnessCoeff,
													 1, 1E-6, ctThreads);

}

void Deformable::setPulledVertex(int index) {
	m_idxPulledVertex = index;
}

bool Deformable::hapticStart(int index)
{
	m_idxPulledVertex = index;
	m_bHapticInProgress = true;
	m_vHapticIndices.resize(0);
	m_vHapticDisplacements.resize(0);
	cleanupCuttingStructures();

	return true;
}

bool Deformable::hapticStart(const vec3d& wpos)
{
	vec3d vertex;
	m_idxPulledVertex = pickVertex(wpos, vertex);
	if (m_vFixedVertices.size() > 0) {
		for (U32 i = 0; i < m_vFixedVertices.size(); i++) {
			if (m_vFixedVertices[i] == m_idxPulledVertex)
				return false;
		}
	}

	//Call the usual
	return hapticStart(m_idxPulledVertex);
}

void Deformable::hapticEnd()
{
	m_bHapticInProgress = false;
	m_idxPulledVertex = -1;
	m_vHapticIndices.resize(0);
	m_vHapticDisplacements.resize(0);

	cleanupCuttingStructures();
}

bool Deformable::collisionDetect() {
	if(!m_collisionObj)
		return false;

	//ProfileAuto();
	AABB box2 = m_collisionObj->aabb();
	box2.transform(m_collisionObj->transform()->forward());


	if(this->aabb().intersect(box2)) {
		vec3f c = m_collisionObj->transform()->forward().map(vec3f(0,0,0));
		vec3d n = vec3d(0.0, 1.0, 0.0);

		//Count collided vertices
		U32 ctCollided = 0;
		U32 dof = 0;
		for (U32 i = 0; i < m_lpSurfaceMesh->countVertices(); i++) {
			vec3d pc = m_lpSurfaceMesh->vertexAt(i);
			if (pc.y <= (double)c.y)
				ctCollided++;
		}

		if(ctCollided > 0) {
			printf("Compensation!\n");

			for (U32 i = 0; i < m_lpSurfaceMesh->countVertices(); i++) {
				dof = i * 3;
				vec3d pc = m_lpSurfaceMesh->vertexAt(i);
				vec3d q   = vec3d(&m_q[dof]);
				//vec3d acc = vec3d(&m_qAcc[dof]);
				vec3d vel = vec3d(&m_qVel[dof]);

				vec3d vn = n * vec3d::dot(vel, n);
				vec3d vp = vel - vn;
				vec3d vr = vp - vn * 0.8;

				m_qAcc[dof] = 0.0;
				m_qAcc[dof+1] = 0.0;
				m_qAcc[dof+2] = 0.0;

				//vel = vel * (-0.80);
				m_qVel[dof] = vr.x;
				m_qVel[dof+1] = vr.y;
				m_qVel[dof+2] = vr.z;

				//Collision
				if (pc.y <= (double)c.y) {
					ctCollided++;

					double delta = (double)c.y - pc.y;

					m_q[dof] = q.x;
					m_q[dof+1] = q.y + delta;
					m_q[dof+2] = q.z;
				}
			}
		}


		m_lpIntegrator->SetqState(m_q, m_qVel, m_qAcc);

		//Collided
//		if (ctCollided > 0) {
//			for (U32 i = 0; i < m_lpSurfaceMesh->countVertices(); i++) {
//				m_arrExtForces[i * 3] = force.x;
//				m_arrExtForces[i * 3 + 1] = force.y;
//				m_arrExtForces[i * 3 + 2] = force.z;
//			}
//		}


		return true;
	}
	else {
		//Apply Gravity along global y direction: W = mg
		if(m_bApplyGravity) {
			for(U32 i=0; i < m_dof; i++) {
				if(i % 3 == 1) {
					m_arrExtForces[i] += -10.0;
				}
			}
		}
	}


	return false;
}

bool Deformable::applyHapticForces() {
	if (m_vHapticIndices.size() == 0)
		return false;
	if (!m_bHapticInProgress)
		return false;

	//Instead of pulled vertex we may now have an array of vertices
	for(U32 i=0; i<m_vHapticIndices.size(); i++)
	{
		int idxVertex = m_vHapticIndices[i];
		m_arrExtForces[3 * idxVertex + 0] += m_vHapticForces[i].x;
		m_arrExtForces[3 * idxVertex + 1] += m_vHapticForces[i].y;
		m_arrExtForces[3 * idxVertex + 2] += m_vHapticForces[i].z;
	}

	//Distribute force over the neighboring vertices
	for(U32 iVertex = 0; iVertex < m_vHapticIndices.size(); iVertex++)
	{
		set<int> affectedVertices;
		set<int> lastLayerVertices;
		affectedVertices.insert(m_vHapticIndices[iVertex]);
		lastLayerVertices.insert(m_vHapticIndices[iVertex]);
		vec3d extForce = m_vHapticForces[iVertex];

		for (int j = 1; j < m_hapticForceNeighorhoodSize; j++)
		{
			// linear kernel
			double forceMagnitude = 1.0 * (m_hapticForceNeighorhoodSize - j)
					/ static_cast<double>(m_hapticForceNeighorhoodSize);

			set<int> newAffectedVertices;
			for (set<int>::iterator iter = lastLayerVertices.begin();
					iter != lastLayerVertices.end(); iter++)
			{
				// traverse all neighbors and check if they were already previously inserted
				int vtx = *iter;
				int deg = m_lpMeshGraph->GetNumNeighbors(vtx);
				for (int k = 0; k < deg; k++) {
					int vtxNeighbor = m_lpMeshGraph->GetNeighbor(vtx, k);

					//If the vertex was not in the set of affected vertices then
					//Add it to the new set
					if (affectedVertices.find(vtxNeighbor)
							== affectedVertices.end()) {
						// discovered new vertex
						newAffectedVertices.insert(vtxNeighbor);
					}
				}
			}

			//LastLayerVertices only keeps the last circle of vertices, so clear it now
			lastLayerVertices.clear();

			//Apply forces to the newly affected vertices
			for (set<int>::iterator iter = newAffectedVertices.begin();
					iter != newAffectedVertices.end(); iter++)
			{

				// apply force
				m_arrExtForces[3 * *iter + 0] += forceMagnitude * extForce.x;
				m_arrExtForces[3 * *iter + 1] += forceMagnitude * extForce.y;
				m_arrExtForces[3 * *iter + 2] += forceMagnitude * extForce.z;

				// generate new layers
				lastLayerVertices.insert(*iter);
				affectedVertices.insert(*iter);
			}
		}
	}

	return true;
}

/*!
 * Update model based on displacements induced on model.
 */
bool Deformable::hapticUpdateDisplace()
{
	if(m_vHapticIndices.size() == 0)
		return false;
	if (!m_bHapticInProgress)
		return false;


	//Reset Displacements Vector
	memset(m_q, 0, sizeof(double) * m_dof);

	//Instead of pulled vertex we may now have an array of vertices
	for(size_t i=0; i<m_vHapticIndices.size(); i++)
	{
		int idxVertex = m_vHapticIndices[i];
		m_q[3 * idxVertex + 0] = m_vHapticDisplacements[i].x;
		m_q[3 * idxVertex + 1] = m_vHapticDisplacements[i].y;
		m_q[3 * idxVertex + 2] = m_vHapticDisplacements[i].z;
	}

	//Distribute displace over the neighboring vertices
	for(size_t iVertex = 0; iVertex < m_vHapticIndices.size(); iVertex++)
	{
		set<int> affectedVertices;
		set<int> lastLayerVertices;
		affectedVertices.insert(m_vHapticIndices[iVertex]);
		lastLayerVertices.insert(m_vHapticIndices[iVertex]);
		vec3d displace = m_vHapticDisplacements[iVertex];

		for (int j = 1; j < m_hapticForceNeighorhoodSize; j++)
		{
			// linear kernel
			double displaceMagnitude = 1.0 * (m_hapticForceNeighorhoodSize - j)
					/ static_cast<double>(m_hapticForceNeighorhoodSize);

			set<int> newAffectedVertices;
			for (set<int>::iterator iter = lastLayerVertices.begin();
					iter != lastLayerVertices.end(); iter++)
			{
				// traverse all neighbors and check if they were already previously inserted
				int vtx = *iter;
				int deg = m_lpMeshGraph->GetNumNeighbors(vtx);
				for (int k = 0; k < deg; k++) {
					int vtxNeighbor = m_lpMeshGraph->GetNeighbor(vtx, k);

					//If the vertex was not in the set of affected vertices then
					//Add it to the new set
					if (affectedVertices.find(vtxNeighbor)
							== affectedVertices.end()) {
						// discovered new vertex
						newAffectedVertices.insert(vtxNeighbor);
					}
				}
			}

			//LastLayerVertices only keeps the last circle of vertices, so clear it now
			lastLayerVertices.clear();

			//Apply forces to the newly affected vertices
			for (set<int>::iterator iter = newAffectedVertices.begin();
					iter != newAffectedVertices.end(); iter++)
			{

				// apply force
				m_q[3 * *iter + 0] += displaceMagnitude * displace.x;
				m_q[3 * *iter + 1] += displaceMagnitude * displace.y;
				m_q[3 * *iter + 2] += displaceMagnitude * displace.z;

				// generate new layers
				lastLayerVertices.insert(*iter);
				affectedVertices.insert(*iter);
			}
		}
	}

	//Compute external forces to be applied. Input: displacements, Output: forces
	memset(m_arrExtForces, 0, sizeof(double) * m_dof);
	m_lpDeformableForceModel->GetInternalForce(m_q, m_arrExtForces);

	//Negate forces
	//for(U32 i=0; i < m_dof; i++)
		//m_arrExtForces[i] = -1.0 * m_arrExtForces[i];
	m_lpIntegrator->SetExternalForces(m_arrExtForces);

	return true;
}

/*!
 * Haptic forces
 */
void Deformable::hapticSetCurrentForces(const vector<int>& indices,
											  const vector<vec3d>& forces)
{
	m_vHapticIndices.assign(indices.begin(), indices.end());
	m_vHapticForces.assign(forces.begin(), forces.end());
}

/*!
 * Haptic displacements
 */
void Deformable::hapticSetCurrentDisplacements(const vector<int>& indices,
													 const vector<vec3d>& displacements)
{
	m_vHapticIndices.assign(indices.begin(), indices.end());
	m_vHapticDisplacements.assign(displacements.begin(), displacements.end());
}

void Deformable::cleanupCuttingStructures() {
	m_vCrossedFaces.resize(0);
	m_vCrossedTets.resize(0);
	m_vCuttingPathEdge0.resize(0);
	m_vCuttingPathEdge1.resize(0);

	//Faces
    for(MAPFACE_ITER it = m_mapElementFaceIntersection.begin(); it != m_mapElementFaceIntersection.end(); it++) {
        SAFE_DELETE(it->second);
    }
    m_mapElementFaceIntersection.clear();

    //Edges
    for(MAPEDGE_ITER it = m_mapElementEdgeIntersection.begin(); it != m_mapElementEdgeIntersection.end(); it++) {
        SAFE_DELETE(it->second);
    }
    m_mapElementEdgeIntersection.clear();
    m_isSweptQuadValid = false;
}

int Deformable::performCuts(const vec3d& s0, const vec3d& s1)
{
	if(!m_lpSurfaceMesh || !m_lpTetMesh)
		return -1;
	U32 ctFaces = m_lpSurfaceMesh->countTriangles();
	U32 ctTets = m_lpTetMesh->getNumElements();
	if(ctFaces == 0)
		return -1;

	//Select a swept surface for edge cutting test
	const double minSweptLength = 0.01;
	m_isSweptQuadValid = false;
	m_sweptQuad[0] = s0;
	m_sweptQuad[1] = s1;
	if(m_vCuttingPathEdge0.size() > 1) {
		//Loop over the path from the recently added to the first one
		for(int i=(int)m_vCuttingPathEdge0.size()-1; i>=0; i--) {
			double d = vec3d::distance(m_vCuttingPathEdge0[i], s0);
			if(d >= minSweptLength) {
				m_sweptQuad[2] = m_vCuttingPathEdge0[i];
				m_sweptQuad[3] = m_vCuttingPathEdge1[i];
				m_isSweptQuadValid = true;
				break;
			}
		}
	}

	//Insert new scalpal position into buffer
	const U32 maxNodes = 512;
	m_vCuttingPathEdge0.push_back(s0);
	m_vCuttingPathEdge0.push_back(s1);
	if(m_vCuttingPathEdge0.size() > maxNodes)
		m_vCuttingPathEdge0.erase(m_vCuttingPathEdge0.begin());
	if(m_vCuttingPathEdge1.size() > maxNodes)
		m_vCuttingPathEdge1.erase(m_vCuttingPathEdge1.begin());


	//Return if not valid quad
	if(!m_isSweptQuadValid) {
		LogError("CUT: Swept quad is not valid yet!");
		return -1;
	}

	//Test for all tetrahedra. Per each tetrahedra test against the 6 edges and 4 faces
	//6 Edges: Against swept surface
	//4 Faces: Against cutting tip path

	vec3d elemVertices[4];
	int edgeBuffer[12];
	int edgeMask[6][2] = {
	   { 0, 1 }, { 1, 2 }, { 2, 0 },
	   { 0, 3 }, { 1, 3 }, { 2, 3 } };

	//Face Mask
	int faceMask[4][3] = {
	   {0, 1, 2}, {1, 2, 3}, {2, 3, 0}, {0, 1, 3}
	};

	U32 idxVertex;
	//Loop over elements
	for(U32 el=0; el<ctTets; el++) {
		//Get Vertices
		for(int i=0; i<4; i++) {
		    idxVertex = m_lpTetMesh->getVertexIndex(el, i);
		    elemVertices[i] = m_lpSurfaceMesh->vertexAt(idxVertex);
		}

		//Get Edges
		m_lpTetMesh->getElementEdges(el, &edgeBuffer[0]);

		//4 Face Intersections
		vec3d p[3];
		vec3d uvw;
		vec3d xyz;
		for(int i=0; i<4; i++) {
			p[0] = elemVertices[faceMask[i][0]];
			p[1] = elemVertices[faceMask[i][1]];
			p[2] = elemVertices[faceMask[i][2]];
			int res = IntersectSegmentTriangle(s0, s1, p, uvw, xyz);
			if(res > 0) {
				FaceIntersection* fi = new FaceIntersection();
				fi->face = i;
				fi->xyz = xyz;
				fi->uvw = uvw;
				m_mapElementFaceIntersection.insert(std::make_pair(el, fi));
			}
		}


		//6 Edge Intersections
		/*
		if(m_isSweptQuadValid) {

			vec3d tri1[3];
			tri1[0] = m_sweptQuad[0];
			tri1[1] = m_sweptQuad[3];
			tri1[2] = m_sweptQuad[1];

			vec3d tri2[3];
			tri2[0] = m_sweptQuad[0];
			tri2[1] = m_sweptQuad[2];
			tri2[2] = m_sweptQuad[3];


			for(int i=0; i<6; i++) {
				vec3d edge0 = vertices[ edgeMask[i][0] ];
				vec3d edge1 = vertices[ edgeMask[i][1] ];

				int res = IntersectSegmentTriangle(edge0, edge1, tri1, uvw, xyz);
				if(res == 0)
					res = IntersectSegmentTriangle(edge0, edge1, tri2, uvw, xyz);

				if(res > 0)
				{
					EdgeIntersection* ei = new EdgeIntersection();
					ei->edge = i;
					ei->xyz = xyz;
					m_mapElementEdgeIntersection.insert(std::make_pair(el, ei));
				}
			}
		}
		*/
	}

	//
	return m_mapElementFaceIntersection.size() + m_mapElementEdgeIntersection.size();

	//Crossed Faces
	/*
	m_vCrossedFaces.resize(0);
	m_vCrossedFaces.reserve(32);
	m_vCrossedTets.resize(0);
	m_vCrossedTets.reserve(32);

	//Maps vertex to face index
	double tri[3][3];
	double center[3];
	double halfLengths[3];
	U32 ctCrossed = 0;
	std::tr1::unordered_map<U32, U32> mapCrossedVertices;

	//Center and HalfLength Needed once
	avatarCenter.store(center);
	avatarHalfLength.store(halfLengths);


	//2.Find all faces intersected with avatar
	for(U32 iface=0; iface < ctFaces; iface++) {
		vec3u32 face = m_lpSurfaceMesh->faceAt(iface);
		//Copy Face Pos
		for(int i=0; i < 3; i++) {
			vec3d p = m_lpSurfaceMesh->faceVertexAt(iface, i);
			for(int j=0; j < 3; j++) {
				tri[i][j] = p.element(j);
			}
		}

		int isCrossed = IntersectBoxTriangle<double>(center, halfLengths, tri);
		ctCrossed += isCrossed;
		if(isCrossed) {
			m_vCrossedFaces.push_back(iface);
			for(int i=0; i < 3; i++) {
				//Maps vertices to faces
				if(mapCrossedVertices.find(face.element(i) ) == mapCrossedVertices.end())
					mapCrossedVertices.insert(std::make_pair(face.element(i), iface));
			}
		}
	}

	//Log number of crossed faces
	if(ctCrossed > 0)
		LogInfoArg1("Crossed faces count = %d", ctCrossed);

	//Remove crossed faces
//	m_lpSurfaceMesh->removeTriangles(crossedFaces);
//	m_lpSurfaceMesh->updateFaceBuffer();


	//Nearby Tet elements:
	if(ctCrossed == 0)
		return -2;

	//2.Find all intersecting tetrahedra
	U32 ctTetrahedra = m_lpTetMesh->getNumElements();
	int indices[4];
	vec3d a,b,c,d;

	//Loop Over Elements
	for(U32 iTet=0; iTet<ctTetrahedra; iTet++)
	{
		indices[0] = m_lpTetMesh->getVertexIndex(iTet, 0);
		indices[1] = m_lpTetMesh->getVertexIndex(iTet, 1);
		indices[2] = m_lpTetMesh->getVertexIndex(iTet, 2);
		indices[3] = m_lpTetMesh->getVertexIndex(iTet, 3);

		int ctUsedVertex = 0;
		for(int j=0; j<4; j++) {
			if(mapCrossedVertices.find(indices[j]) != mapCrossedVertices.end()) {
				ctUsedVertex++;
			}
		}

		if(ctUsedVertex != 3)
			continue;

		//Element tobe removed
		m_vCrossedTets.push_back(iTet);
	}
	*/
	//Update Meshes
	//Update System Matrices: Mass, Stiffness, Damping
	/*
	for(int i=0; i<elementsToBeRemoved; i++) {
		m_lp
	}

	return (int)m_vCrossedTets.size();
	*/
}

void Deformable::drawTetElement(U32 el, vec4f& color) {
	if(m_lpSurfaceMesh == NULL || m_lpTetMesh == NULL)
		return;
	if(el >= (U32)m_lpTetMesh->getNumElements())
		return;

	vec3d a, b, c, d;
	a = m_lpSurfaceMesh->vertexAt(m_lpTetMesh->getVertexIndex(el, 0));
	b = m_lpSurfaceMesh->vertexAt(m_lpTetMesh->getVertexIndex(el, 1));
	c = m_lpSurfaceMesh->vertexAt(m_lpTetMesh->getVertexIndex(el, 2));
	d = m_lpSurfaceMesh->vertexAt(m_lpTetMesh->getVertexIndex(el, 3));

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glColor4fv(color.cptr());
	glBegin(GL_LINES);
	glVertex3dv(&a[0]);
	glVertex3dv(&b[0]);

	glVertex3dv(&b[0]);
	glVertex3dv(&d[0]);

	glVertex3dv(&a[0]);
	glVertex3dv(&d[0]);

	glVertex3dv(&a[0]);
	glVertex3dv(&c[0]);

	glVertex3dv(&b[0]);
	glVertex3dv(&c[0]);

	glVertex3dv(&c[0]);
	glVertex3dv(&d[0]);
	glEnd();
	glPopAttrib();

}

void Deformable::drawCuttingArea() {
/*
	//Test
	vec3d s0 = vec3d(0, -2, 0);
	vec3d s1 = vec3d(0, 4, 0);
	vec3d p[3];
	p[0] = vec3d(-3, 1, 3);
	p[1] = vec3d(3, 1, 3);
	p[2] = vec3d(0, 1, -3);

	vec3d xyz, uvw;
	int res = IntersectSegmentTriangle(s0, s1, p, uvw, xyz);
	if(res > 0) {
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColor3d(0.0, 1.0, 0.0);
			glBegin(GL_TRIANGLES);
			glVertex3dv(p[0].cptr());
			glVertex3dv(p[1].cptr());
			glVertex3dv(p[2].cptr());
			glEnd();

			glLineWidth(1.0f);
			glColor3d(0,0,0);
			glBegin(GL_LINES);
			glVertex3dv(s0.cptr());
			glVertex3dv(s1.cptr());
			glEnd();

			glPointSize(4.0f);
			glColor3d(1.0, 0.0, 0.0);
			glBegin(GL_POINTS);
			glVertex3d(xyz.x, xyz.y, xyz.z);
			glEnd();
		glPopAttrib();
	}
*/

	vec4f red(0.4,0,0,1);
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPointSize(5.0f);
	//FACE POINTS
	glColor3f(0.0f, 0.0f, 1.0f);
		glBegin(GL_POINTS);
		for(MAPFACE_ITER it = m_mapElementFaceIntersection.begin(); it != m_mapElementFaceIntersection.end(); it++ ) {
			glVertex3dv(it->second->xyz.cptr());
		}
		glEnd();

		//Draw Face Tets
		for(MAPFACE_ITER it = m_mapElementFaceIntersection.begin(); it != m_mapElementFaceIntersection.end(); it++ )
			drawTetElement(it->first, red);


	//Edge POINTS
	glColor3f(0.0f, 0.0f, 1.0f);
		glBegin(GL_POINTS);
		for(MAPEDGE_ITER it = m_mapElementEdgeIntersection.begin(); it != m_mapElementEdgeIntersection.end(); it++ ) {
			glVertex3dv(it->second->xyz.cptr());
		}
		glEnd();

	glPopAttrib();

	//Draw Scalpel history
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glLineWidth(1.0f);
		glColor3d(0.0, 1.0, 0.0);
		glBegin(GL_LINES);
		for(U32 i=0; i< m_vCuttingPathEdge0.size(); i+=16) {
			glVertex3dv(m_vCuttingPathEdge0[i].cptr());
			glVertex3dv(m_vCuttingPathEdge1[i].cptr());
		}
		glEnd();
	glPopAttrib();

	//Draw Swept Quad
	//if(m_isSweptQuadValid)
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColor3d(0.7, 0.7, 0.7);
			glBegin(GL_QUADS);
				glVertex3dv(m_sweptQuad[0].cptr());
				glVertex3dv(m_sweptQuad[1].cptr());
				glVertex3dv(m_sweptQuad[3].cptr());
				glVertex3dv(m_sweptQuad[2].cptr());
			glEnd();
		glPopAttrib();
	}

	/*
	if(m_vCrossedFaces.size() == 0)
		return;
	//Draw Cut Triangles
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(2.0f);
	glColor3f(0.0f, 1.0f, 0.0f);
	for(U32 i=0; i<m_vCrossedFaces.size(); i++) {
		vec3u32 t = m_lpSurfaceMesh->faceAt(m_vCrossedFaces[i]);
		glBegin(GL_LINE_LOOP);
			glVertex3dv(m_lpSurfaceMesh->vertexAt(t.x).cptr());
			glVertex3dv(m_lpSurfaceMesh->vertexAt(t.y).cptr());
			glVertex3dv(m_lpSurfaceMesh->vertexAt(t.z).cptr());
		glEnd();
	}
	glPopAttrib();

	//Draw Cut Tetrahedra
	vec4f color(0,0,0,1);
	vec3d a,b,c,d;
	int indices[4];
	for(U32 i=0; i<m_vCrossedTets.size(); i++) {
		U32 itet = m_vCrossedTets[i];
		a = m_lpSurfaceMesh->vertexAt(m_lpTetMesh->getVertexIndex(itet, 0));
		b = m_lpSurfaceMesh->vertexAt(m_lpTetMesh->getVertexIndex(itet, 1));
		c = m_lpSurfaceMesh->vertexAt(m_lpTetMesh->getVertexIndex(itet, 2));
		d = m_lpSurfaceMesh->vertexAt(m_lpTetMesh->getVertexIndex(itet, 3));

		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor4fv(color.ptr());
		glBegin(GL_LINES);
			glVertex3dv(&a[0]);
			glVertex3dv(&b[0]);

			glVertex3dv(&b[0]);
			glVertex3dv(&d[0]);

			glVertex3dv(&a[0]);
			glVertex3dv(&d[0]);

			glVertex3dv(&a[0]);
			glVertex3dv(&c[0]);

			glVertex3dv(&b[0]);
			glVertex3dv(&c[0]);

			glVertex3dv(&c[0]);
			glVertex3dv(&d[0]);
		glEnd();
		glPopAttrib();
	}
	*/
}

void Deformable::draw()
{
	m_lpSurfaceMesh->setWireFrameMode(getWireFrameMode());
	if(m_lpSurfaceMesh)
		m_lpSurfaceMesh->draw();

	if(m_idxPulledVertex >= 0  && m_idxPulledVertex < (int)m_lpSurfaceMesh->countVertices() ) {
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glEnable(GL_POLYGON_OFFSET_POINT);
			glPolygonOffset(-1.0f, -1.0f);
			glColor3f(0.0f, 0.0f, 1.0f);
			glPointSize(8.0f);
			glBegin(GL_POINTS);
				glVertex3dv(m_lpSurfaceMesh->vertexAt(m_idxPulledVertex).cptr());
			glEnd();
			glDisable(GL_POLYGON_OFFSET_FILL);
		glPopAttrib();
	}

	DrawAABB(this->aabb(), vec3f(0,0,0));
}
