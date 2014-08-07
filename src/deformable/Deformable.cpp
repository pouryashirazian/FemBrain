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

Deformable::Deformable(const vector<double>& inTetVertices,
				 	 const vector<U32>& inTetElements,
				 	 const vector<int>& vFixedVertices) {
	init();
	setupTetMesh(inTetVertices, inTetElements, vFixedVertices);
}


Deformable::~Deformable()
{
	this->cleanup();
}

void Deformable::cleanup()
{
	LogInfo("Cleaning up all objects for deformable.");

	//Cleanup all objects
	SAFE_DELETE(m_lpIntegrator);

	SAFE_DELETE(m_lpMassMatrix);

	//SAFE_DELETE(m_lpDeformableMesh);
	SAFE_DELETE(m_lpDeformableForceModel);
	SAFE_DELETE(m_lpDeformable);
	SAFE_DELETE(m_lpMesh);

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
	m_lpMesh = NULL;

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



int Deformable::setupTetMesh(const vector<double>& inTetVertices,
		 	 	 	 	 	 const vector<U32>& inTetElements,
		 	 	 	 	 	 const vector<int>& vFixedVertices) {

	vector<int> arrElements;
	arrElements.resize(inTetElements.size());
	for(U32 i=0; i<inTetElements.size(); i++)
		arrElements[i] = (int)inTetElements[i];

	//Compute Volume
	m_restVolume = computeVolume();

	//Copy from the input fixed vertices
	m_vFixedVertices.assign(vFixedVertices.begin(), vFixedVertices.end());

	//Setup Volumetric Mesh
	m_lpMesh = NULL;
	m_lpMesh = new CuttableMesh(inTetVertices, inTetElements);
	m_lpMeshGraph = GenerateMeshGraph::Generate(m_lpMesh);

	//Compute AABB
	setAABB(m_lpMesh->aabb());


	//Setup Deformable Model
	m_lpDeformable = new CorotationalLinearFEM(m_lpMesh);

	//Setup Force Model
	m_lpDeformableForceModel = new CorotationalLinearFEMForceModel(m_lpDeformable);

	//Compute Mass Matrix
	GenerateMassMatrix::computeMassMatrix(m_lpMesh, &m_lpMassMatrix, true);

	// total number of DOFs
	m_dof = 3 * m_lpMesh->getNumVertices();
	m_q = new double[m_dof];
	m_qVel = new double[m_dof];
	m_qAcc = new double[m_dof];

	m_arrExtForces = new double[m_dof];


	//Create the integrator
	this->setupIntegrator(tbb::task_scheduler_init::default_num_threads());

	return 1;
}

/*!
 * Fill the record for reporting stats.
 */
void Deformable::statFillRecord(DBLogger::Record& rec) const
{
	rec.ctElements = m_lpMesh->getNumElements();
	rec.ctVertices = m_lpMesh->getNumVertices();
	rec.restVolume = m_restVolume;
	rec.totalVolume = this->computeVolume();

	rec.youngModulo = 0.0;
	rec.poissonRatio = 0.0;
	if(m_lpMesh->getNumMaterials() > 0)
	{
		VolumetricMesh::Material* lpMaterial = m_lpMesh->getMaterial(0);
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
	if(!m_lpMesh)
		return 0.0;

	U32 ctElements = m_lpMesh->getNumElements();
	double vol = 0.0;
	bool store = false;
	if(arrStore != NULL && count == ctElements)
		store = true;

	for(U32 i=0; i<ctElements; i++)
	{
		double cur = m_lpMesh->getElementVolume(i);
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

	for (U32 i = 0; i < m_lpMesh->countNodes(); i++) {
		pr = m_lpMesh->vertexRestPosAt(i);

		q = vec3d(&m_q[i*3]);

		pc = pr + q;
		if(pc.y <= c.y) {
			m_ctCollided++;
		}
	}

	//response
	if(m_ctCollided > 0) {
		U32 dof = 0;
		for (U32 i = 0; i < m_lpMesh->countNodes(); i++) {
			dof = i*3;

			pr = m_lpMesh->vertexRestPosAt(i);
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


	//Apply displacements
	m_lpMesh->displace(m_q);

	//Update AABB
	setAABB(m_lpMesh->aabb());


	if(m_fOnDeform)
		m_fOnDeform(m_dof, m_q);

	//Increment time step
	m_ctTimeStep++;
}

int Deformable::pickVertex(const vec3d& wpos, vec3d& vertex)
{
	double dist;
	int index = m_lpMesh->findClosestVertex(wpos, dist, vertex);
	LogInfoArg2("Clicked on vertex: %d (0-indexed). Dist: %.2f", index, dist);
	return index;
}

int Deformable::pickVertices(const vec3d& boxLo, const vec3d& boxHi,
								 vector<vec3d>& arrFoundCoords, vector<int>& arrFoundIndices) const
{
	arrFoundCoords.resize(0);
	arrFoundIndices.resize(0);
	U32 ctVertices = m_lpMesh->countNodes();
	for(U32 i=0; i<ctVertices;i++)
	{
		vec3d v = m_lpMesh->vertexAt(i);

		if(Contains<double>(boxLo, boxHi, v))
		{
			arrFoundCoords.push_back(v);
			arrFoundIndices.push_back(i);
		}
	}

	return (int)arrFoundCoords.size();
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
		for (U32 i = 0; i < m_lpMesh->countNodes(); i++) {
			vec3d pc = m_lpMesh->vertexAt(i);
			if (pc.y <= (double)c.y)
				ctCollided++;
		}

		if(ctCollided > 0) {
			printf("Compensation!\n");

			for (U32 i = 0; i < m_lpMesh->countNodes(); i++) {
				dof = i * 3;
				vec3d pc = m_lpMesh->vertexAt(i);
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
					if (affectedVertices.find(vtxNeighbor) == affectedVertices.end()) {
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

void Deformable::draw()
{
	if(m_lpMesh)
		m_lpMesh->draw();

	if(m_idxPulledVertex >= 0  && m_idxPulledVertex < (int)m_lpMesh->countNodes() ) {
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glEnable(GL_POLYGON_OFFSET_POINT);
			glPolygonOffset(-1.0f, -1.0f);
			glColor3f(0.0f, 0.0f, 1.0f);
			glPointSize(8.0f);
			glBegin(GL_POINTS);
				glVertex3dv(m_lpMesh->vertexAt(m_idxPulledVertex).cptr());
			glEnd();
			glDisable(GL_POLYGON_OFFSET_FILL);
		glPopAttrib();
	}

	DrawAABB(this->aabb(), vec3f(0,0,0));
}
