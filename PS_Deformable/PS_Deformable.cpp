#include "PS_Deformable.h"
#include "../PS_Base/PS_Logger.h"
#include "../PS_Graphics/PS_Box.h"
#include "volumetricMeshLoader.h"
#include "generateMeshGraph.h"
#include <algorithm>

#define DEFAULT_TIME_STEP 0.0333
#define DEFAULT_FORCE_NEIGHBORHOOD_SIZE 5

Deformable::Deformable()
{
}

Deformable::Deformable(const char* lpVegFilePath,
						  const char* lpObjFilePath,
						  std::vector<int>& vFixedVertices)
{
	this->setup(lpVegFilePath, lpObjFilePath, vFixedVertices);
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

	SAFE_DELETE(m_lpMeshGraph);

	SAFE_DELETE(m_lpDeformableMesh);

	SAFE_DELETE(m_lpDeformableForceModel);

	SAFE_DELETE(m_lpDeformable);

	SAFE_DELETE(m_lpTetMesh);

	//Double Arrays
	SAFE_DELETE_ARRAY(m_arrDisplacements);
	SAFE_DELETE_ARRAY(m_arrExtForces);
}

//Setup
void Deformable::setup(const char* lpVegFilePath,
						  const char* lpObjFilePath,
						  std::vector<int>& vFixedVertices)
{
	m_lpDeformableMesh = new SceneObjectDeformable(const_cast<char*>(lpObjFilePath));
	//m_lpDeformableMesh->EnableVertexSelection();
	m_lpDeformableMesh->ResetDeformationToRest();
	m_lpDeformableMesh->BuildNeighboringStructure();
	m_lpDeformableMesh->BuildNormals();
	m_lpDeformableMesh->SetMaterialAlpha(0.5);
	m_idxPulledVertex = -1;
	m_bRenderFixedVertices = true;
	m_bRenderVertices = false;

	//m_hapticCompliance = 1.0;
	m_hapticCompliance = 1.0;
	m_hapticForceNeighorhoodSize = DEFAULT_FORCE_NEIGHBORHOOD_SIZE;
	m_bHapticInProgress = false;


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


	// This option only affects PARDISO and SPOOLES solvers, where it is best
	// to keep it at 0, which implies a symmetric, non-PD solve.
	// With CG, this option is ignored.
	m_positiveDefiniteSolver = 0;

	//Copy from the input fixed vertices
	m_vFixedVertices.assign(vFixedVertices.begin(), vFixedVertices.end());

	// (tangential) Rayleigh damping
	// "underwater"-like damping
	m_dampingMassCoeff = 0.0;

	// (primarily) high-frequency damping
	m_dampingStiffnessCoeff = 0.01;

	// total number of DOFs
	m_dof = 3 * m_lpTetMesh->getNumVertices();
	m_arrDisplacements = new double[m_dof];
	m_arrExtForces = new double[m_dof];

	//Time Step the model
	m_timeStep = 0.0333;
	m_ctTimeStep = 0;
	m_lpIntegrator = NULL;

	//Create the integrator
	this->setupIntegrator();


	//Compute AABB
	Vec3d lo, up;
	m_lpDeformableMesh->GetMesh()->getBoundingBox(1.0, &lo, &up);
	m_aabb.set(vec3(lo[0], lo[1], lo[2]), vec3(up[0], up[1], up[2]));

	//Compute model rest volume
	m_restVolume = this->computeVolume();
}

void Deformable::statFillRecord(DBLogger::Record& rec) const
{
	rec.ctElements = m_lpTetMesh->getNumElements();
	rec.ctVertices = m_lpTetMesh->getNumVertices();

	rec.msComputeDeformation = 0;
	rec.msComputeTetrahedra = 0;
	rec.msRenderTime = 0;
	rec.restVolume = m_restVolume;
	rec.totalVolume = this->computeVolume();
	rec.xpElementType = "TET";
	rec.xpForceModel = "COROTATIONAL LINEAR FEM";
	rec.xpIntegrator = "Backward Euler";
	rec.xpModelName = "DISC";
	rec.xpTime = DBLogger::timestamp();
}

double Deformable::computeVolume() const
{
	U32 ctElements = m_lpTetMesh->getNumElements();
	double vol = 0.0;
	for(U32 i=0; i<ctElements; i++)
	{
		vol += m_lpTetMesh->getElementVolume(i);
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
		/*
		if (m_ctTimeStep % 100 == 0) // set some force at the first timestep
		{
			LogInfoArg1("Apply force at timestep %d",m_ctTimeStep);
			memset(m_arrExtForces, 0, sizeof(double) * m_dof);

			// apply force of -500 N to vertex 12, in y-direction, 3*12+1 = 37
			m_idxPulledVertex = 12;
			m_arrExtForces[37] = -500;

			m_lpIntegrator->SetExternalForces(m_arrExtForces);
		}
		*/
	if(m_lpIntegrator == NULL)
		return;

	// important: must always clear forces, as they remain in effect unless changed
	m_lpIntegrator->SetExternalForcesToZero();

	//Update the haptic external forces applied
	//this->hapticUpdateDisplace();
	this->hapticUpdateForce();

	//Time Step
	m_lpIntegrator->DoTimestep();

	m_lpIntegrator->GetqState(m_arrDisplacements);

	m_lpDeformableMesh->SetVertexDeformations(m_arrDisplacements);

	//Increment time step
	m_ctTimeStep++;

	glutPostRedisplay();
}

int Deformable::pickVertex(const vec3d& wpos, vec3d& vertex)
{
	Vec3d world(wpos.x, wpos.y, wpos.z);
	double dist;
	double arrVPos[3];
	int index = m_lpDeformableMesh->FindClosestVertex(world, &dist, &arrVPos[0]);
	vertex = vec3d(&arrVPos[0]);
	LogInfoArg2("Clicked on vertex: %d (0-indexed). Dist: %.2f", index, dist);
	return index;
}

int Deformable::pickVertices(const vec3d& boxLo, const vec3d& boxHi,
								 vector<vec3d>& arrFoundCoords, vector<int>& arrFoundIndices) const
{
	arrFoundCoords.resize(0);
	arrFoundIndices.resize(0);
	ObjMesh* lpMesh = m_lpDeformableMesh->GetMesh();
	U32 ctVertices = lpMesh->getNumVertices();
	for(U32 i=0; i<ctVertices;i++)
	{
		Vec3d v = lpMesh->getPosition(i);
		vec3d vv = vec3d(v[0], v[1], v[2]);

		if(Contains<double>(boxLo, boxHi, vv))
		{
			arrFoundCoords.push_back(vv);
			arrFoundIndices.push_back(i);
		}
	}

	return (int)arrFoundCoords.size();
}

int Deformable::pickVertices(const vec3d& boxLo, const vec3d& boxHi)
{
	m_vHapticIndices.resize(0);
	m_vHapticDisplacements.resize(0);

	ObjMesh* lpMesh = m_lpDeformableMesh->GetMesh();
	U32 ctVertices = lpMesh->getNumVertices();
	for(U32 i=0; i<ctVertices;i++)
	{
		Vec3d v = lpMesh->getPosition(i);
		vec3d vv = vec3d(v[0], v[1], v[2]);

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
	this->setupIntegrator();

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

	this->setupIntegrator();
	return true;
}

void Deformable::setupIntegrator()
{
	//Update DOFS
	FixedVerticesToFixedDOF(m_vFixedVertices, m_vFixedDofs);

	//Rebuilt Integrator
	SAFE_DELETE(m_lpIntegrator);

	// initialize the Integrator
	m_lpIntegrator = new VolumeConservingIntegrator(m_dof, m_timeStep,
													 m_lpMassMatrix,
													 m_lpDeformableForceModel,
													 m_positiveDefiniteSolver,
													 m_vFixedDofs.size(),
													 &m_vFixedDofs[0],
													 m_dampingMassCoeff,
													 m_dampingStiffnessCoeff);
}

bool Deformable::hapticStart(int index)
{
	m_idxPulledVertex = index;
	m_bHapticInProgress = true;
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

	m_bHapticInProgress = true;
	return true;
}

void Deformable::hapticEnd()
{
	m_bHapticInProgress = false;
	m_idxPulledVertex = -1;
	m_vHapticIndices.resize(0);
	m_vHapticDisplacements.resize(0);
}

//Apply External Forces to the integrator
bool Deformable::hapticUpdateForce()
{
	if (m_vHapticIndices.size() == 0)
		return false;
	if (!m_bHapticInProgress)
		return false;

	//Reset External Force
	memset(m_arrExtForces, 0, sizeof(double) * m_dof);

	//Instead of pulled vertex we may now have an array of vertices
	for(size_t i=0; i<m_vHapticIndices.size(); i++)
	{
		int idxVertex = m_vHapticIndices[i];
		m_arrExtForces[3 * idxVertex + 0] += m_vHapticForces[i].x;
		m_arrExtForces[3 * idxVertex + 1] += m_vHapticForces[i].y;
		m_arrExtForces[3 * idxVertex + 2] += m_vHapticForces[i].z;
	}

	//Distribute force over the neighboring vertices
	for(size_t iVertex = 0; iVertex < m_vHapticIndices.size(); iVertex++)
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

	// set forces to the integrator
	m_lpIntegrator->SetExternalForces(m_arrExtForces);

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
	memset(m_arrDisplacements, 0, sizeof(double) * m_dof);

	//Instead of pulled vertex we may now have an array of vertices
	for(size_t i=0; i<m_vHapticIndices.size(); i++)
	{
		int idxVertex = m_vHapticIndices[i];
		m_arrDisplacements[3 * idxVertex + 0] = m_vHapticDisplacements[i].x;
		m_arrDisplacements[3 * idxVertex + 1] = m_vHapticDisplacements[i].y;
		m_arrDisplacements[3 * idxVertex + 2] = m_vHapticDisplacements[i].z;
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
				m_arrDisplacements[3 * *iter + 0] += displaceMagnitude * displace.x;
				m_arrDisplacements[3 * *iter + 1] += displaceMagnitude * displace.y;
				m_arrDisplacements[3 * *iter + 2] += displaceMagnitude * displace.z;

				// generate new layers
				lastLayerVertices.insert(*iter);
				affectedVertices.insert(*iter);
			}
		}
	}

	//Compute external forces to be applied. Input: displacements, Output: forces
	memset(m_arrExtForces, 0, sizeof(double) * m_dof);
	m_lpDeformableForceModel->GetInternalForce(m_arrDisplacements, m_arrExtForces);

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

/*!
 * Draw deformable model
 */
void Deformable::draw()
{
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

	glStencilFunc(GL_ALWAYS, 0, ~(0u));
	// render embedded triangle mesh
	//if (renderSecondaryDeformableObject)
		//secondaryDeformableObjectRenderingMesh->Render();

	glStencilFunc(GL_ALWAYS, 1, ~(0u));

	// render the main deformable object (surface of volumetric mesh)
	{
		{
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glEnable(GL_BLEND);

			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(1.0, 1.0);
			glDrawBuffer(GL_NONE);
			m_lpDeformableMesh->Render();
			glDisable(GL_POLYGON_OFFSET_FILL);
			glDrawBuffer(GL_BACK);
			glEnable(GL_LIGHTING);
			glDisable(GL_BLEND);
		}

		glColor3f(0.0, 0.0, 0.0);
		m_lpDeformableMesh->Render();

		//Vertices
		if(m_bRenderVertices)
		{
			glDisable(GL_LIGHTING);
			glColor3f(0.5, 0, 0);
			glPointSize(8.0);
			m_lpDeformableMesh->RenderVertices();
			glEnable(GL_LIGHTING);
		}
	}



	// render any extra scene geometry
	glStencilFunc(GL_ALWAYS, 0, ~(0u));

	/*
	double groundPlaneHeight = 1.0;
	double groundPlaneLightHeight = 1.0;
	double ground[4] = { 0, 1, 0, -groundPlaneHeight - 0.01 };
	double light[4] = { 0, groundPlaneLightHeight, 0, 1 };
	*/

	glDisable(GL_TEXTURE_2D);

	// render shadow
	{

		/*
		glColor3f(0.1, 0.1, 0.1);
		glDisable(GL_LIGHTING);
		m_lpDeformableMesh->RenderShadow(ground, light);

		glEnable(GL_LIGHTING);
		*/
	}

	glDisable(GL_LIGHTING);

	glStencilFunc(GL_ALWAYS, 1, ~(0u));
	glColor3f(0, 0, 0);
	m_lpDeformableMesh->RenderEdges();

	// disable stencil buffer modifications
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

	glColor3f(0, 0, 0);

	// render model fixed vertices
	if (m_bRenderFixedVertices) {
		for (int i = 0; i < (int)m_vFixedVertices.size(); i++) {
			glColor3f(1, 0, 0);
			double fixedVertexPos[3];
			m_lpDeformableMesh->GetSingleVertexRestPosition(
					m_vFixedVertices[i],
					&fixedVertexPos[0],
					&fixedVertexPos[1],
					&fixedVertexPos[2]);

			glEnable(GL_POLYGON_OFFSET_POINT);
			glPolygonOffset(-1.0, -1.0);
			glPointSize(12.0);
			glBegin(GL_POINTS);
			glVertex3f(fixedVertexPos[0], fixedVertexPos[1], fixedVertexPos[2]);
			glEnd();
			glDisable(GL_POLYGON_OFFSET_FILL);
		}
	}

	// render the currently pulled vertex
	if (m_idxPulledVertex >= 0) {
		glColor3f(0, 1, 0);
		double pulledVertexPos[3];
		m_lpDeformableMesh->GetSingleVertexPositionFromBuffer(
				m_idxPulledVertex,
				&pulledVertexPos[0],
				&pulledVertexPos[1],
				&pulledVertexPos[2]);

		glEnable(GL_POLYGON_OFFSET_POINT);
		glPolygonOffset(-1.0, -1.0);
		glPointSize(8.0);
		glBegin(GL_POINTS);
		glVertex3f(pulledVertexPos[0], pulledVertexPos[1], pulledVertexPos[2]);
		glEnd();
		glDisable(GL_POLYGON_OFFSET_FILL);
	}


/*
	m_lpDeformableMesh->Render();

	glStencilFunc(GL_ALWAYS, 1, ~(0u));
	glColor3f(0, 0, 0);
	//if (renderWireframe)
	m_lpDeformableMesh->RenderEdges();

	// disable stencil buffer modifications
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);


	glPushAttrib(GL_ALL_ATTRIB_BITS);
	{
		glDisable(GL_LIGHTING);
		glColor3f(0.5, 0, 0);
		glPointSize(8.0);

		m_lpDeformableMesh->RenderVertices();
		glEnable(GL_LIGHTING);
	}
	glPopAttrib();

	//Render the currently pulled vertex
	if (m_idxPulledVertex >= 0)
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3f(0, 1, 0);
		double pulledVertexPos[3];
		m_lpDeformableMesh->GetSingleVertexPositionFromBuffer(m_idxPulledVertex,
				&pulledVertexPos[0], &pulledVertexPos[1], &pulledVertexPos[2]);

		glEnable(GL_POLYGON_OFFSET_POINT);
		glPolygonOffset(-1.0, -1.0);
		glPointSize(8.0);
		glBegin(GL_POINTS);
		glVertex3f(pulledVertexPos[0], pulledVertexPos[1], pulledVertexPos[2]);
		glEnd();
		glDisable(GL_POLYGON_OFFSET_FILL);

		glPopAttrib();
	}

	// render model fixed vertices
	if (m_bRenderFixedVertices) {
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		for (int i = 0; i < m_vFixedVertices.size(); i++)
		{
			glColor3f(1, 0, 0);
			double fixedVertexPos[3];
			m_lpDeformableMesh->GetSingleVertexRestPosition(m_vFixedVertices[i],
					&fixedVertexPos[0], &fixedVertexPos[1], &fixedVertexPos[2]);

			glEnable(GL_POLYGON_OFFSET_POINT);
			glPolygonOffset(-1.0, -1.0);
			glPointSize(12.0);
			glBegin(GL_POINTS);
			glVertex3f(fixedVertexPos[0], fixedVertexPos[1], fixedVertexPos[2]);
			glEnd();
			glDisable(GL_POLYGON_OFFSET_FILL);
		}

		glPopAttrib();
	}
*/
}
