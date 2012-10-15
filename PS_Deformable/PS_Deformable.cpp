#include "PS_Deformable.h"
#include "../PS_Base/PS_Logger.h"
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
	m_bHapticForceInProgress = false;


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
	int positiveDefiniteSolver = 0;

	//Copy from the input fixed vertices
	m_vFixedVertices.assign(vFixedVertices.begin(), vFixedVertices.end());
	//(constrained DOFs are specified 0-indexed):
	//DISC
/*
	m_vFixedVertices.push_back(2);
	m_vFixedVertices.push_back(3);
	m_vFixedVertices.push_back(4);
	m_vFixedVertices.push_back(5);

	m_vFixedVertices.push_back(34);
	m_vFixedVertices.push_back(35);
	m_vFixedVertices.push_back(36);
	m_vFixedVertices.push_back(37);

*/

	/*
	//BEAM
	m_vFixedVertices.push_back(206);
	m_vFixedVertices.push_back(207);
	m_vFixedVertices.push_back(154);
	m_vFixedVertices.push_back(155);

	m_vFixedVertices.push_back(102);
	m_vFixedVertices.push_back(103);
	m_vFixedVertices.push_back(50);
	m_vFixedVertices.push_back(51);
	*/

	vector<int> vConstainedDofs;
	computeConstrainedDof(vConstainedDofs);

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

	// initialize the integrator
	m_lpIntegrator = new ImplicitBackwardEulerSparse(m_dof, m_timeStep,
													 m_lpMassMatrix,
													 m_lpDeformableForceModel,
													 positiveDefiniteSolver,
													 vConstainedDofs.size(),
													 &vConstainedDofs[0],
													 m_dampingMassCoeff,
													 m_dampingStiffnessCoeff);


	m_ctTimeStep = 0;
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
void Deformable::computeConstrainedDof(std::vector<int>& vArrFixedDof)
{
	if(m_vFixedVertices.size() == 0)
		return;

	//Fixed Vertices
	std::sort(m_vFixedVertices.begin(), m_vFixedVertices.end());

	//4,10,14
	vArrFixedDof.resize(m_vFixedVertices.size() * 3);
	for(U32 i=0; i < m_vFixedVertices.size(); i++)
	{
		int start = m_vFixedVertices[i] * 3;
		vArrFixedDof[i*3] = start;
		vArrFixedDof[i*3 + 1] = start + 1;
		vArrFixedDof[i*3 + 2] = start + 2;
	}
}


//TimeStep the animation
void Deformable::timestep()
{
	// important: must always clear forces, as they remain in effect unless changed
	m_lpIntegrator->SetExternalForcesToZero();

	this->hapticUpdate();

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

	//Time Step
	m_lpIntegrator->DoTimestep();

	//Increment time step
	m_ctTimeStep++;

	m_lpIntegrator->GetqState(m_arrDisplacements);

	m_lpDeformableMesh->SetVertexDeformations(m_arrDisplacements);

	glutPostRedisplay();
}

bool Deformable::pickFreeVertex(double worldX, double worldY, double worldZ)
{
	Vec3d wPos(worldX, worldY, worldZ);
	m_idxPulledVertex = m_lpDeformableMesh->FindClosestVertex(wPos);
	LogInfoArg1("Clicked on vertex: %d (0-indexed).", m_idxPulledVertex);

	if(m_vFixedVertices.size() > 0)
	{
		for(U32 i=0; i < m_vFixedVertices.size(); i++)
		{
			if(m_vFixedVertices[i] == m_idxPulledVertex)
				return false;
		}
	}

	m_bHapticForceInProgress = true;
	return true;
}

void Deformable::hapticStart()
{
	m_bHapticForceInProgress = true;
}

void Deformable::hapticEnd()
{
	m_bHapticForceInProgress = false;
	m_idxPulledVertex = -1;
}

void Deformable::hapticSetCurrentForce(double extForce[3])
{
	for(int i=0; i<3; i++)
		m_hapticExtForce[i] = extForce[i];
}

//Apply External Forces to the integrator
bool Deformable::hapticUpdate()
{
	if (m_idxPulledVertex < 0)
		return false;
	if (!m_bHapticForceInProgress)
		return false;

	double extForce[3];
	for (int j = 0; j < 3; j++)
		extForce[j] = m_hapticCompliance * m_hapticExtForce[j];

	//LogInfoArg3()("%d fx: %G fy: %G | %G %G %G\n", m_idxPulledVertex, forceX, forceY, externalForce[0], externalForce[1], externalForce[2]);

	//Reset External Force
	memset(m_arrExtForces, 0, sizeof(double) * m_dof);

	//Register force on the pulled vertex
	m_arrExtForces[3 * m_idxPulledVertex + 0] += extForce[0];
	m_arrExtForces[3 * m_idxPulledVertex + 1] += extForce[1];
	m_arrExtForces[3 * m_idxPulledVertex + 2] += extForce[2];

	// distribute force over the neighboring vertices
	set<int> affectedVertices;
	set<int> lastLayerVertices;
	affectedVertices.insert(m_idxPulledVertex);
	lastLayerVertices.insert(m_idxPulledVertex);

	for (int j = 1; j < m_hapticForceNeighorhoodSize; j++)
	{
		// linear kernel
		double forceMagnitude = 1.0 * (m_hapticForceNeighorhoodSize - j)
				/ static_cast<double>(m_hapticForceNeighorhoodSize);

		set<int> newAffectedVertices;
		for (set<int>::iterator iter = lastLayerVertices.begin(); iter != lastLayerVertices.end(); iter++)
		{
			// traverse all neighbors and check if they were already previously inserted
			int vtx = *iter;
			int deg = m_lpMeshGraph->GetNumNeighbors(vtx);
			for (int k = 0; k < deg; k++)
			{
				int vtxNeighbor = m_lpMeshGraph->GetNeighbor(vtx, k);

				if (affectedVertices.find(vtxNeighbor) == affectedVertices.end())
				{
					// discovered new vertex
					newAffectedVertices.insert(vtxNeighbor);
				}
			}
		}

		lastLayerVertices.clear();
		for (set<int>::iterator iter = newAffectedVertices.begin();iter != newAffectedVertices.end(); iter++)
		{
			// apply force
			m_arrExtForces[3 * *iter + 0] += forceMagnitude * extForce[0];
			m_arrExtForces[3 * *iter + 1] += forceMagnitude * extForce[1];
			m_arrExtForces[3 * *iter + 2] += forceMagnitude * extForce[2];

			// generate new layers
			lastLayerVertices.insert(*iter);
			affectedVertices.insert(*iter);
		}
	}

 // apply any scripted force loads
/*
 if (timestepCounter < numForceLoads)
 {
 printf("  External forces read from the binary input file.\n");
 for(int i=0; i<3*n; i++)
 f_ext[i] += forceLoads[ELT(3*n, i, timestepCounter)];
 }
 */

	// set forces to the integrator
	m_lpIntegrator->SetExternalForces(m_arrExtForces);

	return true;
}

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
		for (int i = 0; i < m_vFixedVertices.size(); i++) {
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
