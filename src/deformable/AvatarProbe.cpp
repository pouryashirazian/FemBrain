/*
 * Avatar.cpp
 *
 *  Created on: Oct 14, 2012
 *      Author: pourya
 */
#include "AvatarProbe.h"
#include "base/Vec.h"
#include "graphics/SceneGraph.h"

#define DEFAULT_HAPTIC_FORCE_COEFF 200000

using namespace PS::MATH;
using namespace PS::SG;

AvatarProbe::AvatarProbe():SGMesh(), IGizmoListener()
{
	m_lpTissue = NULL;
	setup();
}

AvatarProbe::AvatarProbe(Deformable* tissue):SGMesh(), IGizmoListener() {
	m_lpTissue = tissue;
	setup();
}

AvatarProbe::~AvatarProbe(){
	SGMesh::cleanup();
}

void AvatarProbe::setup()
{
	this->setName("probe");

	m_pickMode = false;
	m_pickedNode = -1;
	m_idxContactFace = -1;
	m_hapticForceCoeff = DEFAULT_HAPTIC_FORCE_COEFF;

	m_lower = vec3f(-0.5f);
	m_upper = vec3f(0.5f);

	//Geometry
	Geometry g;
	g.init(3, 4, 2, ftTriangles);
	g.addCube(m_lower, m_upper);
	g.addPerVertexColor(vec4f(0, 0, 1, 1), g.countVertices());
	SGMesh::setup(g);

	//Outline
	Geometry gWireframe;
	gWireframe.init(3, 4, 2, ftQuads);
	gWireframe.addCube(m_lower, m_upper);
	m_outline.setup(gWireframe);
	m_outline.setWireFrameMode(true);


	resetTransform();
	if(TheShaderManager::Instance().has("phong")) {
        m_spEffect = SmartPtrSGEffect(new SGEffect(TheShaderManager::Instance().get("phong")));
    }

	//Add a header
	TheSceneGraph::Instance().headers()->addHeaderLine("avatar", "avatar");
}

vec3f AvatarProbe::lower() const {
	if(m_spTransform)
		return m_spTransform->forward().map(m_lower);
	else
		return m_lower;
}

vec3f AvatarProbe::upper() const {
	if(m_spTransform)
		return m_spTransform->forward().map(m_upper);
	else
		return m_upper;
}

void AvatarProbe::mousePress(int button, int state, int x, int y) {
	//Down Start
	if(state == 0) {
		if(m_lpTissue) {

			if(m_pickMode) {
				vec3f expand(0.2);
				Ray ray = TheSceneGraph::Instance().screenToWorldRay(x, y);
				m_pickedNode = -1;
				for (int i = 0; i < (int) m_lpTissue->getVolMesh()->countNodes();
						i++) {
					AABB aabb;

					vec3d pos = m_lpTissue->getVolMesh()->const_nodeAt(i).pos;
					vec3f posF = vec3f((float) pos.x, (float) pos.y, (float) pos.z);
					aabb.set(posF - expand, posF + expand);
					if (aabb.intersect(ray, 0.0, FLT_MAX)) {
						m_pickedNode = i;
						break;
					}
				}

				m_lpTissue->hapticStart(m_pickedNode);
				TheSceneGraph::Instance().headers()->updateHeaderLine("avatar", "avatar: start picking node");
			}
			else {
				m_lpTissue->hapticStart(0);
				TheSceneGraph::Instance().headers()->updateHeaderLine("avatar", "avatar: start probing");
			}


		}
	}
	else {
		//Up Stop
		if (m_lpTissue->isHapticInProgress()) {
			m_lpTissue->hapticEnd();
			m_hashVertices.clear();
			TheSceneGraph::Instance().headers()->updateHeaderLine("avatar", "avatar: end probing");
		}
	}
}

void AvatarProbe::onTranslate(const vec3f& delta, const vec3f& pos) {

	if(m_lpTissue == NULL)
		return;

	//Avatar corners
	m_aabbCurrent = this->aabb();
	m_aabbCurrent.transform(m_spTransform->forward());

	if(m_pickMode && m_lpTissue->getVolMesh()->isNodeIndex(m_pickedNode)) {

		vector<int> vIndices;
		vIndices.push_back(m_pickedNode);

		vector<vec3d> vForces;
		vForces.push_back(vec3d(delta.x, delta.y, delta.z) * 10000.0);
		m_lpTissue->hapticSetCurrentForces(vIndices, vForces);

		return;
	}


	vec3f lo = m_aabbCurrent.lower();
	vec3f hi = m_aabbCurrent.upper();
	vec3d lower = vec3d(lo.x, lo.y, lo.z);
	vec3d upper = vec3d(hi.x, hi.y, hi.z);

	//1.If boxes donot intersect return
	if (!m_lpTissue->aabb().intersect(m_aabbCurrent)) {
		return;
	}

	//printf("Box collision!\n");

	//List all the vertices in the model impacted
	{
		vector<vec3d> arrVertices;
		vector<int> arrIndices;

		m_lpTissue->pickVertices(lower, upper, arrVertices, arrIndices);

		//Add new vertices
		for (U32 i = 0; i < arrIndices.size(); i++) {
			//If it does not have the vertex then add it. If we have it then the original vertex is used.
			if (m_hashVertices.find(arrIndices[i]) == m_hashVertices.end())
				m_hashVertices.insert(std::pair<int, vec3d>(arrIndices[i], arrVertices[i]));
		}
	}

	//If no vertices impacted then return. If we return here then gaps cannot be filled.
	if (m_hashVertices.size() == 0)
		return;

	//Input Arrays
	vec3d n[6];
	vec3d s[6];
	//X. Left and Right
	n[0] = vec3d(-1, 0, 0);
	n[1] = vec3d(1, 0, 0);

	//Y. Bottom and Top
	n[2] = vec3d(0, -1, 0);
	n[3] = vec3d(0, 1, 0);

	//Z. Back and Front
	n[4] = vec3d(0, 0, -1);
	n[5] = vec3d(0, 0, 1);

	//Sample Point to use
	s[0] = lower;
	s[1] = upper;
	s[2] = lower;
	s[3] = upper;
	s[4] = lower;
	s[5] = upper;

	//Detect Collision Face
	//Check against six faces of Avatar to find the intersection
	//Previously Detected Face? If no then detect now
	if (m_idxContactFace < 0) {
		//g_appSettings.hapticForceCoeff = DEFAULT_FORCE_COEFF;
		double minDot = GetMaxLimit<double>();
		int idxMin = 0;

		//Iterate over vertices in collision
		for (map<int, vec3d>::iterator it = m_hashVertices.begin();
			 it != m_hashVertices.end(); ++it) {
			vec3d p = it->second;
			for (int j = 0; j < 6; j++) {
				double dot = vec3d::dot(s[j] - p, n[j]);
				if (dot < minDot) {
					minDot = dot;
					idxMin = j;
					m_closestPoint = p;
				}

				m_idxContactFace = idxMin;
				m_contactDist = minDot;
			}
		}
	} else {
		double minDot = GetMaxLimit<double>();
		int idxFace = m_idxContactFace;
		//Iterate over vertices in collision
		for (map<int, vec3d>::iterator it = m_hashVertices.begin();
				it != m_hashVertices.end(); ++it) {
			double dot = vec3d::dot(s[idxFace] - it->second, n[idxFace]);
			if (dot < minDot) {
				minDot = dot;
				m_contactDist = minDot;
				m_closestPoint = it->second;
			}
		}
	}

	//Compute Displacement
	vector<vec3d> arrForces;
	vector<int> arrIndices;
	double dot = 0.0;
	int idxCFace = m_idxContactFace;

	for (std::map<int, vec3d>::iterator it = m_hashVertices.begin();
			it != m_hashVertices.end(); ++it) {
		vec3d v = it->second;

		//1000000
		dot = MATHMAX(vec3d::dot(s[idxCFace] - v, n[idxCFace]) * m_hapticForceCoeff, 0);

		arrIndices.push_back(it->first);
		arrForces.push_back(n[idxCFace] * dot);
	}

	{
		char buffer[1024];
		string arrFaces[] = { "LEFT", "RIGHT", "BOTTOM", "TOP", "NEAR", "FAR" };
		sprintf(buffer, "avatar: face[%d] = %s, dot = %.4f, coeff: %.2f, v# %d",
				idxCFace, arrFaces[idxCFace].c_str(), dot, m_hapticForceCoeff, (int)m_hashVertices.size());
		TheSceneGraph::Instance().headers()->updateHeaderLine("avatar", buffer);
	}
	//Apply displacements/forces to the model
	m_lpTissue->hapticSetCurrentForces(arrIndices, arrForces);
}

void AvatarProbe::draw() {
/*
	glDisable(GL_LIGHTING);
	DrawAABB(m_aabbCurrent, vec3f(1,0,0));

	if(m_lpTissue)
		DrawAABB(m_lpTissue->aabb(), vec3f(0,1,0));

	glEnable(GL_LIGHTING);
*/

	//faces
    SGMesh::draw();

    //WireFrame
    m_spTransform->bind();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glColor3f(0,0,0);
    	m_outline.draw();
    glEnable(GL_LIGHTING);
    glPopAttrib();

    m_spTransform->unbind();
}



