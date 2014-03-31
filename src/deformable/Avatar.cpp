/*
 * Avatar.cpp
 *
 *  Created on: Oct 14, 2012
 *      Author: pourya
 */
#include "Avatar.h"
#include "base/Vec.h"

#define DEFAULT_HAPTIC_FORCE_COEFF 200000

using namespace PS::MATH;
using namespace PS::SG;

AvatarCube::AvatarCube():SGMesh(), IGizmoListener()
{
	m_lpTissue = NULL;
	setup();
}

AvatarCube::AvatarCube(Deformable* tissue):SGMesh(), IGizmoListener() {
	m_lpTissue = tissue;
	setup();
}

AvatarCube::~AvatarCube(){
	SGMesh::cleanup();
}

void AvatarCube::setup()
{
	this->setName("cubeavatar");
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

vec3f AvatarCube::lower() const {
	if(m_spTransform)
		return m_spTransform->forward().map(m_lower);
	else
		return m_lower;
}

vec3f AvatarCube::upper() const {
	if(m_spTransform)
		return m_spTransform->forward().map(m_upper);
	else
		return m_upper;
}

void AvatarCube::mousePress(int button, int state, int x, int y) {
	printf("Press %d\n", state);
	//Down Start
	if(state == 0) {
		if(m_lpTissue) {
			m_lpTissue->hapticStart(0);
			TheSceneGraph::Instance().headers()->updateHeaderLine("avatar", "avatar: start haptic on tissue");
		}
	}
	else {
		//Up Stop
		if (m_lpTissue->isHapticInProgress()) {
			m_lpTissue->hapticEnd();
			m_hashVertices.clear();
			TheSceneGraph::Instance().headers()->updateHeaderLine("avatar", "avatar: ended haptic on tissue");
		}
	}
}

void AvatarCube::onTranslate(const vec3f& delta, const vec3f& pos) {

	if(m_lpTissue == NULL)
		return;

	//Avatar corners
	m_aabbCurrent = this->aabb();
	m_aabbCurrent.transform(m_spTransform->forward());
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

void AvatarCube::draw() {
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


//Avatar Scalpel
AvatarScalpel::AvatarScalpel() {

}

AvatarScalpel::AvatarScalpel(const vec3d& lo, const vec3d& hi) {

}

AvatarScalpel::~AvatarScalpel(){
	GLMeshBuffer::cleanup();
}

void AvatarScalpel::setup(const vec3d& lo, const vec3d& hi)
{
	this->m_lo = lo;
	this->m_hi = hi;

	float l = lo.x;
	float r = hi.x;
	float b = lo.y;
	float t = hi.y;
	float n = lo.z;
	float f = hi.z;

	float vertices[][3] = { { l, b, f }, { l, t, f }, { r, t, f },
							{ r, b, f }, { l, b, n }, { l, t, n }, { r, t, n }, { r, b, n } };

	float normals[][3] = { { -1, -1, 1 }, { -1, 1, 1 }, { 1, 1, 1 }, { 1, -1, 1 },
							{ -l, -1, -1 }, { -1, 1, -1 }, { 1, 1, -1 }, { 1, -1, -1 } };


	const int ctVertices = 8;
	vector<float> arrVertices;
	arrVertices.resize(24);
	for(int i=0; i < ctVertices; i++)
	{
		arrVertices[i*3 + 0] = vertices[i][0];
		arrVertices[i*3 + 1] = vertices[i][1];
		arrVertices[i*3 + 2] = vertices[i][2];
	}

	vector<float> arrNormals;
	arrNormals.resize(24);
	for(int i=0; i < ctVertices; i++)
	{
		arrNormals[i*3 + 0] = normals[i][0];
		arrNormals[i*3 + 1] = normals[i][1];
		arrNormals[i*3 + 2] = normals[i][2];
	}

	vector<U32> arrIndices;
	arrIndices.resize(24);
	arrIndices[0] = 0;
	arrIndices[1] = 3;
	arrIndices[2] = 2;
	arrIndices[3] = 1;

	arrIndices[4] = 4;
	arrIndices[5] = 5;
	arrIndices[6] = 6;
	arrIndices[7] = 7;

	arrIndices[8] = 3;
	arrIndices[9] = 0;
	arrIndices[10] = 4;
	arrIndices[11] = 7;

	arrIndices[12] = 1;
	arrIndices[13] = 2;
	arrIndices[14] = 6;
	arrIndices[15] = 5;

	arrIndices[16] = 2;
	arrIndices[17] = 3;
	arrIndices[18] = 7;
	arrIndices[19] = 6;

	arrIndices[20] = 5;
	arrIndices[21] = 4;
	arrIndices[22] = 0;
	arrIndices[23] = 1;

	const vec4f clBlue(0,0,1,1);

	//Setup Buffers
	setupVertexAttribs(arrVertices, 3, mbtPosition);
	setupVertexAttribs(arrNormals, 3, mbtNormal);
	setupPerVertexColor(clBlue, ctVertices, 4);
	setupIndexBufferObject(arrIndices, ftQuads);
}



