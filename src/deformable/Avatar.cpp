/*
 * Avatar.cpp
 *
 *  Created on: Oct 14, 2012
 *      Author: pourya
 */
#include "Avatar.h"
#include "base/Vec.h"

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

void AvatarCube::translate(const vec3f& delta, const vec3f& pos) {

	if(m_lpTissue == NULL)
		return;

	//Avatar corners
	AABB aabbAvatar = this->aabb();
    aabbAvatar.transform(m_spTransform->forward());
    m_aabbAvatar = aabbAvatar;

	//1.If boxes donot intersect return
	if (!m_lpTissue->aabb().intersect(aabbAvatar)) {
		return;
	}

	printf("Collision!\n");
}

void AvatarCube::draw() {

	glDisable(GL_LIGHTING);
	DrawAABB(m_aabbAvatar, vec3f(1,0,0));

	if(m_lpTissue)
		DrawAABB(m_lpTissue->aabb(), vec3f(0,1,0));

	glEnable(GL_LIGHTING);
/*
 *
	m_spTransform->bind();
    SGMesh::draw();

    //WireFrame
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glColor3f(0,0,0);
    	m_outline.draw();
    glEnable(GL_LIGHTING);
    glPopAttrib();
    m_spTransform->unbind();
    */
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



