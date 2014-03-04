/*
 * MassSpringSystem.cpp
 *
 *  Created on: Jun 1, 2013
 *      Author: pourya
 */
#include "MassSpringSystem.h"

#define DEFAULT_ROW_COUNT 64
#define DEFAULT_COL_COUNT 64
#define DEFAULT_KSPRING 1.0f
#define DEFAULT_KDAMPER 0.3f

#define TIMESTEP 1000.0f/60.0f

MassPoint::MassPoint() {
	setup(vec3f(-1, -1, -1), vec3f(1, 1, 1), vec4f(1, 0, 0, 1));
}

MassPoint::MassPoint(float side, const vec3f& center, const vec4f& color) {
	float hs = side * 0.5f;
	setup(center - vec3f(hs, hs, hs), center + vec3f(hs, hs, hs), color);
}

MassPoint::~MassPoint() {
	GLMeshBuffer::cleanup();
}

void MassPoint::setup(const vec3f& lo, const vec3f& hi, const vec4f& color)
{
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
		vec3f n(&normals[i][0]);
		n.normalize();
		arrNormals[i*3 + 0] = n.x;
		arrNormals[i*3 + 1] = n.y;
		arrNormals[i*3 + 2] = n.z;
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

	//Setup Buffers
	setupVertexAttribs(arrVertices, 3, mbtPosition);
	setupVertexAttribs(arrNormals, 3, mbtNormal);
	setupPerVertexColor(color, ctVertices, 4);
	setupIndexBufferObject(arrIndices, ftQuads);

}

void MassPoint::draw() {
	glPushMatrix();

	if(m_spTransform)
		glMultMatrixf(m_spTransform->forward().cptr());
	GLMeshBuffer::draw();


	glPopMatrix();
}

//Dumble
SpringDumble::SpringDumble() {
	setup();
}

SpringDumble::~SpringDumble() {
	SAFE_DELETE(m_lpMassTop);
	SAFE_DELETE(m_lpMassBottom);
}

void SpringDumble::setup() {
	m_top = vec3f(0, 2, 0);
	m_bottom = vec3f(0, 0, 0);
	m_restLen = vec3f::distance(m_top, m_bottom);
	m_lpMassTop = new MassPoint(1.0f, vec3f(0, 0, 0), vec4f(0, 1, 0, 1));
	m_lpMassTop->resetTransform();
	m_lpMassTop->transform()->scale(vec3f(0.3, 0.3, 0.3));
	m_lpMassTop->transform()->translate(m_top);

	m_lpMassBottom = new MassPoint(1.0f, vec3f(0, 0, 0), vec4f(1, 0, 0, 1));
	m_lpMassBottom->resetTransform();
	m_lpMassBottom->transform()->scale(vec3f(0.3, 0.3, 0.3));
	//m_lpMassBottom->transform()->translate(vec3f(0, 1, 0));
	m_kspring = DEFAULT_KSPRING;
	m_kdamper = DEFAULT_KDAMPER;
	m_v1 = m_v2 = 0;
	m_massTop = 1.0f;
	m_massBottom = 1.0f;
}

void SpringDumble::draw() {
	m_lpMassTop->draw();
	m_lpMassBottom->draw();
}

void SpringDumble::timestep() {

	float dt = static_cast<float>(TIMESTEP) / 1000.0f;
	float len = (m_top - m_bottom).length();
	if(len > 2.0f)
		len = len + 1 - 1;
	float vs = (m_restLen - len) / dt;

	float fs = m_kspring * (m_restLen - len) + m_kdamper * vs;
	float acc = fs / m_massBottom;
	m_v2 = acc * dt;

	m_bottom = m_bottom + vec3f(0, 1, 0) * 0.5 * (m_v1 + m_v2) * dt;
	m_lpMassBottom->transform()->translate(m_bottom);

	m_v1 = m_v2;
}

void SpringDumble::force() {
	m_bottom = m_bottom - vec3f(0, 0.2, 0);
	m_lpMassBottom->transform()->translate(m_bottom);
}

//Textile
Textile::Textile() {
	m_nRows = DEFAULT_ROW_COUNT;
	m_nCols = DEFAULT_COL_COUNT;
	m_kspring = DEFAULT_KSPRING;
	m_kdamper = DEFAULT_KDAMPER;
	init();
}

Textile::Textile(U32 nRows, U32 nCols, float kspring, float kdamper) :
		m_nRows(nRows), m_nCols(nCols), m_kspring(kspring), m_kdamper(kdamper)
{
	init();
}

Textile::~Textile() {
	SAFE_DELETE(m_lpGPU);
}

void Textile::init() {
	m_lpGPU = new ComputeDevice(ComputeDevice::dtGPU, true, false, "AMD");
}

void Textile::timestep() {

}

void Textile::draw() {

}




