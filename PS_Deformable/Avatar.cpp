/*
 * Avatar.cpp
 *
 *  Created on: Oct 14, 2012
 *      Author: pourya
 */
#include "Avatar.h"
#include "../PS_Graphics/PS_Vector.h"
#include "GL/glew.h"

using namespace PS::MATH;

AvatarCube::AvatarCube()
{
	vec3d lo = vec3d(-0.5, -0.5, -0.5);
	vec3d hi = vec3d(0.5, 0.5, 0.5);

	this->setup(lo, hi);
}

AvatarCube::AvatarCube(const vec3d& lo, const vec3d& hi)
{
	setup(lo, hi);
}

AvatarCube::~AvatarCube(){
	GLMeshBuffer::cleanup();
}

void AvatarCube::setup(const vec3d& lo, const vec3d& hi)
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
	setupVertexAttribs(arrVertices, 3, vatPosition);
	setupVertexAttribs(arrNormals, 3, vatNormal);
	setupPerVertexColor(clBlue, ctVertices, 4);
	setupIndexBufferObject(arrIndices, ftQuads);
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
	setupVertexAttribs(arrVertices, 3, vatPosition);
	setupVertexAttribs(arrNormals, 3, vatNormal);
	setupPerVertexColor(clBlue, ctVertices, 4);
	setupIndexBufferObject(arrIndices, ftQuads);
}



