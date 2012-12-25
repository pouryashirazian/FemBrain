/*
 * PS_MeshGLBuffer.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: pourya
 */

#include "PS_GLMeshBuffer.h"
#include "GL/glew.h"

GLMeshBuffer::GLMeshBuffer()
{
	m_vboColor = m_vboNormal = m_vboTexCoord = m_vboVertex = m_iboFaces = INVALID_GLBUFFER;
	m_isValidColor = m_isValidIndex = m_isValidNormal = m_isValidTexCoord = m_isValidVertex = false;
	m_ctFaceElements = m_ctVertices = 0;
	m_faceMode = GL_TRIANGLES;
	m_bWireFrame = false;

	m_stepVertex = 3;
	m_stepColor = 3;
	m_stepTexCoord = 2;
	m_stepFace = 3;

	m_uShaderEffectProgram = INVALID_GLBUFFER;
}

GLMeshBuffer::~GLMeshBuffer()
{
	cleanup();
}

void GLMeshBuffer::cleanup()
{
	if(m_isValidColor)
		glDeleteBuffers(1, &m_vboColor);
	if(m_isValidTexCoord)
		glDeleteBuffers(1, &m_vboTexCoord);
	if(m_isValidVertex)
		glDeleteBuffers(1, &m_vboVertex);
	if(m_isValidNormal)
		glDeleteBuffers(1, &m_vboNormal);
	if(m_isValidIndex)
		glDeleteBuffers(1, &m_iboFaces);
	m_isValidColor = m_isValidIndex = m_isValidNormal = m_isValidTexCoord = m_isValidVertex = false;
}

void GLMeshBuffer::setupVertexAttribs(const vector<float>& arrAttribs, int step, VertexAttribType attribKind)
{
	/*
	glGenBuffers(1, &m_outputMesh.vboVertex);
	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboVertex);
	glBufferData(GL_ARRAY_BUFFER, ctVertices*3 * sizeof(float), arrVertices, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &m_outputMesh.vboColor);
	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboColor);
	glBufferData(GL_ARRAY_BUFFER, ctVertices*3 * sizeof(float), arrColors, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &m_outputMesh.vboNormal);
	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboNormal);
	glBufferData(GL_ARRAY_BUFFER, ctVertices*3 * sizeof(float), arrNormals, GL_DYNAMIC_DRAW);


	glGenBuffers(1, &m_outputMesh.iboFaces);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_outputMesh.iboFaces);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ctTriangles *3 * sizeof(U32), arrFaces, GL_DYNAMIC_DRAW);
	*/

	if(attribKind == vatPosition)
	{
		m_isValidVertex = true;
		m_stepVertex = step;
		m_ctVertices = arrAttribs.size() / m_stepVertex;
		glGenBuffers(1, &m_vboVertex);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
		glBufferData(GL_ARRAY_BUFFER, arrAttribs.size() * sizeof(float), &arrAttribs[0], GL_STATIC_DRAW);
	}
	else if(attribKind == vatNormal)
	{
		m_isValidNormal = true;
		vector<float> arrNormalizedNormals;
		arrNormalizedNormals.resize(arrAttribs.size());
		U32 ctNormals = arrAttribs.size() / 3;
		for(U32 i=0; i<ctNormals; i++)
		{
			vec3f n = vec3f(&arrAttribs[i * 3]);
			n.normalize();
			arrNormalizedNormals[i * 3 + 0] = n.x;
			arrNormalizedNormals[i * 3 + 1] = n.y;
			arrNormalizedNormals[i * 3 + 2] = n.z;
		}

		glGenBuffers(1, &m_vboNormal);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboNormal);
		glBufferData(GL_ARRAY_BUFFER, arrAttribs.size() * sizeof(float), &arrAttribs[0], GL_STATIC_DRAW);
	}
	else if(attribKind == vatColor)
	{
		m_isValidColor = true;
		m_stepColor = step;

		glGenBuffers(1, &m_vboColor);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
		glBufferData(GL_ARRAY_BUFFER, arrAttribs.size() * sizeof(float), &arrAttribs[0], GL_STATIC_DRAW);
	}
	else if(attribKind == vatTexCoord)
	{
		m_isValidTexCoord = true;
		m_stepTexCoord = step;

		glGenBuffers(1, &m_vboTexCoord);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboTexCoord);
		glBufferData(GL_ARRAY_BUFFER, arrAttribs.size() * sizeof(float), &arrAttribs[0], GL_STATIC_DRAW);
	}
}

void GLMeshBuffer::setupPerVertexColor(const vec4f& color, U32 ctVertices, int step)
{
	if(m_isValidColor)
		glDeleteBuffers(1, &m_vboColor);

	m_stepColor = step;
	vector<float> arrColors;
	arrColors.resize(ctVertices * step);
	for(U32 i=0; i<ctVertices; i++)
	{
		for(int j=0; j<step; j++)
			arrColors[i*step + j] = color.e[j];
	}

	glGenBuffers(1, &m_vboColor);
	glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
	glBufferData(GL_ARRAY_BUFFER, arrColors.size() * sizeof(float), &arrColors[0], GL_STATIC_DRAW);
	m_isValidColor = true;
}

void GLMeshBuffer::setupIndexBufferObject(const vector<U32>& arrIndex, int faceMode /*GL_TRIANGLES */)
{
	m_faceMode = faceMode;
	m_isValidIndex = true;
	m_ctFaceElements = arrIndex.size();
	glGenBuffers(1, &m_iboFaces);
	glBindBuffer(GL_ARRAY_BUFFER, m_iboFaces);
	glBufferData(GL_ARRAY_BUFFER, arrIndex.size() * sizeof(U32), &arrIndex[0], GL_STATIC_DRAW);
}

void GLMeshBuffer::setShaderEffectProgram(U32 glEffect)
{
	if(glEffect != INVALID_GLBUFFER)
	{
		m_uShaderEffectProgram = glEffect;
		m_effectType = setCustom;
	}
}


void GLMeshBuffer::draw()
{
	if(m_effectType == setCustom && m_uShaderEffectProgram != INVALID_GLBUFFER)
		glUseProgram(m_uShaderEffectProgram);

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	if(m_bWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//Color
	if(m_isValidColor)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
		glColorPointer(m_stepColor, GL_FLOAT, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);
	}

	//TexCoord
	if(m_isValidTexCoord)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vboTexCoord);
		glTexCoordPointer(m_stepTexCoord, GL_FLOAT, 0, 0);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	}

	//Normal
	if(m_isValidNormal)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vboNormal);
		glNormalPointer(GL_FLOAT, 0, 0);
		glEnableClientState(GL_NORMAL_ARRAY);
	}

	//Vertex
	if(m_isValidVertex)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
		glVertexPointer(m_stepVertex, GL_FLOAT, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);
	}


	//Draw Faces
	if(m_isValidIndex)
	{
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboFaces);
		glEnableClientState(GL_ELEMENT_ARRAY_BUFFER);
		glDrawElements(m_faceMode, (GLsizei)m_ctFaceElements, GL_UNSIGNED_INT, (GLvoid*)0);

		glDisableClientState(GL_ELEMENT_ARRAY_BUFFER);
	}
	else
		glDrawArrays(m_faceMode, 0, m_ctFaceElements);

	if(m_isValidColor)
		glDisableClientState(GL_COLOR_ARRAY);
	if(m_isValidTexCoord)
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	if(m_isValidNormal)
		glDisableClientState(GL_NORMAL_ARRAY);
	if(m_isValidVertex)
		glDisableClientState(GL_VERTEX_ARRAY);

	glPopAttrib();

	if(m_effectType == setCustom && m_uShaderEffectProgram != INVALID_GLBUFFER)
		glUseProgram(0);
}
