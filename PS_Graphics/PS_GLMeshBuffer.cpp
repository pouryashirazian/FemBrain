/*
 * PS_MeshGLBuffer.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: pourya
 */

#include "PS_GLMeshBuffer.h"
#include "GL/glew.h"
#include "PS_DebugUtils.h"

GLMeshBuffer::GLMeshBuffer():SceneNode()
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

bool GLMeshBuffer::readbackMeshVertexAttribGL(VertexAttribType attrib, U32& count, vector<float>& values) const {
	U32 vbo = 0;
	U32 szRead = 0;
	count = 0;
	if(attrib == vatPosition) {
		vbo = m_vboVertex;
		szRead = m_ctVertices * m_stepVertex;
	}
	else if(attrib == vatNormal) {
		vbo = m_vboNormal;
		szRead = m_ctVertices * 3;
	}
	else if(attrib == vatColor) {
		vbo = m_vboColor;
		szRead = m_ctVertices * m_stepColor;
	}
	else if(attrib == vatTexCoord) {
		vbo = m_vboTexCoord;
		szRead = m_ctVertices * m_stepTexCoord;
	}
	else
		return false;


	glBindBuffer( GL_ARRAY_BUFFER, vbo);
	float* lpBuffer = (float *) glMapBuffer( GL_ARRAY_BUFFER, GL_READ_ONLY );
	if(lpBuffer == NULL) {
		return false;
	}

	values.assign(lpBuffer, lpBuffer + szRead);
	glUnmapBuffer( GL_ARRAY_BUFFER );
	count = m_ctVertices;

	//PS::DEBUG::PrintArrayF(&values[0], 180);
	return true;
}

bool GLMeshBuffer::isVertexBufferValid(VertexAttribType attribType) const {
	switch(attribType) {
	case(vatPosition):
		return m_isValidVertex;
	case(vatNormal):
		return m_isValidNormal;
	case(vatColor):
		return m_isValidColor;
	case(vatTexCoord):
		return m_isValidTexCoord;
	default:
		return m_isValidVertex;
	}
}

//Steps
U32 GLMeshBuffer::vertexAttribStep(VertexAttribType attribType) const {
	switch(attribType) {
	case(vatPosition):
		return m_stepVertex;
	case(vatNormal):
		return 3;
	case(vatColor):
		return m_stepColor;
	case(vatTexCoord):
		return m_stepTexCoord;
	default:
		return m_stepVertex;
	}

}

//Buffer Objects
U32 GLMeshBuffer::vertexBufferObjectGL(VertexAttribType attribType) const {
	switch(attribType) {
	case(vatPosition):
		return m_vboVertex;
	case(vatNormal):
		return m_vboNormal;
	case(vatColor):
		return m_vboColor;
	case(vatTexCoord):
		return m_vboTexCoord;
	default:
		return m_vboVertex;
	}
}


bool GLMeshBuffer::readbackMeshFaceGL(U32& count, vector<U32>& elements) const {

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_iboFaces);
	U32* lpBuffer = (U32 *) glMapBuffer( GL_ELEMENT_ARRAY_BUFFER, GL_READ_ONLY );
	elements.assign(lpBuffer, lpBuffer + (m_ctFaceElements * m_stepFace));
	glUnmapBuffer( GL_ELEMENT_ARRAY_BUFFER );
	count = m_ctFaceElements;


	return true;
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

/*
GLMeshBuffer* GLMeshBuffer::prepareMeshBufferNormals(float len) const {
	if (!m_isValidVertex || !m_isValidNormal)
		return NULL;

	U32 ctVertices;
	vector<float> vertices;
	vector<float> normals;
	readbackMeshVertexAttrib(vatPosition, ctVertices, vertices);
	readbackMeshVertexAttrib(vatNormal, ctVertices, normals);

	vector<float> allvertices;
	allvertices.resize(ctVertices * 3 * 2);
	for (U32 i = 0; i < ctVertices; i++) {
		vec3f ptStart = vec3f(&vertices[i * 3]);
		vec3f ptEnd = ptStart + vec3f(&normals[i * 3]) * len;
		allvertices[i * 6] = ptStart.x;
		allvertices[i * 6 + 1] = ptStart.y;
		allvertices[i * 6 + 2] = ptStart.z;
		allvertices[i * 6 + 3] = ptEnd.x;
		allvertices[i * 6 + 4] = ptEnd.y;
		allvertices[i * 6 + 5] = ptEnd.z;
	}

	//Create scene node
	GLMeshBuffer* lpDrawNormal = new GLMeshBuffer();
	lpDrawNormal->setupPerVertexColor(vec4f(0, 0, 1, 1), ctVertices * 2, 4);
	lpDrawNormal->setupVertexAttribs(allvertices, 3, vatPosition);
	lpDrawNormal->setFaceMode(ftLines);

	return lpDrawNormal;
}
*/

GLMeshBuffer* GLMeshBuffer::PrepareMeshBufferForDrawingNormals(float len, U32 ctVertices, U32 fstep,
                                                               const vector<float>& arrVertices,
                                                               const vector<float>& arrNormals) {
    if(arrVertices.size() == 0 || arrNormals.size() == 0)
        return NULL;

    vector<float> allvertices;
    allvertices.resize(ctVertices* 3 * 2);
    for(U32 i=0; i < ctVertices; i++) {
        vec3f ptStart = vec3f(&arrVertices[i * fstep]);
        vec3f ptEnd = ptStart + vec3f(&arrNormals[i*3]) * len;
        allvertices[i*6] = ptStart.x;
        allvertices[i*6 + 1] = ptStart.y;
        allvertices[i*6 + 2] = ptStart.z;
        allvertices[i*6 + 3] = ptEnd.x;
        allvertices[i*6 + 4] = ptEnd.y;
        allvertices[i*6 + 5] = ptEnd.z;
    }

    //Create scene node
    GLMeshBuffer* lpDrawNormal = new GLMeshBuffer();
    lpDrawNormal->setupPerVertexColor(vec4f(0,0,1,1), ctVertices*2, 4);
    lpDrawNormal->setupVertexAttribs(allvertices, 3, vatPosition);
    lpDrawNormal->setFaceMode(ftLines);

    return lpDrawNormal;
}

U32 GLMeshBuffer::ConvertFloat4ToFloat3(const vector<float>& arrInFloat4, vector<float>& arrOutFloat3) {
	if(arrInFloat4.size() == 0)
		return 0;

	U32 ctAttribs = arrInFloat4.size() / 4;
	arrOutFloat3.resize(ctAttribs * 3);
	for(U32 i=0; i < ctAttribs; i++) {
		arrOutFloat3[i * 3] = arrInFloat4[i * 4];
		arrOutFloat3[i * 3 + 1] = arrInFloat4[i * 4 + 1];
		arrOutFloat3[i * 3 + 2] = arrInFloat4[i * 4 + 2];
	}

	return ctAttribs;
}

U32 GLMeshBuffer::ConvertFloat3ToFloat4(const vector<float>& arrInFloat3, vector<float>& arrOutFloat4) {
	if(arrInFloat3.size() == 0)
		return 0;

	U32 ctAttribs = arrInFloat3.size() / 3;
	arrOutFloat4.resize(ctAttribs * 4);
	for(U32 i=0; i < ctAttribs; i++) {
		arrOutFloat4[i * 4] = arrInFloat3[i * 3];
		arrOutFloat4[i * 4 + 1] = arrInFloat3[i * 3 + 1];
		arrOutFloat4[i * 4 + 2] = arrInFloat3[i * 3 + 2];
		arrOutFloat4[i * 4 + 3] = 1.0f;
	}

	return ctAttribs;
}




