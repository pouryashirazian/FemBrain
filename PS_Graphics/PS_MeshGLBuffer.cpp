/*
 * PS_MeshGLBuffer.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: pourya
 */

#include "PS_MeshGLBuffer.h"
#include "GL/glew.h"

GLMeshBuffer::GLMeshBuffer()
{
	bIsValid = false;
}

GLMeshBuffer::~GLMeshBuffer()
{
	cleanup();
}

void GLMeshBuffer::cleanup()
{
	if(bIsValid)
	{
		glDeleteBuffers(1, &vboVertex);
		glDeleteBuffers(1, &vboColor);
		glDeleteBuffers(1, &vboNormal);
		glDeleteBuffers(1, &iboFaces);
		bIsValid = false;
	}
}

void GLMeshBuffer::draw(bool bWireFrameMode)
{
	if(!this->bIsValid)	 return;

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	if(bWireFrameMode)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//Color
	glBindBuffer(GL_ARRAY_BUFFER, vboColor);
	glColorPointer(4, GL_FLOAT, 0, 0);
	glEnableClientState(GL_COLOR_ARRAY);

	//Normal
	glBindBuffer(GL_ARRAY_BUFFER, vboNormal);
	glNormalPointer(GL_FLOAT, 0, 0);
	glEnableClientState(GL_NORMAL_ARRAY);

	//Vertex
	glBindBuffer(GL_ARRAY_BUFFER, vboVertex);
	glVertexPointer(4, GL_FLOAT, 0, 0);
	glEnableClientState(GL_VERTEX_ARRAY);

	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshBuffers.iboFaces);
	//glEnableClientState(GL_ELEMENT_ARRAY_BUFFER);
	//glDrawElements(GL_TRIANGLES, (GLsizei)meshBuffers.ctTriangles * 3, GL_UNSIGNED_INT, (GLvoid*)0);
	glDrawArrays(GL_TRIANGLES, 0, ctVertices);

	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	//glDisableClientState(GL_ELEMENT_ARRAY_BUFFER);

	glPopAttrib();
}
