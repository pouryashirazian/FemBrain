/*
 * CLMeshBuffer.cpp
 *
 *  Created on: Apr 14, 2013
 *      Author: pourya
 */
#include "CLMeshBuffer.h"
#include "PS_DebugUtils.h"
#include "PS_Mesh.h"

using namespace PS;
using namespace PS::MESH;
namespace PS {
namespace HPC {


bool CLMeshBuffer::ReadbackMeshVertexAttribCL(ComputeDevice* lpDevice,
												const GLMeshBuffer* lpBuffer,
												VertexAttribType attrib,
												U32& count, U32& fstep,
												vector<float>& values)  {
	if(lpDevice == NULL || lpBuffer == NULL)
		return false;

	if(!lpBuffer->isVertexBufferValid(attrib))
		return false;

	count = lpBuffer->countVertices();
	fstep = lpBuffer->vertexAttribStep(attrib);

	//Vertices
	U32 szVertexBuffer = sizeof(float) * fstep * count;
	values.resize(fstep * count);

	//Vertices
	cl_mem outMemMeshVertices = lpDevice->createMemBufferFromGL(lpBuffer->vertexBufferObjectGL(attrib), ComputeDevice::memReadOnly);
	lpDevice->enqueueAcquireGLObject(1, &outMemMeshVertices);
	lpDevice->enqueueReadBuffer(outMemMeshVertices, szVertexBuffer, &values[0]);
	lpDevice->enqueueReleaseGLObject(1, &outMemMeshVertices);
	lpDevice->finishAllCommands();


	clReleaseMemObject(outMemMeshVertices);

	return true;
}

bool CLMeshBuffer::ReadbackMeshFaceCL(ComputeDevice* lpDevice,
									   const GLMeshBuffer* lpBuffer,
									   U32& count, U32& istep,
									   vector<U32>& elements)  {

	if(lpDevice == NULL || lpBuffer == NULL)
		return false;

	if(!lpBuffer->isFaceBufferValid())
		return false;

	//faces
	count = lpBuffer->countTriangles();
	istep = lpBuffer->faceStep();

	elements.resize(istep * count);

	//Vertices
	cl_mem outMemMeshFaces = lpDevice->createMemBufferFromGL(lpBuffer->indexBufferObjectGL(), ComputeDevice::memReadOnly);
	lpDevice->enqueueAcquireGLObject(1, &outMemMeshFaces);
	lpDevice->enqueueReadBuffer(outMemMeshFaces, sizeof(U32) * istep * count, &elements[0]);
	lpDevice->enqueueReleaseGLObject(1, &outMemMeshFaces);
	lpDevice->finishAllCommands();

	clReleaseMemObject(outMemMeshFaces);

	return true;


}

bool CLMeshBuffer::CopyMeshBufferCL(ComputeDevice* lpDevice,
									 	 const GLMeshBuffer* lpSource,
									 	 GLMeshBuffer* lpDest) {
	if(lpDevice == NULL || lpSource == NULL)
		return false;

	int ctCompleted = 0;

	//Copy AABB
	lpDest->setBBox(lpSource->bbox());

	//1.Vertex and other attribs
	U32 ctVertices, fstep;
	vector<float> values;
	VertexAttribType attribs[] = {vatPosition, vatColor, vatNormal, vatTexCoord};
	int ctAttribs = sizeof(attribs) / sizeof(attribs[0]);
	for(int i=0; i<ctAttribs; i++) {
		if(lpSource->isVertexBufferValid(attribs[i])) {
			//Readback the attributes
			if(ReadbackMeshVertexAttribCL(lpDevice, lpSource, attribs[i], ctVertices, fstep, values)) {
				lpDest->setupVertexAttribs(values, fstep, attribs[i]);
				ctCompleted ++;
			}
		}
	}


	//Readback indices
	U32 ctIndices, istep;
	vector<U32> faceElements;
	if(ReadbackMeshFaceCL(lpDevice, lpSource, ctIndices, istep, faceElements)) {
		lpDest->setupIndexBufferObject(faceElements, lpSource->getFaceMode());
		ctCompleted ++;
	}

	return (ctCompleted > 0);
}

bool CLMeshBuffer::StoreAsObjMesh(const char* chrFilePath,
										ComputeDevice* lpDevice,
										const GLMeshBuffer* lpBuffer) {
	if(lpDevice == NULL || lpBuffer == NULL)
		return false;

	int ctCompleted = 0;

	Mesh* lpMesh = new Mesh();
	MeshNode* lpNode = new MeshNode();

	//1.Vertex and other attribs
	U32 ctVertices, fstep;
	vector<float> values;
	VertexAttribType attribs[] = {vatPosition, vatColor, vatNormal, vatTexCoord};
	int ctAttribs = sizeof(attribs) / sizeof(attribs[0]);
	for(int i=0; i<ctAttribs; i++) {
		if(lpBuffer->isVertexBufferValid(attribs[i])) {
			//Readback the attributes
			if(ReadbackMeshVertexAttribCL(lpDevice, lpBuffer, attribs[i], ctVertices, fstep, values)) {

				//Add attribs to meshnode
				lpNode->setVertexAttrib(values, attribs[i], fstep);
				ctCompleted ++;
			}
		}
	}


	//Readback indices
	U32 ctIndices, istep;
	vector<U32> elements;
	if(ReadbackMeshFaceCL(lpDevice, lpBuffer, ctIndices, istep, elements)) {

		lpNode->setFaceIndices(elements, lpBuffer->faceStep());
		ctCompleted ++;
	}

	//Add Mesh Node and save to file
	lpMesh->addNode(lpNode);
	lpMesh->store(chrFilePath);

	return (ctCompleted > 0);


}


GLMeshBuffer* CLMeshBuffer::PrepareMeshBufferNormals(ComputeDevice* lpDevice, GLMeshBuffer* lpBuffer, float len)
{
	if(lpDevice == NULL || lpBuffer == NULL)
		return NULL;

	U32 ctVertices;
	U32 fstepVertex, fstepNormal;
	vector<float> vertices;
	vector<float> normals;
	ReadbackMeshVertexAttribCL(lpDevice, lpBuffer, vatPosition, ctVertices, fstepVertex, vertices);
	ReadbackMeshVertexAttribCL(lpDevice, lpBuffer, vatNormal, ctVertices, fstepNormal, normals);

	//Produce new MeshBuffer
	vector<float> allvertices;
	allvertices.resize(ctVertices* 3 * 2);
	for(U32 i=0; i < ctVertices; i++) {
		vec3f ptStart = vec3f(&vertices[i * fstepVertex]);
		vec3f ptEnd = ptStart + vec3f(&normals[i * fstepNormal]) * len;
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

}
}
