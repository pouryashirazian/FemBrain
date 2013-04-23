/*
 * MeshRenderer.cpp
 *
 *  Created on: Apr 9, 2013
 *      Author: pourya
 */


#include "MeshRenderer.h"
#include "PS_Base/PS_Logger.h"

MeshRenderer::MeshRenderer() {

}

MeshRenderer::MeshRenderer(const string& strObjMeshFP) {
	LogInfoArg1("Loading mesh file: %s", strObjMeshFP.c_str());

	Mesh* aMesh = new Mesh(strObjMeshFP.c_str());
	assert(aMesh != NULL);
	this->setup(aMesh, 0);
	SAFE_DELETE(aMesh);
}

MeshRenderer::MeshRenderer(const Mesh* aMesh, int iNode) {
	this->setup(aMesh, iNode);
}

MeshRenderer::~MeshRenderer() {

}

bool MeshRenderer::setup(const Mesh* aMesh, int iNode) {
	if(aMesh == NULL)
		return false;

	if(iNode < 0 || iNode >= (int)aMesh->countNodes())
		return false;

	//Get Node
	MeshNode* aNode = aMesh->getNode(iNode);

	if(aNode->countVertices() > 0)
		this->setupVertexAttribs(aNode->vertices(), aNode->getUnitVertex(), vatPosition);

	if(aNode->countNormals() > 0)
		this->setupVertexAttribs(aNode->normals(), aNode->getUnitNormal(), vatNormal);

	if(aNode->countFaces() > 0)
		this->setupIndexBufferObject(aNode->faceElements(), ftTriangles);

	return true;
}





