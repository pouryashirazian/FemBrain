/*
 * MeshRenderer.h
 *
 *  Created on: Apr 9, 2013
 *      Author: pourya
 */

#ifndef MESHRENDERER_H_
#define MESHRENDERER_H_

#include "PS_Graphics/PS_Mesh.h"
#include "PS_Graphics/PS_GLMeshBuffer.h"

using namespace PS::MESH;

class MeshRenderer : public GLMeshBuffer {
public:
	MeshRenderer();
	MeshRenderer(const string& strObjMeshFP);
	MeshRenderer(const Mesh* aMesh, int iNode);
	virtual ~MeshRenderer();

	bool setup(const Mesh* aMesh, int iNode);
};


#endif /* MESHRENDERER_H_ */
