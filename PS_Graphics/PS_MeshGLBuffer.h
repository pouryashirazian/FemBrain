/*
 * PS_MeshGLBuffer.h
 *
 *  Created on: Sep 29, 2012
 *      Author: pourya
 */

#ifndef PS_MESHGLBUFFER_H_
#define PS_MESHGLBUFFER_H_

#include "../PS_Base/PS_MathBase.h"

//Vertex Buffer Objects To Draw
class GLMeshBuffer{
public:
	GLMeshBuffer();
	virtual ~GLMeshBuffer();

	//Releases all buffer objects for rendering
	void cleanup();

	//Draw
	void draw(bool bWireFrameMode);
public:
	//Index to the buffer objects
	U32 vboVertex;
	U32 vboColor;
	U32 vboNormal;
	U32 iboFaces;

	//Count
	U32 ctTriangles;
	U32 ctVertices;

	bool bIsValid;

};


#endif /* PS_MESHGLBUFFER_H_ */
