#ifndef AVATAR_H
#define AVATAR_H

#include "../PS_Graphics/PS_MeshGLBuffer.h"

class Avatar{
public:
	Avatar();
	virtual ~Avatar();

	void setup();

	void draw();

private:
	GLMeshBuffer m_glBuffer;

};

#endif
