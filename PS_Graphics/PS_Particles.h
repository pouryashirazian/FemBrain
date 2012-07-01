#ifndef PS_PARTICLES_H
#define PS_PARTICLES_H

#include "PS_ComputeDevice.h"
#include "PS_MathBase.h"

namespace PS{
namespace HPC{

//Vertex Buffer Objects To Draw
struct MESH_BUFFER_OBJECTS{
	//Index to the buffer objects
	U32 vboVertex;
	U32 vboColor;
	U32 vboNormal;
	U32 iboFaces;

	U32 ctTriangles;
	U32 ctVertices;

	bool bIsValid;

	//releases all buffer objects for rendering
	void cleanup();
};

/*!
* A Particles system which runs on the GPU with OpenCL.
*/ 
class Particles
{
public:
	Particles(const U32 maxParticles = 1024);
	virtual ~Particles();

	void setup();
	void render();
	void animate();

private:
	U32 m_maxParticles;
	MESH_BUFFER_OBJECTS m_meshBO;
};

}
}

#endif