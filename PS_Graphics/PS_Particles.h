#ifndef PS_PARTICLES_H
#define PS_PARTICLES_H

#include "PS_ComputeDevice.h"
#include "../PS_Base/PS_MathBase.h"
#include "PS_VectorMath.h"
#include "PS_MATRIX4.h"
#include <vector>

using namespace PS::FUNCTIONALMATH;

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

	bool setup();
	void render();
	void animate();

private: 
	void cleanup();
private:
	ComputeDevice* m_lpDevice;
	ComputeKernel* m_lpKernelMove;
	U32 m_maxParticles;
	MESH_BUFFER_OBJECTS m_meshBO;
	std::vector<float> m_vPos;
	std::vector<float> m_vVel;
	std::vector<float> m_vColor;
};

}
}

#endif
