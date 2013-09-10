#ifndef PS_PARTICLES_H
#define PS_PARTICLES_H

#include "PS_ComputeDevice.h"
#include "PS_GLMeshBuffer.h"
#include <vector>

namespace PS{
namespace HPC{


/*!
* A Particles system which runs on the GPU with OpenCL.
*/ 
class Particles : public GLMeshBuffer
{
public:
	/*!
	 * Ctor
	 * @param maxParticles max number of particles to generate
	 */
	Particles(const U32 maxParticles = 1024);
	virtual ~Particles();


	void timestep();

private: 
	void cleanup();
	bool setup();
private:
	ComputeDevice* m_lpDevice;
	ComputeKernel* m_lpKernelMove;
	U32 m_maxParticles;
	std::vector<float> m_vPos;
	std::vector<float> m_vVel;
	std::vector<float> m_vColor;

	cl_mem m_inoutMemVelocity;
};

}
}

#endif
