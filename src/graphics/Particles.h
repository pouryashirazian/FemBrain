#ifndef PS_PARTICLES_H
#define PS_PARTICLES_H

#include <vector>
#include "CLManager.h"
#include "SGMesh.h"

using namespace PS;
using namespace PS::CL;

namespace PS {
namespace SG {

/*!
* A Particles system which runs on the GPU with OpenCL.
*/ 
class Particles : public SGMesh
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
