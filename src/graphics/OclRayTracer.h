/*
 * OclRayTracer.h
 *
 *  Created on: Nov 24, 2012
 *      Author: pourya
 */

#ifndef OCLRAYTRACER_H_
#define OCLRAYTRACER_H_

#include "ComputeDevice.h"
#include "GLSurface.h"
#include "base/Vec.h"
#include "base/Matrix.h"

using namespace PS::MATH;
using namespace PS::GL;

namespace PS {
namespace CL {

class RayTracer{
public:
	RayTracer();
	RayTracer(int w, int h);
	virtual ~RayTracer();

	bool run();


private:
	void cleanup();
	void setup(int w, int h);

private:
	ComputeDevice* m_lpDevice;
	ComputeKernel* m_lpKernelPrimary;
	vec2i m_screenDim;

	GLSurface* m_lpSurface;

};

}
}



#endif /* OCLRAYTRACER_H_ */
