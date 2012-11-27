/*
 * OclRayTracer.h
 *
 *  Created on: Nov 24, 2012
 *      Author: pourya
 */

#ifndef OCLRAYTRACER_H_
#define OCLRAYTRACER_H_

#include "PS_ComputeDevice.h"
#include "PS_GLSurface.h"
#include "PS_Vector.h"
#include "PS_Matrix.h"

using namespace PS::MATH;

namespace PS{
namespace HPC{

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
