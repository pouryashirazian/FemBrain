/*
 * PS_RBF.h
 *
 *  Created on: Apr 2, 2013
 *      Author: pourya
 */

#ifndef PS_RBF_H_
#define PS_RBF_H_

#include "../PS_Graphics/PS_ComputeDevice.h"
#include "PS_OclPolygonizer.h"
#include "../PS_Graphics/PS_Vector.h"

using namespace PS;
using namespace PS::MATH;

#define DEFAULT_OFFSURFACE_LEN 0.05

/*
 * A BlobTree model is defined using implicit primitives and operators.
 * The model is polygonized with our high performance GPU polygonizer and
 * the the resulting polygon mesh represents the model's boundary surface or
 * iso-surface.
 * For our physically-based animation system volumetric model has to be
 * descretized to small tetrahedral elements which uses the brep mesh vertices
 * but will possibly add more surface and internal vertices to produce tetrahedral
 * connnectivity.
 *
 * When animation starts the tetrahedral elements may undergo deformations which will
 * displace their associated vertices and deform the original brep mesh. To keep an uptodate
 * implicit definition of the model we need to approximate the implicit function which defines
 * the model. Radial-basis functions will help us to compute a fast, accurate implicit definition.
 *
 * Benefits of implicit definition:
 * 1. To compute fast collision detection between Avatar and the model
 * 2. To compute the haptic feedback force in realtime.
 */

namespace PS {
namespace HPC {


class FastRBF {
public:
	FastRBF();
	FastRBF(GPUPoly* lpGPUPoly);
	virtual ~FastRBF();

	bool prepareInterpolation();
	bool testWithVoxelGridSamples();


	float fieldRBF(const vec3f& v);
	vec3f gradientRBF(const vec3f& v);

	float getOffSurfaceLen() const {return m_offSurfaceLen;}
	void setOffSurfaceLen(float len) { m_offSurfaceLen = len;}
private:
	GPUPoly* m_lpGPUPoly;
	U32 m_ctCenters;
	vector<float> m_centers;
	vector<float> m_fields;
	vector<float> m_lambda;
	float m_offSurfaceLen;

	double m_solutionError;

};

}
}
#endif /* PS_RBF_H_ */
