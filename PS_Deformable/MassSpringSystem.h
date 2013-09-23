/*
 * MassSpringSystem.h
 *
 *  Created on: Jun 1, 2013
 *      Author: pourya
 */

#ifndef MASSSPRINGSYSTEM_H_
#define MASSSPRINGSYSTEM_H_

#include <vector>
#include "../PS_Graphics/SceneGraph.h"
#include "../PS_Graphics/PS_ComputeDevice.h"
#include "../PS_Graphics/PS_GLMeshBuffer.h"

using namespace PS;
using namespace PS::HPC;

class MassPoint : public GLMeshBuffer {
public:
	MassPoint();
	MassPoint(float side, const vec3f& center, const vec4f& color);
	virtual ~MassPoint();
	void draw();

protected:
	void setup(const vec3f& lo, const vec3f& hi, const vec4f& color);
};

/*!
 * Simple Spring Test
 */
class SpringDumble : public SceneNode {
public:
	SpringDumble();
	virtual ~SpringDumble();

	void draw();
	void timestep(U64 timer);

	void force();
protected:
	void setup();

private:
	MassPoint* m_lpMassTop;
	MassPoint* m_lpMassBottom;
	float m_kspring;
	float m_kdamper;
	float m_v1;
	float m_v2;
	float m_massTop;
	float m_massBottom;
	vec3f m_top;
	vec3f m_bottom;
	float m_restLen;
};

/*!
 * Textile
 */
class Textile : public GLMeshBuffer {
public:
	Textile();
	Textile(U32 nRows, U32 nCols, float kspring = 1.0f, float kdamper = 0.01f);
	virtual ~Textile();

	//TimeStep Animation
	void timestep();

	//Draw
	void draw();

protected:
	void init();

private:
	U32 m_nRows;
	U32 m_nCols;
	float m_kspring;
	float m_kdamper;

	ComputeDevice* m_lpGPU;
	ComputeKernel* m_lpKernelMassSpring;
};


#endif /* MASSSPRINGSYSTEM_H_ */
