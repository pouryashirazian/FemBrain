#pragma once
#ifndef CRAY
#define CRAY

#include "PS_VectorMath.h"

using namespace PS::FUNCTIONALMATH;

namespace PS{
namespace MATH{

//Used optimized ray implemented in http://www.cs.utah.edu/~awilliam/box/box.pdf
class Ray
{
public:
	svec3f start;
	svec3f direction;
	svec3f inv_direction;
	int	  sign[3];

public:
	Ray() 	{
		start = svec3f(0,0,0);
		direction = svec3f(1,0,0);
	}

	Ray(const svec3f& s, const svec3f& dir)
	{
		set(s, dir);
	}

	svec3f point(float t) const
	{
		svec3f res = vadd3f(start, vscale3f(t, direction));
		return res;
	}

	void set(const svec3f& s, const svec3f& dir)
	{
		start = s;
		direction = dir;
		inv_direction = svec3f(1/dir.x, 1/dir.y, 1/dir.z);
		sign[0] = (inv_direction.x < 0.0f);
		sign[1] = (inv_direction.y < 0.0f);
		sign[2] = (inv_direction.z < 0.0f);
	}

	Ray& operator =(const Ray& r)
	{
		start	  = r.start;
		direction = r.direction;
		inv_direction = r.inv_direction;
		sign[0] = r.sign[0];
		sign[1] = r.sign[1];
		sign[2] = r.sign[2];
		return(*this);
	}
};

}
}
#endif
