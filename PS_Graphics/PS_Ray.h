#pragma once
#ifndef CRAY
#define CRAY

#include "PS_Vector.h"

using namespace PS::MATH;

namespace PS{
namespace MATH{

//Used optimized ray implemented in http://www.cs.utah.edu/~awilliam/box/box.pdf
class Ray
{
public:
	vec3f start;
	vec3f direction;
	vec3f inv_direction;
	int	  sign[3];

public:
	Ray() 	{
		start = vec3f(0,0,0);
		direction = vec3f(1,0,0);
	}

	Ray(const vec3f& s, const vec3f& dir)
	{
		set(s, dir);
	}

	Ray(const vec3d& s, const vec3d& dir)
	{
		set(s, dir);
	}

	vec3f point(float t) const
	{
		vec3f res = start + direction * t;
		return res;
	}

	void set(const vec3d& s, const vec3d& dir)
	{
		this->set(vec3f(s.x, s.y, s.z), vec3f(dir.x, dir.y, dir.z));
	}

	void set(const vec3f& s, const vec3f& dir)
	{
		start = s;
		direction = dir;
		inv_direction = vec3f(1/dir.x, 1/dir.y, 1/dir.z);
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
