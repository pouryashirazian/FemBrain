/*
 * PS_Box.cpp
 *
 *  Created on: Oct 29, 2012
 *      Author: pourya
 */
#include "PS_Vector.h"
#include "PS_Ray.h"

#ifndef PS_BOX_H
#define PS_BOX_H

namespace PS{
namespace MATH{


class AABB{
public:
	AABB(){
		lower = upper = vec3(0,0,0);
	}

	AABB(const vec3& lo, const vec3& hi):lower(lo), upper(hi)
	{}

	virtual ~AABB(){}

	void set(const vec3& lo, const vec3& hi){
		lower = lo;
		upper = hi;
	}

	inline vec3 bounds(int idx) const { return (idx == 0)?lower:upper;}

    bool isValid() const { return ((lower.x < upper.x)&&(lower.y < upper.y)&&(lower.z < upper.z));}
    bool contains(const vec3& p) const;
    bool intersect(const AABB& rhs) const;
    bool intersect(const Ray& ray, float t0, float t1) const;
    AABB united(const AABB& rhs) const{
    	AABB result;
    	result.lower = vec3::minP(this->lower, rhs.lower);
    	result.upper = vec3::maxP(this->upper, rhs.upper);
    	return result;
    }

    void translate(const vec3& d){
    	lower = lower + d;
    	upper = upper + d;
    }


    vec3f lo() const {return this->lower;}
    vec3f up() const {return this->upper;}


private:
	vec3 lower;
	vec3 upper;
};


template <typename T>
bool Contains(const Vec3<T>& lo, const Vec3<T>& hi, const Vec3<T>& p)
{
	if(((p.x >= lo.x) && (p.x <= hi.x))&&
	  ((p.y >= lo.y) && (p.y <= hi.y))&&
 	  ((p.z >= lo.z) && (p.z <= hi.z)))
		return true;
	else
		return false;
}

}
}

#endif

