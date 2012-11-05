/*
 * PS_Box.cpp
 *
 *  Created on: Oct 29, 2012
 *      Author: pourya
 */
#include "PS_Vector.h"
#include "PS_Ray.h"

using namespace PS::MATH;
using namespace PS::FUNCTIONALMATH;

class AABB{
public:
	AABB(){
		lower = upper = vec3f(0,0,0);
	}

	AABB(const vec3f& lo, const vec3f& hi):lower(lo), upper(hi)
	{}

	virtual ~AABB(){}

	void set(const vec3f& lo, const vec3f& hi){
		lower = lo;
		upper = hi;
	}

	inline vec3f bounds(int idx) const { return (idx == 0)?lower:upper;}

    bool isValid() const { return ((lower.x < upper.x)&&(lower.y < upper.y)&&(lower.z < upper.z));}
    bool intersect(const AABB& rhs) const;
    bool intersect(const Ray& ray, float t0, float t1) const;
    AABB united(const AABB& rhs) const{
    	AABB result;
    	result.lower = vec3f::minP(this->lower, rhs.lower);
    	result.upper = vec3f::maxP(this->upper, rhs.upper);
    	return result;
    }

    void translate(const vec3f& d){
    	lower = lower + d;
    	upper = upper + d;
    }



private:
	vec3f lower;
	vec3f upper;
};



