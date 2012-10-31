/*
 * PS_Box.cpp
 *
 *  Created on: Oct 29, 2012
 *      Author: pourya
 */
#include "PS_VectorMath.h"
#include "PS_Ray.h"

using namespace PS::MATH;
using namespace PS::FUNCTIONALMATH;

class AABB{
public:
	AABB(){
		lower = upper = svec3f(0,0,0);
	}

	AABB(const svec3f& lo, const svec3f& hi):lower(lo), upper(hi)
	{}

	virtual ~AABB(){}

	void set(const svec3f& lo, const svec3f& hi){
		lower = lo;
		upper = hi;
	}

	inline svec3f bounds(int idx) const { return (idx == 0)?lower:upper;}

    bool isValid() const { return ((lower.x < upper.x)&&(lower.y < upper.y)&&(lower.z < upper.z));}
    bool intersect(const AABB& rhs) const;
    bool intersect(const Ray& ray, float t0, float t1) const;

    void translate(const svec3f& d){
    	lower = vadd3f(lower, d);
    	upper = vadd3f(upper, d);
    }



private:
	svec3f lower;
	svec3f upper;
};



