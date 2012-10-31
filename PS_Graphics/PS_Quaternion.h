//***************************************************************************
// Author: Pourya Shirazian 
// Quaternion math for efficient rotations. Use quaternions to avoid 
// gymbal lock problem.
//***************************************************************************
#ifndef PS_QUATERNION_H
#define PS_QUATERNION_H

#include <math.h>

#include "PS_VectorMath.h"
#include "PS_MATRIX4.h"

using namespace PS::FUNCTIONALMATH;

namespace PS{
namespace MATH{

// quaternions are always represent as type float.
// Represents 3d rotations as a quaternion number.
class Quaternion
{
public:
    Quaternion() { identity();}
    Quaternion(const Quaternion& rhs)
    {
    	this->q = rhs.q;
    	this->w = rhs.w;
    }

    Quaternion(const svec3f& q_, float w_)
    {
    	this->q = q_;
    	this->w = w_;
    }

    ~Quaternion(){}

	void identity() {
		q = svec3f(0, 0, 0);
		w = 1;
	}

    svec3f transform(const svec3f& p) const
    {
        Quaternion input(p, 0.0f);
        Quaternion inv = inverted();

        Quaternion res = multiply(input);
        res = res.multiply(inv);
        return res.q;
    }

    svec3f transform(const Quaternion& inv, svec3f p)
    {
        Quaternion input(p, 0.0f);
        Quaternion res = multiply(input);
        res = res.multiply(inv);
        return res.q;
    }


    //Convert from quaternion to a rotation matrix and vice versa
    void fromMatrix(const MAT44& mtx);
    void toMatrix(MAT44 &mtx) const;

    //Convert To/From Euler
    void fromEuler(float roll, float pitch, float yaw);
    void toEuler(float& roll, float& pitch, float& yaw) const;

    // Convert angle/axis into quaternion, and return rotation matrix.
    void fromAngleAxis(float radians,const svec3f &axis)
    {
        float halftheta    = radians*0.5f;
        float sinHalfTheta = (float)sin( halftheta );
        float cosHalfTheta = (float)cos( halftheta );

        q.x = axis.x*sinHalfTheta;
        q.y = axis.y*sinHalfTheta;
        q.z = axis.z*sinHalfTheta;
        w   = cosHalfTheta;
    }

    // Taking the reciprocal of a quaternion makes its rotation go the other way
    void reciprocal()
    {
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    void invert()
    {
        float square = q.x * q.x + q.y * q.y + q.z * q.z + w * w;
        if(square == 0.0f)
            square = 1.0f;
        float coeff = 1.0f/square;

        q = vscale3f(- coeff, q);
        w = w * coeff;
    }

    Quaternion inverted() const
    {
        Quaternion res = *this;
        res.invert();
        return res;
    }

    Quaternion multiply(const Quaternion &b) const
    {
        Quaternion res;
        res.q.x = b.w * q.x + b.q.x * w   + b.q.y * q.z - b.q.z * q.y;
        res.q.y = b.w * q.y + b.q.y * w   + b.q.z * q.x - b.q.x * q.z;
        res.q.z = b.w * q.z + b.q.z * w   + b.q.x * q.y - b.q.y * q.x;
        res.w   = b.w * w   - b.q.x * q.x - b.q.y * q.y - b.q.z * q.z;
        res.normalize();
        return res;
    }

    static Quaternion Multiply(const Quaternion &a, const Quaternion &b)
    {
    	Quaternion res = a.multiply(b);
        return res;
    }

    /*!
     * Get Axis and Angle
     */
    void getAxisAngle(svec3f &axis, float& angleDeg) const
    {
        angleDeg = RADTODEG(acos(w) * 2.0f);
        float sa = sqrt(1.0f - w*w);
        if (sa)
            axis = svec3f(q.x/sa, q.y/sa, q.z/sa);
        else
            axis = svec3f(1,0,0);
    }


    /*!
     * Normalize Quat
     */
    void normalize();


    /*!
     * Assignment to another quaternion
     */
    Quaternion& operator = (const Quaternion& rhs)
    {
        q = rhs.q;
        w = rhs.w;
        return(*this);
    }

    /*!
     * Equality of two quaternions
     */
    bool operator==(const Quaternion &rhs) const
    {
        return ( rhs.q.x == q.x && rhs.q.y == q.y && rhs.q.z == q.z && rhs.w == w );
    }


    Quaternion operator*(const Quaternion& rhs)
    {
    	return this->multiply(rhs);
    }


public:
    //x/y/z components of quaternion.
    svec3f q;

    // w component of quaternion.
    float w;
};



typedef Quaternion quat;
}
}
#endif
