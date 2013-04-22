#ifndef PS_VECTORMATH_H
#define PS_VECTORMATH_H

#include <iostream>
#include <algorithm>
#include "../PS_Base/PS_MathBase.h"

namespace PS{
namespace FUNCTIONALMATH{

//Compact Vector Library
template <typename T>
struct svec2
{
	svec2() {}
	explicit svec2(T x_, T y_):x(x_), y(y_) {}
	T x;
	T y;
};

template <typename T>
struct svec3
{

	svec3() {}
	explicit svec3(T x_, T y_, T z_):x(x_), y(y_), z(z_) {}
	T x;
	T y;
	T z;
};

template <typename T>
struct svec4
{
	svec4() {}
	explicit svec4(T x_, T y_, T z_, T w_):x(x_), y(y_), z(z_), w(w_) {}
	T x;
	T y;
	T z;
	T w;
};

///////////////////////////////////////////////////////////
typedef svec2<int> svec2i;
typedef svec2<float> svec2f;
typedef svec2<double> svec2d;

typedef svec3<int> svec3i;
typedef svec3<float> svec3f;
typedef svec3<double> svec3d;

typedef svec4<int> svec4i;
typedef svec4<float> svec4f;
typedef svec4<double> svec4d;
///////////////////////////////////////////////////////////
inline bool isBoxInside(const svec3f& aLo, const svec3f& aHi, const svec3f& bLo, const svec3f& bHi)
{
	if((aLo.x < bLo.x)||(aLo.y < bLo.y)||(aLo.z < bLo.z))
		return false;
	if((aHi.x > bHi.x)||(aHi.y > bHi.y)||(aHi.z > bHi.z))
		return false;
	return true;
}

inline bool isEqual(const svec3i& a, const svec3i& b)
{
	return ((a.x == b.x)&&(a.y == b.y)&&(a.z == b.z));
}
///////////////////////////////////////////////////////////
inline void vsetElement3i(svec3i& a, int dim, int val)
{
	if(dim == 0)
		a.x = val;
	else if(dim == 1)
		a.y = val;
	else
		a.z = val;
}

inline int velement3i(const svec3i& a, int dim)
{
	if(dim == 0)
		return a.x;
	else if(dim == 1)
		return a.y;
	else
		return a.z;
}

//////////////////////////////////////////////////////////////
inline svec3f vadd3f(const svec3f& a, const svec3f& b)
{
	svec3f res;
	res.x = a.x + b.x;
	res.y = a.y + b.y;
	res.z = a.z + b.z;
	return res;
}

inline svec3f vsub3f(const svec3f& a, const svec3f& b)
{
	svec3f res;
	res.x = a.x - b.x;
	res.y = a.y - b.y;
	res.z = a.z - b.z;
	return res;
}

inline float vdot3f(const svec3f& a, const svec3f& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;	
}

inline svec3f vscale3f(float a, const svec3f& b)
{
	svec3f res(a * b.x, a * b.y, a * b.z);	
	return res;
}

inline float vdist3f(const svec3f& a, const svec3f& b)
{
	svec3f d = vsub3f(a, b);
	return sqrtf(vdot3f(d, d));
}

inline float vdistSquared3f(const svec3f& a, const svec3f& b)
{
	svec3f d = vsub3f(a, b);
	return vdot3f(d, d);
}

inline int vmaxExtent3f(const svec3f& hi, const svec3f& lo)
{
	svec3f d = vsub3f(hi, lo);
	if(d.x > d.y && d.x > d.z)
		return 0;
	else if(d.y > d.z)
		return 1;
	else
		return 2;
}

inline float vsurfaceArea3f(const svec3f& hi, const svec3f& lo)
{
	svec3f d = vsub3f(hi, lo);
	return 2.0f * (d.x * d.y + d.y * d.z + d.x * d.z);
}

inline float velement3f(const svec3f& a, int dim)
{
	if(dim == 0)
		return a.x;
	else if(dim == 1)
		return a.y;
	else
		return a.z;
}

inline svec3f vsetAll3f(float val)
{
	return svec3f(val, val, val);
}

inline svec4f vsetAll4f(float val)
{
	return svec4f(val, val, val, val);
}

inline svec3f vload3f(const float* lpArray)
{
	return svec3f(lpArray[0], lpArray[1], lpArray[2]);
}

inline svec4f vload4f(const float* lpArray)
{
	return svec4f(lpArray[0], lpArray[1], lpArray[2], lpArray[3]);
}

inline void vsetElement3f(svec3f& a, int dim, float val)
{
	if(dim == 0)
		a.x = val;
	else if(dim == 1)
		a.y = val;
	else
		a.z = val;
}


inline float vlength3f(const svec3f& a)
{
	float d = a.x * a.x + a.y * a.y + a.z * a.z;
	return sqrtf(d);
}

inline float vlengthSquared3f(const svec3f& a)
{
	return a.x * a.x + a.y * a.y + a.z * a.z;	
}

inline float vnormalize3f(svec3f& a)
{	
	float d = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
	if ( d > 0 )
	{
		float r = 1.0f / d;
		a.x *= r;
		a.y *= r;
		a.z *= r;
	}
	else
	{
		a.x = 0.0f;
		a.y = 0.0f;
		a.z = 0.0f;
	}

	return d;
}

inline svec3f vcross3f(const svec3f& a, const svec3f& b)
{
	svec3f res;
	res.x = a.y*b.z - a.z*b.y;
	res.y = a.z*b.x - a.x*b.z;
	res.z = a.x*b.y - a.y*b.x;
	return res;
}

inline svec3f vmin3f(const svec3f& a, const svec3f& b)
{
	svec3f res;
	res.x = MINF(a.x, b.x);
	res.y = MINF(a.y, b.y);
	res.z = MINF(a.z, b.z);
	return res;
}

inline svec3f vmax3f(const svec3f& a, const svec3f& b)
{
	svec3f res;
	res.x = MAXF(a.x, b.x);
	res.y = MAXF(a.y, b.y);
	res.z = MAXF(a.z, b.z);
	return res;
}

/////////////////////////////////////////////////////////////////|
inline svec4f vadd4f(const svec4f& a, const svec4f& b)
{
	svec4f res;
	res.x = a.x + b.x;
	res.y = a.y + b.y;
	res.z = a.z + b.z;
	res.w = a.w + b.w;
	return res;
}

inline svec4f vsub4f(const svec4f& a, const svec4f& b)
{
	svec4f res;
	res.x = a.x - b.x;
	res.y = a.y - b.y;
	res.z = a.z - b.z;
	res.w = a.w - b.w;
	return res;
}

inline float vdot4f(const svec4f& a, const svec4f& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;	
}

inline svec4f vscale4f(float a, const svec4f& b)
{
	svec4f res(a * b.x, a * b.y, a * b.z, a * b.w);	
	return res;
}
///////////////////////////////////////////////////////////

}
}

#endif
