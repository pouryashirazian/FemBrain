/*!
  * Namespace: PS::MATH
  * Synopsis: 2,3,4 Dimensional generic vector math. Clean implementation with template classes.
  * Author: Pourya Shirazian
  */
#ifndef PS_VECTOR_H
#define PS_VECTOR_H

#include "PS_MathBase.h"
namespace PS{
namespace MATH{

/*!
  * 2D Vector arithmetic
  */
template<typename T>
class Vec2{
public:
    //Constructors
    Vec2() {}
    Vec2(T x_, T y_):x(x_), y(y_) {}
    Vec2(const Vec2& rhs):x(rhs.x), y(rhs.y) {}

    //Public this functions
    inline void normalize();
    inline Vec2 normalized() const;
    inline T length() const;

    //Static Functions
    static T dot(const Vec2& a, const Vec2& b);
    static T angleDeg(const Vec2& a, const Vec2& b);
    static Vec2 reflect(const Vec2& a, const Vec2& n);
    static T distance(const Vec2& a, const Vec2& b);
    static Vec2 minP(const Vec2& a, const Vec2& b);
    static Vec2 maxP(const Vec2& a, const Vec2& b);


    //Operators
    Vec2& operator=(const Vec2& rhs);
    Vec2 operator*(T s) const;

    Vec2 operator+(const Vec2& rhs) const;
    Vec2 operator-(const Vec2& rhs) const;
public:
    T x;
    T y;
};

template<typename T>
void Vec2<T>::normalize(){
    T dInv = 1.0 / sqrt(x*x + y*y);
    x *= dInv;
    y *= dInv;
}

template<typename T>
Vec2<T> Vec2<T>::normalized() const{
    Vec2<T> result = (*this);
    result.normalize();
    return result;
}

template<typename T>
T Vec2<T>::length() const{
    return sqrt(x*x + y*y);
}

template<typename T>
Vec2<T>& Vec2<T>::operator=(const Vec2<T>& rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
    return (*this);
}

template<typename T>
Vec2<T> Vec2<T>::operator*(T s) const
{
    Vec2<T> result;
    result.x = x * s;
    result.y = y * s;
    return result;
}

template<typename T>
Vec2<T> Vec2<T>::operator+(const Vec2<T>& rhs) const
{
    Vec2<T> result;
    result.x = x + rhs.x;
    result.y = y + rhs.y;
    return result;
}

template<typename T>
Vec2<T> Vec2<T>::operator-(const Vec2<T>& rhs) const
{
    Vec2<T> result;
    result.x = x - rhs.x;
    result.y = y - rhs.y;
    return result;
}

template<typename T>
T Vec2<T>::dot(const Vec2<T>& a, const Vec2<T>& b)
{
    return a.x * b.x + a.y * b.y;
}

template<typename T>
T Vec2<T>::angleDeg(const Vec2<T>& a, const Vec2<T>& b)
{
    Vec2<T> aN = a.normalized();
    Vec2<T> bN = b.normalized();
    return RADTODEG(dot(aN, bN));
}

template<typename T>
Vec2<T> Vec2<T>::reflect(const Vec2<T>& a, const Vec2<T>& n)
{
    Vec2<T> result = - a + 2.0 * dot(a, n);
    return result;
}

template<typename T>
T Vec2<T>::distance(const Vec2<T>& a, const Vec2<T>& b)
{
    return (b - a).length();
}

template<typename T>
Vec2<T> Vec2<T>::minP(const Vec2& a, const Vec2& b)
{
    Vec2<T> result;
    result.x = MATHMIN(a.x, b.x);
    result.y = MATHMIN(a.y, b.y);
    return result;
}

template<typename T>
Vec2<T> Vec2<T>::maxP(const Vec2& a, const Vec2& b)
{
    Vec2<T> result;
    result.x = MATHMAX(a.x, b.x);
    result.y = MATHMAX(a.y, b.y);
    return result;
}


typedef Vec2<float> vec2;
typedef Vec2<float> vec2f;

///////////////////////////////////////////////////////
/*!
  * 3D Vector arithmetic
  */
template<typename T>
class Vec3{
public:
    //Constructors
    Vec3() {}
    Vec3(T x_, T y_, T z_):x(x_), y(y_), z(z_) {}
    Vec3(const Vec3& rhs):x(rhs.x), y(rhs.y), z(rhs.z) {}

    //Public this functions
    inline void normalize();
    inline Vec3 normalized() const;
    inline T length() const;

    //Static Functions
    static T dot(const Vec3& a, const Vec3& b);
    static Vec3 cross(const Vec3& a, const Vec3& b);
    static T angleDeg(const Vec3& a, const Vec3& b);
    static Vec3 reflect(const Vec3& a, const Vec3& n);
    static T distance(const Vec3& a, const Vec3& b);
    static Vec3 minP(const Vec3& a, const Vec3& b);
    static Vec3 maxP(const Vec3& a, const Vec3& b);

    //Operators
    inline Vec3& operator=(const Vec3& rhs);
    Vec3 operator*(T s) const;

    Vec3 operator+(const Vec3& rhs) const;
    Vec3 operator-(const Vec3& rhs) const;
public:
    T x;
    T y;
    T z;
};

template<typename T>
void Vec3<T>::normalize(){
    T dInv = 1.0 / sqrt(x*x + y*y + z*z);
    x *= dInv;
    y *= dInv;
    z *= dInv;
}

template<typename T>
Vec3<T> Vec3<T>::normalized() const{
    Vec3<T> result = (*this);
    result.normalize();
    return result;
}

template<typename T>
T Vec3<T>::length() const{
    return sqrt(x*x + y*y + z*z);
}

template<typename T>
Vec3<T>& Vec3<T>::operator=(const Vec3<T>& rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;
    return (*this);
}

template<typename T>
Vec3<T> Vec3<T>::operator*(T s) const
{
    Vec3<T> result;
    result.x = x * s;
    result.y = y * s;
    result.z = z * s;
    return result;
}

template<typename T>
Vec3<T> Vec3<T>::operator+(const Vec3<T>& rhs) const
{
    Vec3<T> result;
    result.x = x + rhs.x;
    result.y = y + rhs.y;
    result.z = z + rhs.z;
    return result;
}

template<typename T>
Vec3<T> Vec3<T>::operator-(const Vec3<T>& rhs) const
{
    Vec3<T> result;
    result.x = x - rhs.x;
    result.y = y - rhs.y;
    result.z = z - rhs.z;
    return result;
}

template<typename T>
T Vec3<T>::dot(const Vec3<T>& a, const Vec3<T>& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template<typename T>
Vec3<T> Vec3<T>::cross(const Vec3<T>& a, const Vec3<T>& b)
{
    Vec3<T> result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

template<typename T>
T Vec3<T>::angleDeg(const Vec3<T>& a, const Vec3<T>& b)
{
    Vec3<T> aN = a.normalized();
    Vec3<T> bN = b.normalized();
    return RADTODEG(dot(aN, bN));
}

template<typename T>
Vec3<T> Vec3<T>::reflect(const Vec3<T>& a, const Vec3<T>& n)
{
    Vec3<T> result = - a + 2.0 * dot(a, n);
    return result;
}

template<typename T>
T Vec3<T>::distance(const Vec3<T>& a, const Vec3<T>& b)
{
    return (b - a).length();
}

template<typename T>
Vec3<T> Vec3<T>::minP(const Vec3& a, const Vec3& b)
{
    Vec3<T> result;
    result.x = MATHMIN(a.x, b.x);
    result.y = MATHMIN(a.y, b.y);
    result.z = MATHMIN(a.z, b.z);

    return result;
}

template<typename T>
Vec3<T> Vec3<T>::maxP(const Vec3& a, const Vec3& b)
{
    Vec3<T> result;
    result.x = MATHMAX(a.x, b.x);
    result.y = MATHMAX(a.y, b.y);
    result.z = MATHMAX(a.z, b.z);

    return result;
}


typedef Vec3<float> vec3;
typedef Vec3<float> vec3f;
///////////////////////////////////////////////
///////////////////////////////////////////////////////
/*!
  * 4D Vector arithmetic
  */
template<typename T>
class Vec4{
public:
    //Constructors
    Vec4() {}
    Vec4(T x_, T y_, T z_, T w_ = 1.0f):x(x_), y(y_), z(z_), w(w_) {}
    Vec4(const Vec4& rhs):x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w) {}

    /*!
      * Get the xyz part of the 4D vector
      */
    Vec3<T> getVec3() const;

    /*!
      * The dot product of two 4D vectors
      */
    T dot(const Vec4<T>& a, const Vec4<T>& b);

    //Operators
    inline Vec4& operator=(const Vec4& rhs);
    Vec4 operator*(T s) const;

    Vec4 operator+(const Vec4& rhs) const;
    Vec4 operator-(const Vec4& rhs) const;
public:
    T x;
    T y;
    T z;
    T w;
};

template<typename T>
Vec3<T> Vec4<T>::getVec3() const
{
    return Vec3<T>(this->x, this->y, this->z);
}

template<typename T>
T Vec4<T>::dot(const Vec4<T>& a, const Vec4<T>& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template<typename T>
Vec4<T>& Vec4<T>::operator=(const Vec4<T>& rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;
    return (*this);
}

template<typename T>
Vec4<T> Vec4<T>::operator*(T s) const
{
    Vec4<T> result;
    result.x = x * s;
    result.y = y * s;
    result.z = z * s;
    return result;
}

template<typename T>
Vec4<T> Vec4<T>::operator+(const Vec4<T>& rhs) const
{
    Vec4<T> result;
    result.x = x + rhs.x;
    result.y = y + rhs.y;
    result.z = z + rhs.z;
    return result;
}

template<typename T>
Vec4<T> Vec4<T>::operator-(const Vec4<T>& rhs) const
{
    Vec4<T> result;
    result.x = x - rhs.x;
    result.y = y - rhs.y;
    result.z = z - rhs.z;
    return result;
}



typedef Vec4<float> vec4;
typedef Vec4<float> vec4f;
}
}
#endif // PS_VECTOR_H
