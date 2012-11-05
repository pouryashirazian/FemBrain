#ifndef CINTERVAL_H
#define CINTERVAL_H

#include <float.h>

#define INTERVAL_EPSILON 0.0001f

namespace PS{
	namespace MATH{

class Interval
{


public:
	Interval():left(INTERVAL_EPSILON), right(FLT_MAX) {};

	Interval(float min, float max)
	{
		if(min > max)
		{
			left = max;
			right = min;
		}
		else
		{
			left = min;
			right = max;
		}
	}

	void setMin(const float min)
	{
		left = min;
	}

	void setMax(const float max)
	{
		right = max;
	}

	void set(const float l, const float r)
	{
		left = l;
		right = r;
	}

	float length() { return right - left;}

	bool isInside(const float f) const { return ((f >= left)&&(f <= right));}


	bool hasOverlap(const Interval &A)
	{
		return (isInside(A.left))||(isInside(A.right));
	}


	Interval& operator=(const Interval& y)
	{
		this->left = y.left;
		this->right = y.right;
		return(*this);
	}

	bool operator==(const Interval& y) const
	{
		return ((this->left == y.left)&&(this->right == y.right));
	}

	bool operator!=(const Interval& y) const
	{
		return ((this->left != y.left)||(this->right != y.right));
	}

/*
	CInterval operator +(const CInterval &A) const
	{
		
	}
*/	
public:
	float left;
	float right;
};

}
}

#endif
