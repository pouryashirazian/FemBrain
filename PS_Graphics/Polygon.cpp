/*
 * Polygon.cpp
 *
 *  Created on: Mar 29, 2013
 *      Author: pourya
 */
#include "Polygon.h"

namespace PS{
namespace MATH {

Polygon::Polygon() {

}

Polygon::~Polygon() {
	m_vPoints.resize(0);
	m_vNormals.resize(0);
}

float Polygon::area() const {

	return 0;
}

vec3f Polygon::centroid() const {

	return vec3f(0,0,0);
}

bool Polygon::isConvex() const {
	return true;
}

}
}




