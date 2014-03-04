/*
 * VolumeMesh.cpp
 *
 *  Created on: Sep 22, 2013
 *      Author: pourya
 */
#include "VolumeMesh.h"

namespace PS {
namespace FEM {

double TetrahedraMesh::ComputeTetVolume(const vec3d& a, const vec3d& b,
											 const vec3d& c, const vec3d& d) {
	// volume = 1/6 * | (a-d) . ((b-d) x (c-d)) |
	return (1.0 / 6.0) * fabs ( vec3d::dot(a - d, vec3d::cross(b - d, c - d)));
}

}
}


