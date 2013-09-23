/*
 * VolumeMesh.h
 *
 *  Created on: Sep 22, 2013
 *      Author: pourya
 */

#ifndef VOLUMEMESH_H_
#define VOLUMEMESH_H_

#include <vector>
#include "PS_Graphics/PS_Vector.h"
#include "PS_Graphics/PS_Box.h"

using namespace std;
using namespace PS;
using namespace PS::MATH;

namespace PS {
namespace FEM {

class TetrahedraMesh {
public:
	TetrahedraMesh();
	virtual ~TetrahedraMesh();

	static double ComputeTetVolume(const vec3d& a, const vec3d& b,
									   const vec3d& c, const vec3d& d);

private:
	vector<U32> m_elements;

};

}
}

#endif /* VOLUMEMESH_H_ */
