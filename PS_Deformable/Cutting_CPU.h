/*
 * Cutting_CPU.h
 *
 *  Created on: Nov 3, 2013
 *      Author: pourya
 */

#ifndef CUTTING_CPU_H_
#define CUTTING_CPU_H_

#include <vector>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <PS_Graphics/PS_Vector.h>

using namespace tbb;
using namespace std;
using namespace PS;
using namespace PS::MATH;

namespace PS {
namespace FEM {

class FacePointComputer {
public:
	FacePointComputer(const vec3d& s0, const vec3d& s1,
						vector<double>& vertices,
						vector<U32>& tets,
						vector<double>& outFacePoints,
						vector<U32>& outFaceFlags);
	void operator()(const blocked_range<size_t>& range) const;

private:
	vec3d m_edge0;
	vec3d m_edge1;
	vector<double> &m_vertices;
	vector<U32> &m_tets;
	vector<double> &m_vOutFacePoints;
	vector<U32> &m_vOutFaceFlags;

};


/*!
 * SumScan over an array of U32
 */
U32 ParallelSum(vector<U32>& values);

/*!
 * Computes all face intersections in parallel when cutting the tet-mesh
 */
int ComputeFacePointsWhenCuttingTetMesh(const vec3d& s0, const vec3d& s1,
											 vector<double>& vertices,
											 vector<U32>& tets,
											 U32 ctVertices, U32 ctTets,
											 U32& ctOutFacePoints, vector<double>& vOutFacePoints);


}
}

#endif /* CUTTING_CPU_H_ */
