/*
 * Cutting_CPU.cpp
 *
 *  Created on: Nov 3, 2013
 *      Author: pourya
 */
#include "Cutting_CPU.h"
#include "PS_Graphics/Intersections.h"
#include <tbb/parallel_reduce.h>

using namespace tbb;
using namespace PS;
using namespace PS::MATH;
using namespace PS::INTERSECTIONS;

namespace PS {
namespace FEM {

FacePointComputer::FacePointComputer(const vec3d& s0, const vec3d& s1,
					vector<double>& vertices,
				    vector<U32>& tets,
					vector<double>& outFacePoints,
					vector<U32>& outFaceFlags) : m_edge0(s0), m_edge1(s1),
							m_vertices(vertices), m_tets(tets),
							m_vOutFacePoints(outFacePoints), m_vOutFaceFlags(outFaceFlags)
{

}

void FacePointComputer::operator()(const blocked_range<size_t>& range) const {

	for(size_t idxTet = range.begin(); idxTet < range.end(); idxTet++) {

		//Fetch tet indices
		U32 tet[4];
		for(int i=0; i<4; i++)
			tet[i] = m_tets[idxTet*4 + i];

		//Face Mask
		int faceMask[4][3] = {
		   {0, 1, 2}, {1, 2, 3}, {2, 3, 0}, {0, 1, 3}
		};

		//Init output
		vec3d p[3];
		vec3d uvw, xp;
		//const float oneThird = 1.0f / 3.0f;

		for(int i=0; i<4; i++) {
			U32 idxFP = idxTet * 4 + i;

			m_vOutFaceFlags[idxFP] = 0;
			int tri0 = tet[faceMask[i][0]] * 3;
			int tri1 = tet[faceMask[i][1]] * 3;
			int tri2 = tet[faceMask[i][2]] * 3;
			p[0].load(&m_vertices[tri0]);
			p[1].load(&m_vertices[tri1]);
			p[2].load(&m_vertices[tri2]);

			int res = IntersectSegmentTriangle(m_edge0, m_edge1, p, uvw, xp);
			if(res > 0) {
				m_vOutFaceFlags[idxFP] = 1;
				xp.store(&m_vOutFacePoints[idxFP * 3]);
			}
		}
	}
}

//SumScan
class SumScannerBody {
public:
	SumScannerBody(vector<U32>& arrValues_) : sum(0), values(arrValues_) { }

	SumScannerBody(SumScannerBody& s, tbb::split):values(s.values) {
		sum = 0;
		//values.assign(s.values.begin(), s.values.end());
	}

	~SumScannerBody() {
		values.resize(0);
	}


	void operator()(blocked_range<U32>& range) {
		U32 temp = sum;
		for(U32 i = range.begin(); i != range.end(); i++) {
			temp += values[i];
			values[i] = temp;
		}
		sum = temp;
	}

	void join(SumScannerBody& rhs) {
		sum += rhs.sum;
	}

	U32 getSum() const {return sum;}
private:
	U32 sum;
	vector<U32>& values;
};

//Parallel Sum Scan
U32 ParallelSum(vector<U32>& values) {
	SumScannerBody sumScanner(values);
	tbb::parallel_reduce(blocked_range<U32>(0, values.size()), sumScanner);
	return sumScanner.getSum();
}

int ComputeFacePointsWhenCuttingTetMesh(const vec3d& s0, const vec3d& s1,
											 vector<double>& vertices,
											 vector<U32>& tets,
											 U32 ctVertices, U32 ctTets,
											 U32& ctOutFacePoints, vector<double>& vOutFacePoints)
{
	vector<U32> vOutFaceFlags;
	vOutFaceFlags.resize(ctTets * 4);

	vOutFacePoints.resize(ctTets * 4);



	//Compute FacePoints
	FacePointComputer fpc(s0, s1, vertices, tets, vOutFacePoints, vOutFaceFlags);
	tbb::parallel_for(blocked_range<size_t>(0, ctTets), fpc, tbb::auto_partitioner());

	//Compute the sum
	ctOutFacePoints = ParallelSum(vOutFaceFlags);

	return ctOutFacePoints;
}

}
}



