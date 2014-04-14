/*
 * TopologyModifier.h
 *
 *  Created on: Apr 12, 2014
 *      Author: pourya
 */

#ifndef TOPOLOGYMODIFIER_H_
#define TOPOLOGYMODIFIER_H_

#include "graphics/SGMesh.h"
#include "base/Vec.h"


using namespace PS::MATH;


namespace PS {

class TopologyImpl;

class CuttableMesh : public SGMesh {
public:
	CuttableMesh(const vector<double>& inTetVertices,
		 	 	 	 const vector<U32>& inTetElements);
	virtual ~CuttableMesh();

	void draw();

	//cutting
	int cut(const vector<vec3d>& bladePath0, const vector<vec3d>& bladePath1, vec3d sweptSurface[4]);

	static CuttableMesh* CreateOneTetra();
protected:
	void setup(const vector<double>& inTetVertices,
		 	   const vector<U32>& inTetElements);

private:
	TopologyImpl* m_impl;
	SGMesh* m_lpOutline;
};


/*
class HalfEdgeTopology {
public:
	HalfEdgeTopology();
	virtual ~HalfEdgeTopology();

	struct HalfEdge {
		int vertex;
		int face;
		int next;
		int prev;
		int opposite;
	};

	struct Face {
		int edge;
	};

	struct Vertex {
		vec3f pos;
		int edge;
	};

};
*/

}





#endif /* TOPOLOGYMODIFIER_H_ */
