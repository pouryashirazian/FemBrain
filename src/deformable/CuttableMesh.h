/*
 * TopologyModifier.h
 *
 *  Created on: Apr 12, 2014
 *      Author: pourya
 */

#ifndef CUTTABLEMESH_H_
#define CUTTABLEMESH_H_

#include "vegafem/include/tetMesh.h"
#include "graphics/SGMesh.h"
#include "base/Vec.h"


using namespace PS::MATH;


namespace PS {

class TopologyImpl;

class CuttableMesh : public SGNode, public TetMesh {
public:
	CuttableMesh(const vector<double>& vertices, const vector<U32>& elements);
	CuttableMesh(int ctVertices, double* vertices, int ctElements, int* elements);
	virtual ~CuttableMesh();

	//draw
	void draw();
	void drawTetElement(U32 el, vec4f& color);

	//cutting
	int cut(const vector<vec3d>& bladePath0, const vector<vec3d>& bladePath1, vec3d sweptSurface[4]);

	//apply displacements
	void displace(double * u);

	//Access vertex neibors
	U32 getVertexNeighbors(U32 i, vector<U32>& nbors);
	U32 countVertices() const;
	vec3d vertexRestPosAt(U32 i) const;
	vec3d vertexAt(U32 i) const;
	int findClosestVertex(const vec3d& query, double& dist, vec3d& outP) const;

	void clear();

	//create a tetrahedra
	static CuttableMesh* CreateOneTetra();

protected:
	void setup(int ctVertices, double* vertices, int ctElements, int* elements);

	void updateAABB();

	//TODO: Sync physics mesh after cut

	//TODO: Sync vbo after synced physics mesh
private:
	TopologyImpl* m_impl;
	SGMesh* m_lpTetMesh;

	//rest pos
	vector<double> m_vRestPos;
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





#endif /* CUTTABLEMESH_H_ */
