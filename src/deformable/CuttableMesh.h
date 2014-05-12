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
#include "HalfEdgeTetMesh.h"
#include "base/Vec.h"


using namespace PS::MATH;
using namespace PS::FEM;

namespace PS {

class CuttableMesh : public SGNode, public TetMesh {
public:
	//CutEdge
	struct CutEdge {
		vec3d pos;
		vec3d uvw;
		vec3d e0;
		vec3d e1;
		U32 idxE0;
		U32 idxE1;
		U32 idxEdge;
	};

	//CutNode
	struct CutNode {
		vec3d pos;
		U32 idxNode;
	};

public:
	CuttableMesh(const vector<double>& vertices, const vector<U32>& elements);
	CuttableMesh(int ctVertices, double* vertices, int ctElements, int* elements);
	virtual ~CuttableMesh();

	//distances
	double pointLineDistance(const vec3d& v1, const vec3d& v2, const vec3d& p);
	double pointLineDistance(const vec3d& v1, const vec3d& v2,
							 const double len2, const vec3d& p, double* outT = NULL);


	//draw
	void draw();
	void drawTetElement(U32 el, vec4f& color);

	//cutting
	void clearCutContext();
	int cut(const vector<vec3d>& bladePath0, const vector<vec3d>& bladePath1,
			vec3d sweptSurface[4], bool modifyMesh);

	//apply displacements
	void displace(double * u);

	//Access vertex neibors
	U32 countVertices() const;
	vec3d vertexRestPosAt(U32 i) const;
	vec3d vertexAt(U32 i) const;
	int findClosestVertex(const vec3d& query, double& dist, vec3d& outP) const;
	int countCompletedCuts() const {return m_ctCompletedCuts;}

	//create a tetrahedra
	static CuttableMesh* CreateOneTetra();

protected:
	void setup(int ctVertices, double* vertices, int ctElements, int* elements);

	//TODO: Sync physics mesh after cut

	//TODO: Sync vbo after synced physics mesh
private:
	HalfEdgeTetMesh* m_lpHEMesh;
	int m_ctCompletedCuts;

	//Cut Nodes
	std::map<U32, CutNode > m_mapCutNodes;
	typedef std::map<U32, CutNode >::iterator CUTNODEITER;

	//Cut Edges
	std::map<U32, CutEdge > m_mapCutEdges;
	typedef std::map<U32, CutEdge >::iterator CUTEDGEITER;
};


}





#endif /* CUTTABLEMESH_H_ */
