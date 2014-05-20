/*
 * TopologyModifier.cpp
 *
 *  Created on: Apr 13, 2014
 *      Author: pourya
 */

#include "CuttableMesh.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "graphics/selectgl.h"
#include "graphics/Intersections.h"
#include "vegafem/include/vec3d.h"
#include <map>

using namespace std;
using namespace OpenMesh;
using namespace PS::INTERSECTIONS;

namespace PS {

// Define my personal traits
struct MyTraits : OpenMesh::DefaultTraits
{
  // Let Point and Normal be a vector of doubles
  typedef OpenMesh::Vec3d Point;
  typedef OpenMesh::Vec3d Normal;

  // Already defined in OpenMesh::DefaultTraits
  // HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );

  // Uncomment next line to disable attribute PrevHalfedge
  // HalfedgeAttributes( OpenMesh::Attributes::None );
  //
  // or
  //
  // HalfedgeAttributes( 0 );
};

typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  Mesh;

class TopologyImpl {
public:
	TopologyImpl() {}
	TopologyImpl(int ctVertices, double* vertices, int ctElements, int* elements);
	virtual ~TopologyImpl() {
		clear();
	}

	void clear();
	int cut(const vector<vec3d>& bladePath0,
		    const vector<vec3d>& bladePath1,
		    vec3d sweptSurface[4], bool modifyMesh = false);

	//CutEdge
	struct CutEdge {
		vec3d pos;
		vec3d uvw;
		vec3d e0;
		vec3d e1;
		Mesh::EdgeHandle handle;
	};

	//CutNode
	struct CutNode {
		vec3d pos;
		Mesh::VertexHandle handle;
	};

	//compute aabb
	AABB aabb();

	//distance from a point to a line segment
	double pointLineDistance(const vec3d& v1, const vec3d& v2, const vec3d& p);
	double pointLineDistance(const vec3d& v1, const vec3d& v2, const double len2, const vec3d& p, double* outT = NULL);

private:
	vector< Mesh::VertexHandle > tetsVertexHandles;
	vector< Mesh::FaceHandle > tetsFaceHandles;


public:
	Mesh mesh;


	OpenMesh::VPropHandleT< Mesh::Point > vprop_restpos;

	//Cut Nodes
	std::map < Mesh::VertexHandle, CutNode > mapCutNodes;
	typedef std::map< Mesh::VertexHandle, CutNode >::iterator CUTNODEITER;

	//Cut Edges
	std::map< Mesh::EdgeHandle, CutEdge > mapCutEdges;
	typedef std::map< Mesh::EdgeHandle, CutEdge >::iterator CUTEDGEITER;
};

//Ctor
TopologyImpl::TopologyImpl(int ctVertices, double* vertices, int ctElements, int* elements) {


	//for each vertex an extra vec3d rest pos value
	mesh.add_property( vprop_restpos );

	//Add all vertices
	vector<Mesh::VertexHandle> vHandles;
	vHandles.resize(ctVertices);
	for(U32 i=0; i<(U32)ctVertices; i++) {
		vec3d v = vec3d(&vertices[i * 3]);
		Mesh::Point mp(v.x, v.y, v.z);

		vHandles[i] = mesh.add_vertex(mp);
		mesh.property(vprop_restpos, vHandles[i]) = mp;
	}

	//Add all faces
	tetsVertexHandles.resize(ctElements * 4);
	tetsFaceHandles.resize(ctElements * 4);

	for (U32 i = 0; i < (U32)ctElements; i++) {
		vec4u32 e = vec4u32((U32 *)&elements[i * 4]);

		vec3d v[4];
		v[0] = vec3d(&vertices[e.x * 3]);
		v[1] = vec3d(&vertices[e.y * 3]);
		v[2] = vec3d(&vertices[e.z * 3]);
		v[3] = vec3d(&vertices[e.w * 3]);

		//store vhandles
		for(int j=0; j<4; j++)
			tetsVertexHandles[i * 4 + j] = vHandles[e[j]];

		//Face Mask
		//int faceMask[4][3] = { {0, 1, 2}, {1, 2, 3}, {2, 3, 0}, {0, 1, 3} };

		//Determinant
		double det = vec3d::dot(v[1] - v[0], vec3d::cross(v[2] - v[0], v[3] - v[0]));
		if (det >= 0) {
			tetsFaceHandles[i * 4] = mesh.add_face(vHandles[ e[1] ], vHandles[ e[2] ], vHandles[ e[3] ]);
			tetsFaceHandles[i * 4 + 1] = mesh.add_face(vHandles[ e[2] ], vHandles[ e[0] ], vHandles[ e[3] ]);
			tetsFaceHandles[i * 4 + 2] = mesh.add_face(vHandles[ e[3] ], vHandles[ e[0] ], vHandles[ e[1] ]);
			tetsFaceHandles[i * 4 + 3] = mesh.add_face(vHandles[ e[1] ], vHandles[ e[0] ], vHandles[ e[2] ]);

		} else {
			tetsFaceHandles[i * 4] = mesh.add_face(vHandles[ e[3] ], vHandles[ e[2] ], vHandles[ e[1] ]);
			tetsFaceHandles[i * 4 + 1] = mesh.add_face(vHandles[ e[3] ], vHandles[ e[0] ], vHandles[ e[2] ]);
			tetsFaceHandles[i * 4 + 2] = mesh.add_face(vHandles[ e[1] ], vHandles[ e[0] ], vHandles[ e[3] ]);
			tetsFaceHandles[i * 4 + 3] = mesh.add_face(vHandles[ e[2] ], vHandles[ e[0] ], vHandles[ e[1] ]);
		}
	}
}

AABB TopologyImpl::aabb() {

	double vMin[3], vMax[3];

	U32 count = 0;
	for(Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		Mesh::Point mp = mesh.point(v_it);
		vec3d p = vec3d(mp.data());

		if(count == 0) {
			vMin[0] = vMax[0] = p[0];
			vMin[1] = vMax[1] = p[1];
			vMin[2] = vMax[2] = p[2];
		}
		else {
			vMin[0] = MATHMIN(vMin[0], p[0]);
			vMin[1] = MATHMIN(vMin[1], p[1]);
			vMin[2] = MATHMIN(vMin[2], p[2]);

			vMax[0] = MATHMAX(vMax[0], p[0]);
			vMax[1] = MATHMAX(vMax[1], p[1]);
			vMax[2] = MATHMAX(vMax[2], p[2]);
		}

		count++;
	}

	AABB aabb;
	aabb.set(vec3f((float)vMin[0], (float)vMin[1], (float)vMin[2]),
			 vec3f((float)vMax[0], (float)vMax[1], (float)vMax[2]));

	return aabb;
}

void TopologyImpl::clear() {
	mapCutEdges.clear();
	mapCutNodes.clear();

	//vCutEdgesHandle.resize(0);
	//vCutNodesHandle.resize(0);
	//vCutNodesInfo.resize(0);
}

double TopologyImpl::pointLineDistance(const vec3d& v1, const vec3d& v2, const vec3d& p) {
	const double d2 = (v2 - v1).length2();
	if(d2 == 0.0) return vec3d::distance(p, v1);
	return pointLineDistance(v1, v2, d2, p);
}

double TopologyImpl::pointLineDistance(const vec3d& v1, const vec3d& v2,
									   const double len2, const vec3d& p, double* outT) {
	// Consider the line extending the segment, parameterized as v1 + t (v2 - v1).
	// We find projection of point p onto the line.
	// It falls where t = [(p-1) . (v2-v1)] / |v2-v1|^2
	const double t = vec3d::dot(p - v1, v2 - v1) / len2;
	if(outT)
		*outT = t;
	if (t < 0.0)
		return vec3d::distance(p, v1);
	else if (t > 1.0)
		return vec3d::distance(p, v2);

	// Projection falls on the segment
	const vec3d projection = v1 + (v2 - v1) * t;
	return vec3d::distance(p, projection);
}

int TopologyImpl::cut(const vector<vec3d>& bladePath0,
	    			  const vector<vec3d>& bladePath1,
	    			  vec3d sweptSurface[4], bool modifyMesh) {

	//1.Compute all cut-edges
	//2.Compute cut nodes and remove all incident edges to cut nodes from cut edges
	//3.split cut edges and compute the reference position of the split point
	//4.duplicate cut nodes and incident edges

	// iterate over all edges

	vec3d uvw, xyz, ss0, ss1;

	vec3d tri1[3] = {sweptSurface[0], sweptSurface[1], sweptSurface[2]};
	vec3d tri2[3] = {sweptSurface[0], sweptSurface[2], sweptSurface[3]};

	//Radios of Influence in percent
	const double roi = 0.2;

	//Cut-Edges
	int ctVisitedEdges = 0;
	for (Mesh::EdgeIter e_it=mesh.edges_begin(); e_it!=mesh.edges_end(); ++e_it) {
		ctVisitedEdges++;

		Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(e_it, 0);
		//Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(e_it, 1);

		Mesh::Point s0 = mesh.point(mesh.from_vertex_handle(heh0));
		Mesh::Point s1 = mesh.point(mesh.to_vertex_handle(heh0));

		ss0 = vec3d(s0.data());
		ss1 = vec3d(s1.data());

		int res = IntersectSegmentTriangle(ss0, ss1, tri1, uvw, xyz);
		res += IntersectSegmentTriangle(ss0, ss1, tri2, uvw, xyz);
		if(res > 0) {
			CutEdge ce;
			ce.pos = xyz;
			ce.uvw = uvw;
			ce.e0 = ss0;
			ce.e1 = ss1;
			ce.handle = e_it.handle();

			mapCutEdges.insert( std::pair<Mesh::EdgeHandle, CutEdge>(ce.handle, ce) );
		}
	}


	//blade
	vec3d blade0 = bladePath0[bladePath0.size() - 1];
	vec3d blade1 = bladePath1[bladePath1.size() - 1];
	const double edgelen2 = (blade1 - blade0).length2();
//	Mesh::HalfedgeHandle heh;

	std::map< Mesh::EdgeHandle, CutEdge > finalCutEdges;
	finalCutEdges.insert(mapCutEdges.begin(), mapCutEdges.end());

	int ctRemovedCutEdges = 0;
	mapCutNodes.clear();

	//detect all cut-nodes and remove the cut-edges that emanate from a cut-node
	TopologyImpl::CUTEDGEITER it = mapCutEdges.begin();
	for(it = mapCutEdges.begin(); it != mapCutEdges.end(); ++it ) {
		CutEdge e = it->second;
		Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(e.handle, 0);

		//Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(e_it, 1);

//		Mesh::Point s0 = mesh.point(mesh.from_vertex_handle(heh0));
//		Mesh::Point s1 = mesh.point(mesh.to_vertex_handle(heh0));
		ss0 = it->second.e0;
		ss1 = it->second.e1;
		double d0 = pointLineDistance(blade0, blade1, edgelen2, ss0);
		double d1 = pointLineDistance(blade0, blade1, edgelen2, ss1);
		double denom = (ss1 - ss0).length();
		if(denom == 0)
			denom = 1;

		double t = 10.0;
		if(d0 < d1)
			t = (it->second.pos - ss0).length() / denom;
		else
			t = (it->second.pos - ss1).length() / denom;

		//If the start of edge is close to the swept surface remove all incident edges from Ec
		if(d0 < d1 && t < roi) {
			CutNode cn;
			cn.handle = mesh.from_vertex_handle(heh0);
			cn.pos = ss0;
			mapCutNodes.insert( std::pair<Mesh::VertexHandle, CutNode>(cn.handle, cn) );

			//iterate over all incoming half-edges
			for(Mesh::VertexIHalfedgeIter vih_it = mesh.vih_iter(cn.handle); vih_it; ++vih_it) {

				Mesh::EdgeHandle eh = mesh.edge_handle(vih_it.handle());
				finalCutEdges.erase(eh);
				ctRemovedCutEdges++;
			}
		}
		//If the end of edge close to the swept surface remove all incident edges from Ec
		else if(d0 > d1 && t < roi) {
			CutNode cn;
			cn.handle = mesh.to_vertex_handle(heh0);
			cn.pos = ss1;
			mapCutNodes.insert( std::pair<Mesh::VertexHandle, CutNode>(cn.handle, cn) );

			//iterate over all incoming half-edges
			for(Mesh::VertexIHalfedgeIter vih_it = mesh.vih_iter(cn.handle); vih_it; ++vih_it) {

				Mesh::EdgeHandle eh = mesh.edge_handle(vih_it.handle());
				finalCutEdges.erase(eh);
				ctRemovedCutEdges++;
			}
		}
	}

	//Copy
	mapCutEdges.clear();
	mapCutEdges.insert(finalCutEdges.begin(), finalCutEdges.end());
	if(mapCutNodes.size() > 0)
		printf("Cut nodes count %d.\n", (int)mapCutNodes.size());
	if(mapCutEdges.size() > 0)
		printf("Cut edges count %d. removed %d\n", (int)mapCutEdges.size(), ctRemovedCutEdges);

	//Return if the tool has not left the body
	if(!modifyMesh)
		return -1;

	//Splitting cut-edges
	for(CUTEDGEITER it = mapCutEdges.begin(); it != mapCutEdges.end(); it++) {

		vec3d v = it->second.pos;
		Mesh::VertexHandle vh = mesh.add_vertex(Mesh::Point(v.x, v.y, v.z));

		//set the rest position pint0 for this new vertex
		double denom = (it->second.e1 - it->second.e0).length();
		double t = (it->second.pos - it->second.e0).length() / (denom == 0 ? 1 : denom);

		Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(it->second.handle, 0);
		Mesh::Point mp0 = mesh.property(vprop_restpos, mesh.from_vertex_handle(heh0));
		Mesh::Point mp1 = mesh.property(vprop_restpos, mesh.to_vertex_handle(heh0));
		mesh.property(vprop_restpos, vh) = mp0 + t * (mp1 - mp0);

		mesh.split_edge(it->second.handle, vh);
	}

	//Duplicated cut-nodes
	for(CUTNODEITER it = mapCutNodes.begin(); it != mapCutNodes.end(); it++) {
		vec3d v = it->second.pos;
		Mesh::VertexHandle vh = mesh.add_vertex(Mesh::Point(v.x, v.y, v.z));

		mesh.copy_all_properties(it->second.handle, vh, true);
	}

	//Compute cut-edge-code and cut-node-code
	U32 ctTets = tetsVertexHandles.size() / 4;

	vector<U8> vCutEdgeCode;
	vector<U8> vCutNodeCode;
	vCutEdgeCode.resize(ctTets);
	vCutNodeCode.resize(ctTets);

	Mesh::VertexHandle vh[4];
	Mesh::FaceHandle fh[4];

	//Edge Mask
	int edgeMask[6][2] = { { 0, 1 }, { 1, 2 }, { 2, 0 },
						   { 0, 3 }, { 1, 3 }, { 2, 3 } };

	//Face Mask
	int faceMask[4][3] = { {0, 1, 2}, {1, 2, 3}, {2, 3, 0}, {0, 1, 3} };

	for(U32 i=0; i<ctTets; i++) {
		for(int j=0; j<4; j++) {
			Mesh::FaceHandle fh = tetsFaceHandles[i * 4 + j];
			Mesh::FaceHalfedgeIter fh_it = mesh.fh_iter(fh);

			for(; fh_it; ++fh_it) {

			}
		}


	}


	//Return number of tets cut
	return (int)mapCutEdges.size();
}

///////////////////////////////////////////////////////////////////////////
CuttableMesh::CuttableMesh(const vector<double>& vertices, const vector<U32>& elements)
 : TetMesh(vertices.size() / 3, const_cast<double *>(&vertices[0]), elements.size() / 4, (int*) &elements[0]) {

	setup(vertices.size() / 3, const_cast<double*>(&vertices[0]), elements.size() / 4, (int*) &elements[0]);
}

CuttableMesh::CuttableMesh(int ctVertices, double* vertices, int ctElements, int* elements)
	: TetMesh(ctVertices, vertices, ctElements, elements) {

	setup(ctVertices, vertices, ctElements, elements);
}

CuttableMesh::~CuttableMesh() {
	SAFE_DELETE(m_lpHEMesh);
}

void CuttableMesh::setup(int ctVertices, double* vertices, int ctElements, int* elements) {

	resetTransform();
	if(TheShaderManager::Instance().has("phong")) {
        m_spEffect = SmartPtrSGEffect(new SGEffect(TheShaderManager::Instance().get("phong")));
    }

	//HEMesh
	m_lpHEMesh = new HalfEdgeTetMesh(ctVertices, vertices, ctElements, (U32*)elements);
	m_aabb = m_lpHEMesh->aabb();
	m_ctCompletedCuts = 0;
}

void CuttableMesh::clearCutContext() {
	m_mapCutEdges.clear();
	m_mapCutNodes.clear();
}

void CuttableMesh::draw() {

	/*
	vec3d ss0 = vec3d(0, 0, -1);
	vec3d ss1 = vec3d(0, 1, 0);

	vec3d p[4];
	p[0] = vec3d(-2.944, 0.997, -0.653);
	p[1] = vec3d(1.055, 0.997, -0.653);
	p[2] = vec3d(1.055, 0.344, -0.653);
	p[3] = vec3d(-2.944, 0.344, -0.653);

	vec3d tri1[3] = { p[0], p[1], p[2]};
	vec3d tri2[3] = { p[0], p[2], p[3]};

	vec3d uvw, xyz;
	int res = IntersectSegmentTriangle(ss0, ss1, tri1, uvw, xyz);
	if(res == 0)
		res = IntersectSegmentTriangle(ss0, ss1, tri2, uvw, xyz);
	if(res > 0) {
		glDisable(GL_LIGHTING);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glDisable(GL_CULL_FACE);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA,GL_ONE);

			glColor4d(1.0, 0.0, 1.0, 0.5);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glBegin(GL_QUADS);
				glVertex3dv(p[0].cptr());
				glVertex3dv(p[1].cptr());
				glVertex3dv(p[2].cptr());
				glVertex3dv(p[3].cptr());
			glEnd();
			//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			glDisable(GL_BLEND);
			glEnable(GL_CULL_FACE);

			glColor3d(1.0, 1.0, 1.0);
			glLineWidth(3.0);
			glBegin(GL_LINES);
				glVertex3dv(ss0.cptr());
				glVertex3dv(ss1.cptr());
			glEnd();

			glColor3d(0.0, 0.0, 0.0);
			glPointSize(8.0);
			glBegin(GL_POINTS);
				glVertex3dv(xyz.cptr());
			glEnd();

		glPopAttrib();
		glEnable(GL_LIGHTING);
	}
	*/

	//draw tetmesh
	if(m_lpHEMesh)
		m_lpHEMesh->draw();

	//draw cut context
	int ctCutEdges = m_mapCutEdges.size();
	int ctCutNodes = m_mapCutNodes.size();
	if(ctCutNodes > 0 || ctCutEdges > 0) {
		glDisable(GL_LIGHTING);
		glPushAttrib(GL_ALL_ATTRIB_BITS);

			//Draw cut edges
			glColor3f(0.7, 0.7, 0.7);
			glLineWidth(6.0f);
			glBegin(GL_LINES);
				for(CUTEDGEITER it = m_mapCutEdges.begin(); it != m_mapCutEdges.end(); ++it) {
					glVertex3dv(it->second.e0.cptr());
					glVertex3dv(it->second.e1.cptr());
				}
			glEnd();


			//Draw nodes
			glPointSize(7.0f);
			glColor3f(0, 1, 0);
			glBegin(GL_POINTS);
			//Draw cutedges crossing
			for(CUTEDGEITER it = m_mapCutEdges.begin(); it != m_mapCutEdges.end(); ++it) {
				glVertex3dv(it->second.pos.cptr());
			}
			glEnd();

			glColor3f(0, 0, 0);
			glBegin(GL_POINTS);
			//Draw cutnodes
			for(CUTNODEITER it = m_mapCutNodes.begin(); it != m_mapCutNodes.end(); ++it) {
				glVertex3dv(it->second.pos.cptr());
			}
			glEnd();

		glPopAttrib();
		glEnable(GL_LIGHTING);

	}
}

void CuttableMesh::drawTetElement(U32 el, vec4f& color) {

	vec3d a, b, c, d;
	a = vertexAt(TetMesh::getVertexIndex(el, 0));
	b = vertexAt(TetMesh::getVertexIndex(el, 1));
	c = vertexAt(TetMesh::getVertexIndex(el, 2));
	d = vertexAt(TetMesh::getVertexIndex(el, 3));

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glColor4fv(color.cptr());
	glBegin(GL_LINES);
	glVertex3dv(&a[0]);
	glVertex3dv(&b[0]);

	glVertex3dv(&b[0]);
	glVertex3dv(&d[0]);

	glVertex3dv(&a[0]);
	glVertex3dv(&d[0]);

	glVertex3dv(&a[0]);
	glVertex3dv(&c[0]);

	glVertex3dv(&b[0]);
	glVertex3dv(&c[0]);

	glVertex3dv(&c[0]);
	glVertex3dv(&d[0]);
	glEnd();
	glPopAttrib();

}

int CuttableMesh::cut(const vector<vec3d>& bladePath0,
					  const vector<vec3d>& bladePath1,
					  vec3d sweptSurface[4], bool modifyMesh) {

	//if the swept surface is degenerate then return
	double area = (sweptSurface[1] - sweptSurface[0]).length2() * (sweptSurface[2] - sweptSurface[1]).length2();
	if(area < EPSILON)
		return -1;
	double l2 = (sweptSurface[3] - sweptSurface[2]).length2();
	if(l2 < EPSILON)
		return -1;



	//1.Compute all cut-edges
	//2.Compute cut nodes and remove all incident edges to cut nodes from cut edges
	//3.split cut edges and compute the reference position of the split point
	//4.duplicate cut nodes and incident edges

	// iterate over all edges

	vec3d uvw, xyz, ss0, ss1;

	vec3d tri1[3] = {sweptSurface[0], sweptSurface[1], sweptSurface[2]};
	vec3d tri2[3] = {sweptSurface[0], sweptSurface[2], sweptSurface[3]};

	//Radios of Influence in percent
	const double roi = 0.2;

	//Swept Surf
	printf("SWEPT SURF[0]: %.3f, %.3f, %.3f\n", sweptSurface[0].x, sweptSurface[0].y, sweptSurface[0].z);
	printf("SWEPT SURF[2]: %.3f, %.3f, %.3f\n", sweptSurface[2].x, sweptSurface[2].y, sweptSurface[2].z);

	//Cut-Edges
	for (U32 i=0; i < m_lpHEMesh->countEdges(); i++) {

		U32 hei = m_lpHEMesh->halfedge_from_edge(i, 0);
		HalfEdgeTetMesh::HEDGE he = m_lpHEMesh->const_halfedgeAt(hei);

		ss0 = m_lpHEMesh->const_nodeAt(he.from).pos;
		ss1 = m_lpHEMesh->const_nodeAt(he.to).pos;

		int res = IntersectSegmentTriangle(ss0, ss1, tri1, uvw, xyz);
		if(res == 0)
			res = IntersectSegmentTriangle(ss0, ss1, tri2, uvw, xyz);
		if(res > 0) {
			CutEdge ce;
			ce.pos = xyz;
			ce.uvw = uvw;
			ce.e0 = ss0;
			ce.e1 = ss1;
			ce.idxE0 = he.from;
			ce.idxE1 = he.to;
			ce.idxEdge = i;

			m_mapCutEdges[ce.idxEdge] = ce;
		}
	}


	//blade
	vec3d blade0 = bladePath0[bladePath0.size() - 1];
	vec3d blade1 = bladePath1[bladePath1.size() - 1];
	const double edgelen2 = (blade1 - blade0).length2();

	std::map< U32, CutEdge > finalCutEdges;
	finalCutEdges.insert(m_mapCutEdges.begin(), m_mapCutEdges.end());

	int ctRemovedCutEdges = 0;
	m_mapCutNodes.clear();

	//detect all cut-nodes and remove the cut-edges that emanate from a cut-node
	CUTEDGEITER it = m_mapCutEdges.begin();
	for(it = m_mapCutEdges.begin(); it != m_mapCutEdges.end(); ++it ) {

		ss0 = it->second.e0;
		ss1 = it->second.e1;
		double d0 = pointLineDistance(blade0, blade1, edgelen2, ss0);
		double d1 = pointLineDistance(blade0, blade1, edgelen2, ss1);
		double denom = (ss1 - ss0).length();
		if(denom == 0)
			denom = 1;

		double t = 10.0;
		if(d0 < d1)
			t = (it->second.pos - ss0).length() / denom;
		else
			t = (it->second.pos - ss1).length() / denom;

		//If the start of edge is close to the swept surface remove all incident edges from Ec
		if(d0 < d1 && t < roi) {
			CutNode cn;
			cn.idxNode = it->second.idxE0;
			cn.pos = it->second.e0;
			m_mapCutNodes.insert( std::pair<U32, CutNode>(cn.idxNode, cn) );

			//iterate over all incoming half-edges
			vector<U32> incomingHE;
			m_lpHEMesh->getIncomingHalfEdges(cn.idxNode, incomingHE);

			for(U32 i=0; i < incomingHE.size(); i++) {
				finalCutEdges.erase(m_lpHEMesh->edge_from_halfedge(incomingHE[i]));
				ctRemovedCutEdges++;
			}
		}
		//If the end of edge close to the swept surface remove all incident edges from Ec
		else if(d0 > d1 && t < roi) {
			CutNode cn;
			cn.idxNode = it->second.idxE1;
			cn.pos = it->second.e1;
			m_mapCutNodes.insert( std::pair<U32, CutNode>(cn.idxNode, cn) );

			//iterate over all incoming half-edges
			vector<U32> incomingHE;
			m_lpHEMesh->getIncomingHalfEdges(cn.idxNode, incomingHE);

			for(U32 i=0; i < incomingHE.size(); i++) {
				finalCutEdges.erase(m_lpHEMesh->edge_from_halfedge(incomingHE[i]));
				ctRemovedCutEdges++;
			}
		}
	}

	//Copy
	m_mapCutEdges.clear();
	m_mapCutEdges.insert(finalCutEdges.begin(), finalCutEdges.end());
	if(m_mapCutNodes.size() > 0)
		printf("Cut nodes count %d.\n", (int)m_mapCutNodes.size());
	if(m_mapCutEdges.size() > 0)
		printf("Cut edges count %d. removed %d\n", (int)m_mapCutEdges.size(), ctRemovedCutEdges);

	//Return if the tool has not left the body
	if(!modifyMesh)
		return -1;

	//Splitting cut-edges
	for(CUTEDGEITER it = m_mapCutEdges.begin(); it != m_mapCutEdges.end(); it++) {
		//set the rest position pint0 for this new vertex
		double denom = (it->second.e1 - it->second.e0).length();
		double t = (it->second.pos - it->second.e0).length() / (denom == 0 ? 1 : denom);

		m_lpHEMesh->split_edge(it->second.idxEdge, t);
	}

	//Duplicated cut-nodes
	for(CUTNODEITER it = m_mapCutNodes.begin(); it != m_mapCutNodes.end(); it++) {
		//copy cutnode
	}

	//Find the list of all tets impacted
	vector<U8> cutEdgeCodes;
	vector<U8> cutNodeCodes;
	cutEdgeCodes.resize(m_lpHEMesh->countElements());
	cutNodeCodes.resize(m_lpHEMesh->countElements());

	//count of tets cut
	int ctCutTet = 0;

	//	int edgeMaskPos[6][2] = { {1, 2}, {2, 3}, {3, 1}, {2, 0}, {0, 3}, {0, 1} };
	//	int edgeMaskNeg[6][2] = { {3, 2}, {2, 1}, {1, 3}, {3, 0}, {0, 2}, {1, 0} };
	for(U32 i=0; i < m_lpHEMesh->countElements(); i++) {
		const HalfEdgeTetMesh::ELEM& tet = m_lpHEMesh->const_elemAt(i);

		U8 code = 0;
		for(int e=0; e < 6; e++) {
			U32 edge = m_lpHEMesh->edge_from_halfedge(tet.halfedge[e]);
			if(m_mapCutEdges.find(edge) != m_mapCutEdges.end())
				code |= (1 << e);
		}
		cutEdgeCodes[i] = code;

		if(code != 0)
			ctCutTet++;

		//cut node code
		code = 0;
		for(int e=0; e < 4; e++) {
			U32 node = tet.nodes[e];
			if(m_mapCutNodes.find(node) != m_mapCutNodes.end())
				code |= (1 << e);
		}
		cutNodeCodes[i] = code;
	}


	//Return number of tets cut
	return ctCutTet;
}


void CuttableMesh::displace(double * u) {
	m_lpHEMesh->displace(u);
	m_aabb = m_lpHEMesh->aabb();
}


U32 CuttableMesh::countVertices() const {
	return m_lpHEMesh->countNodes();
}

vec3d CuttableMesh::vertexRestPosAt(U32 i) const {
	return m_lpHEMesh->const_nodeAt(i).restpos;
}

vec3d CuttableMesh::vertexAt(U32 i) const {
	double v[3];
	TetMesh::getVertex(i)->convertToArray(v);
	return vec3d(v);
}

int CuttableMesh::findClosestVertex(const vec3d& query, double& dist, vec3d& outP) const {
	double minDist = GetMaxLimit<double>();
	int idxFound = -1;
	for(U32 i=0; i < m_lpHEMesh->countNodes(); i++) {
		vec3d p = m_lpHEMesh->const_nodeAt(i).pos;
		double dist2 = (query - p).length2();
		if (dist2 < minDist) {
			minDist = dist2;
			outP = p;
			idxFound = i;
		}
	}

	//Distance
	dist = sqrt(minDist);
	return idxFound;
}

double CuttableMesh::pointLineDistance(const vec3d& v1, const vec3d& v2, const vec3d& p) {
	const double d2 = (v2 - v1).length2();
	if(d2 == 0.0) return vec3d::distance(p, v1);
	return pointLineDistance(v1, v2, d2, p);
}

double CuttableMesh::pointLineDistance(const vec3d& v1, const vec3d& v2,
									   const double len2, const vec3d& p, double* outT) {
	// Consider the line extending the segment, parameterized as v1 + t (v2 - v1).
	// We find projection of point p onto the line.
	// It falls where t = [(p-1) . (v2-v1)] / |v2-v1|^2
	const double t = vec3d::dot(p - v1, v2 - v1) / len2;
	if(outT)
		*outT = t;
	if (t < 0.0)
		return vec3d::distance(p, v1);
	else if (t > 1.0)
		return vec3d::distance(p, v2);

	// Projection falls on the segment
	const vec3d projection = v1 + (v2 - v1) * t;
	return vec3d::distance(p, projection);
}


CuttableMesh* CuttableMesh::CreateOneTetra() {
	vector<double> vertices;
	vector<U32> elements;

	vec3d points[4];
	points[0] = vec3d(-1, 0, 0);
	points[1] = vec3d(1, 0, 0);
	points[2] = vec3d(0, 0, -1);
	points[3] = vec3d(0, 1, 0);

	vertices.resize(4*3);
	for(int i=0; i<4 ; i++) {
		points[i].store(&vertices[i*3]);
	}

	elements.resize(4);
	elements[0] = 0;
	elements[1] = 1;
	elements[2] = 2;
	elements[3] = 3;

	CuttableMesh* tet = new CuttableMesh(vertices, elements);
	return tet;
}

}


