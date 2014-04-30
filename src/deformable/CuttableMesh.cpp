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
	virtual ~TopologyImpl() {
		clear();
	}

	void clear();
	int cut(const vector<vec3d>& bladePath0,
		    const vector<vec3d>& bladePath1,
		    vec3d sweptSurface[4]);

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
	double pointLineDistance(const vec3d& v1, const vec3d& v2, const double len2, const vec3d& p);

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

double TopologyImpl::pointLineDistance(const vec3d& v1, const vec3d& v2, const double len2, const vec3d& p) {
	// Consider the line extending the segment, parameterized as v1 + t (v2 - v1).
	// We find projection of point p onto the line.
	// It falls where t = [(p-1) . (v2-v1)] / |v2-v1|^2
	const float t = vec3d::dot(p - v1, v2 - v1) / len2;
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
	    			  vec3d sweptSurface[4]) {

	//1.Compute all cut-edges
	//2.Compute cut nodes and remove all incident edges to cut nodes from cut edges
	//3.split cut edges and compute the reference position of the split point
	//4.duplicate cut nodes and incident edges

	// iterate over all edges

	vec3d uvw, xyz, ss0, ss1;

	vec3d tri1[3] = {sweptSurface[0], sweptSurface[1], sweptSurface[2]};
	vec3d tri2[3] = {sweptSurface[0], sweptSurface[2], sweptSurface[3]};

	//Radios of Influence is half of swept surface length
	const double roi = (sweptSurface[2] - sweptSurface[1]).length() * 0.5;

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


	//CutNodes
	vec3d blade0 = bladePath0[bladePath0.size() - 1];
	vec3d blade1 = bladePath1[bladePath1.size() - 1];
	const double edgelen2 = (blade1 - blade0).length2();
	Mesh::HalfedgeHandle heh;

	//cut edges
	std::map< Mesh::EdgeHandle, CutEdge > finalCutEdges;
	finalCutEdges.insert(mapCutEdges.begin(), mapCutEdges.end());

	int ctRemovedCutEdges = 0;
	mapCutNodes.clear();

	TopologyImpl::CUTEDGEITER it = mapCutEdges.begin();
	for(it = mapCutEdges.begin(); it != mapCutEdges.end(); ++it ) {
		CutEdge e = it->second;
		Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(e.handle, 0);
		//Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(e_it, 1);

		Mesh::Point s0 = mesh.point(mesh.from_vertex_handle(heh0));
		Mesh::Point s1 = mesh.point(mesh.to_vertex_handle(heh0));

		ss0 = vec3d(s0.data());
		ss1 = vec3d(s1.data());

		double d0 = pointLineDistance(blade0, blade1, edgelen2, ss0);
		double d1 = pointLineDistance(blade0, blade1, edgelen2, ss1);

		//If the start of edge is close to the swept surface remove all incident edges from Ec
		if(d0 < d1 && d0 < roi) {
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
		else if(d1 < d0 && d1 < roi) {
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
	SAFE_DELETE(m_impl);
	SAFE_DELETE(m_lpTetMesh);
}

void CuttableMesh::setup(int ctVertices, double* vertices, int ctElements, int* elements) {

	U32 dof = ctVertices * 3;

	vector<float> fvertices;
	fvertices.resize(dof);
	m_vRestPos.resize(dof);

	for(U32 i=0;i<dof; i++) {
		m_vRestPos[i] = vertices[i];
		fvertices[i] = static_cast<float>(vertices[i]);
	}

	//elements
	vector<U32> uelems;
	uelems.resize(ctElements*4);
	for(U32 i=0; i<(U32)ctElements*4; i++)
		uelems[i] = elements[i];

	//main
	Geometry g;
	g.init(3, 4, 2, ftTriangles);
	g.addTetrahedra(fvertices, uelems);
	g.addPerVertexColor(vec4f(0, 0, 1, 1));
	m_lpTetMesh = new SGMesh(g);

	resetTransform();
	if(TheShaderManager::Instance().has("phong")) {
        m_spEffect = SmartPtrSGEffect(new SGEffect(TheShaderManager::Instance().get("phong")));
    }

	//Implementation
	m_impl = new TopologyImpl();

	//for each vertex an extra vec3d rest pos value
	m_impl->mesh.add_property( m_impl->vprop_restpos );

	//Add all vertices
	vector<Mesh::VertexHandle> vHandles;
	vHandles.resize(ctVertices);
	for(U32 i=0; i<(U32)ctVertices; i++) {
		vec3d v = vec3d(&vertices[i * 3]);
		Mesh::Point mp(v.x, v.y, v.z);

		vHandles[i] = m_impl->mesh.add_vertex(mp);
		m_impl->mesh.property(m_impl->vprop_restpos, vHandles[i]) = mp;
	}


	//Add all faces
	for (U32 i = 0; i < (U32)ctElements; i++) {
		vec4u32 e = vec4u32((U32 *)&elements[i * 4]);

		vec3d v[4];
		v[0] = vec3d(&vertices[e.x * 3]);
		v[1] = vec3d(&vertices[e.y * 3]);
		v[2] = vec3d(&vertices[e.z * 3]);
		v[3] = vec3d(&vertices[e.w * 3]);


		//Determinant
		double det = vec3d::dot(v[1] - v[0], vec3d::cross(v[2] - v[0], v[3] - v[0]));
		if (det >= 0) {
			m_impl->mesh.add_face(vHandles[ e[1] ], vHandles[ e[2] ], vHandles[ e[3] ]);
			m_impl->mesh.add_face(vHandles[ e[2] ], vHandles[ e[0] ], vHandles[ e[3] ]);
			m_impl->mesh.add_face(vHandles[ e[3] ], vHandles[ e[0] ], vHandles[ e[1] ]);
			m_impl->mesh.add_face(vHandles[ e[1] ], vHandles[ e[0] ], vHandles[ e[2] ]);
		} else {
			m_impl->mesh.add_face(vHandles[ e[3] ], vHandles[ e[2] ], vHandles[ e[1] ]);
			m_impl->mesh.add_face(vHandles[ e[3] ], vHandles[ e[0] ], vHandles[ e[2] ]);
			m_impl->mesh.add_face(vHandles[ e[1] ], vHandles[ e[0] ], vHandles[ e[3] ]);
			m_impl->mesh.add_face(vHandles[ e[2] ], vHandles[ e[0] ], vHandles[ e[1] ]);
		}
	}


	//aabb
	updateAABB();
}

void CuttableMesh::updateAABB() {
	m_aabb = m_impl->aabb();
}

void CuttableMesh::draw() {

	glDisable(GL_LIGHTING);
	m_lpTetMesh->setWireFrameMode(false);
	m_lpTetMesh->draw();
	glEnable(GL_LIGHTING);

	m_lpTetMesh->setWireFrameMode(true);
	m_lpTetMesh->draw();


	int ctCutNodes = m_impl->mapCutNodes.size();
	int ctCutEdges = m_impl->mapCutEdges.size();
	if(ctCutNodes > 0 || ctCutEdges > 0) {
		glDisable(GL_LIGHTING);

		glPushAttrib(GL_ALL_ATTRIB_BITS);

			//Draw cut edges
			glColor3f(0.7, 0.7, 0.7);
			glLineWidth(4.0f);
			glBegin(GL_LINES);
				for(TopologyImpl::CUTEDGEITER it = m_impl->mapCutEdges.begin(); it != m_impl->mapCutEdges.end(); ++it) {
					glVertex3dv(it->second.e0.cptr());
					glVertex3dv(it->second.e1.cptr());
				}
			glEnd();


			//Draw nodes
			glColor3f(1, 0, 0);
			glPointSize(4.0f);
			glBegin(GL_POINTS);

			//Draw cutedges crossing
			for(TopologyImpl::CUTEDGEITER it = m_impl->mapCutEdges.begin(); it != m_impl->mapCutEdges.end(); ++it) {
				glVertex3dv(it->second.pos.cptr());
			}

			//Draw cutnodes
			for(TopologyImpl::CUTNODEITER it = m_impl->mapCutNodes.begin(); it != m_impl->mapCutNodes.end(); ++it) {
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

void CuttableMesh::clear() {
	if(m_impl)
		m_impl->clear();
}

int CuttableMesh::cut(const vector<vec3d>& bladePath0,
					  const vector<vec3d>& bladePath1,
					  vec3d sweptSurface[4]) {

	int res = m_impl->cut(bladePath0, bladePath1, sweptSurface);

	//sync physics mesh

	//sync rendering bo

	return res;
}

void CuttableMesh::displace(double * u) {

	U32 i = 0;
	for(Mesh::VertexIter v_it = m_impl->mesh.vertices_begin();
		v_it != m_impl->mesh.vertices_end(); ++v_it) {

		m_impl->mesh.set_point( *v_it, Mesh::Point(&m_vRestPos[i]) + Mesh::Point(&u[i*3]));
		i++;
	}

	updateAABB();
}

U32 CuttableMesh::getVertexNeighbors(U32 i, vector<U32>& nbors) {

//	Mesh::VertexIter vb = m_impl->mesh.vertices_begin();
//	Mesh::VertexIter v_it = vb + i;
//	nbors.clear();
//	nbors.reserve(5);
//
//	for (Mesh::VertexVertexIter vv_it= m_impl->mesh.vv_iter(v_it.handle()); vv_it; ++vv_it) {
//		nbors.push_back(vv_it - vb);
//	}

	return nbors.size();
}

U32 CuttableMesh::countVertices() const {
	return TetMesh::getNumVertices();
}

vec3d CuttableMesh::vertexRestPosAt(U32 i) const {
	//assert(i < (m_vRestPos.size() / 3));
	return vec3d(m_vRestPos[i * 3]);
}

vec3d CuttableMesh::vertexAt(U32 i) const {
	double v[3];
	TetMesh::getVertex(i)->convertToArray(v);
	return vec3d(v);
}

int CuttableMesh::findClosestVertex(const vec3d& query, double& dist, vec3d& outP) const {
	double minDist = GetMaxLimit<double>();
	int idxFound = -1;
	int i = 0;

	for(Mesh::VertexIter v_it = m_impl->mesh.vertices_begin();
		v_it != m_impl->mesh.vertices_end(); ++v_it) {

		Mesh::Point mp = m_impl->mesh.point(v_it);
		vec3d p = vec3d(mp.data());
		double dist2 = (query - p).length2();
		if (dist2 < minDist) {
			minDist = dist2;
			outP = p;
			idxFound = i;
		}
		i++;
	}

	//Distance
	dist = sqrt(minDist);
	return idxFound;
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


