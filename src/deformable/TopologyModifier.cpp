/*
 * TopologyModifier.cpp
 *
 *  Created on: Apr 13, 2014
 *      Author: pourya
 */

#include "TopologyModifier.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "graphics/selectgl.h"
#include "graphics/Intersections.h"

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

	int cut(const vector<vec3d>& bladePath0,
		    const vector<vec3d>& bladePath1,
		    vec3d sweptSurface[4]);
public:
	Mesh mesh;

	//Cut Nodes
	vector<Mesh::VertexHandle> cutNodes;

	//Cut Edges
	vector<Mesh::EdgeHandle> cutEdges;
};

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

	for (Mesh::EdgeIter e_it=mesh.edges_begin(); e_it!=mesh.edges_end(); ++e_it) {

		Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(e_it, 0);
		Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(e_it, 1);

		/*
		heh0.


		Mesh::Point s0 = mesh.edge (e.halfedges_[0].vertex_handle_);
		Mesh::Point s1 = mesh.point(e.halfedges_[1].vertex_handle_);

		ss0 = vec3d(s0.data());
		ss1 = vec3d(s1.data());

		int res = IntersectSegmentTriangle(ss0, ss1, tri1, uvw, xyz);
		if(res > 0) {
			cutEdges.push_back(e_it);
		}
		*/
	}
//	// iterate over all vertices
//	for (Mesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it) {
//
//	}

//	// iterate over all halfedges
//	for (MyMesh::HalfedgeIter h_it=mesh.halfedges_begin(); h_it!=mesh.halfedges_end(); ++h_it)
//	   ...; // do something with *h_it, h_it->, or h_it.handle()
//
//
//	// iterator over all faces
//	for (MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it)
//	   ...; // do something with *f_it, f_it->, or f_it.handle()
}

///////////////////////////////////////////////////////////////////////////
CuttableMesh::CuttableMesh(const vector<double>& inTetVertices,
	 	 	 	 	 	 	 	   const vector<U32>& inTetElements) {
	setup(inTetVertices, inTetElements);
}

CuttableMesh::~CuttableMesh() {
	SGMesh::cleanup();
	SAFE_DELETE(m_impl);
	SAFE_DELETE(m_lpOutline);
}

void CuttableMesh::setup(const vector<double>& inTetVertices,
							 const vector<U32>& inTetElements) {
	U32 ctElements = inTetElements.size()/4;

	vector<float> vertices;
	vertices.resize(inTetVertices.size());
	for(U32 i=0;i<inTetVertices.size(); i++)
		vertices[i] = static_cast<float>(inTetVertices[i]);

	//main
	Geometry g;
	g.init(3, 4, 2, ftTriangles);
	g.addTetrahedra(vertices, inTetElements);
	g.addPerVertexColor(vec4f(0, 0, 1, 1));
	SGMesh::setup(g);
	resetTransform();
	if(TheShaderManager::Instance().has("phong")) {
        m_spEffect = SmartPtrSGEffect(new SGEffect(TheShaderManager::Instance().get("phong")));
    }


	//outline
	g.clearBuffer(mbtColor);
	g.addPerVertexColor(vec4f(0,0,0,1));
	m_lpOutline = new SGMesh(g);
	m_lpOutline->setWireFrameMode(true);


	//Implementation
	m_impl = new TopologyImpl();
	for (U32 i = 0; i < ctElements; i++) {
		vec4u32 e = vec4u32(&inTetElements[i * 4]);

		vec3d v[4];
		v[0] = vec3d(&inTetVertices[e.x * 3]);
		v[1] = vec3d(&inTetVertices[e.y * 3]);
		v[2] = vec3d(&inTetVertices[e.z * 3]);
		v[3] = vec3d(&inTetVertices[e.w * 3]);


		Mesh::VertexHandle vHandle[4];
		for(int j=0; j<4; j++)
			vHandle[j] = m_impl->mesh.add_vertex(Mesh::Point(v[j].x, v[j].y, v[j].z));

		//Determinant
		double det = vec3d::dot(v[1] - v[0], vec3d::cross(v[2] - v[0], v[3] - v[0]));
		if (det >= 0) {
			m_impl->mesh.add_face(vHandle[1], vHandle[2], vHandle[3]);
			m_impl->mesh.add_face(vHandle[2], vHandle[0], vHandle[3]);
			m_impl->mesh.add_face(vHandle[3], vHandle[0], vHandle[1]);
			m_impl->mesh.add_face(vHandle[1], vHandle[0], vHandle[2]);
		} else {
			m_impl->mesh.add_face(vHandle[3], vHandle[2], vHandle[1]);
			m_impl->mesh.add_face(vHandle[3], vHandle[0], vHandle[2]);
			m_impl->mesh.add_face(vHandle[1], vHandle[0], vHandle[3]);
			m_impl->mesh.add_face(vHandle[2], vHandle[0], vHandle[1]);
		}

	}
}

void CuttableMesh::draw() {
	SGMesh::draw();

	glDisable(GL_LIGHTING);
	m_lpOutline->draw();
	glEnable(GL_LIGHTING);
}

int CuttableMesh::cut(const vector<vec3d>& bladePath0,
					  const vector<vec3d>& bladePath1,
					  vec3d sweptSurface[4]) {
	return -1;

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


