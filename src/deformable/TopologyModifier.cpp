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


using namespace OpenMesh;

namespace PS {

typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;
class TopologyImpl {
public:
	TopologyImpl() {}
public:
	Mesh mesh;

	//Cut Nodes

	//Cut Edges
};

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


