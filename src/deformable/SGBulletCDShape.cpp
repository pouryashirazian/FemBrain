/*
 * SGBulletCDShape.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: pourya
 */

#include "SGBulletCDShape.h"
#include "graphics/selectgl.h"

using namespace PS::SG;



SGBulletShape::SGBulletShape():SGMesh() {
	init();
}

SGBulletShape::SGBulletShape(const Geometry& g):SGMesh(g) {
	init();
	setup(g);
}

SGBulletShape::~SGBulletShape() {
	cleanup();
}

void SGBulletShape::init() {
	m_lpShape = NULL;
	m_lpCollisionObj = NULL;
	m_ndisplace = vec3f(0.0f);
}

void SGBulletShape::cleanup() {
	SAFE_DELETE(m_lpShape);
	SAFE_DELETE(m_lpCollisionObj);
}

void SGBulletShape::setup(const Geometry& g) {
	btMatrix3x3 basisA;
	basisA.setIdentity();

	//set collision object
	m_lpCollisionObj = new btCollisionObject();
	m_lpCollisionObj->getWorldTransform().setBasis(basisA);

	//set shape
	m_lpShape = new btConvexHullShape();
	for (int i = 0; i < g.countVertices(); i++) {
		vec3f v = g.vertexAt(i);
		btVector3 btv = btVector3(v.x, v.y, v.z);
		((btConvexHullShape*) m_lpShape)->addPoint(btv);
	}
	m_lpShape->setMargin(0.f);
	m_lpCollisionObj->setCollisionShape(m_lpShape);

	SGMesh::setup(g);
	m_geometry = g;
	setAABB(g.aabb());
}

void SGBulletShape::setNodalDisplacement(const vec3f& v) {
	m_ndisplace = v;
}

void SGBulletShape::draw() {
	glDisable(GL_CULL_FACE);
	glDisable(GL_LIGHTING);

	SGMesh::draw();

	btConvexHullShape* pCHShape = reinterpret_cast<btConvexHullShape*>(m_lpCollisionObj->getCollisionShape());
	int ctPoints = pCHShape->getNumPoints();
	btVector3* pPoints = pCHShape->getUnscaledPoints();

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glBegin(GL_POINTS);

	glPointSize(5.0f);
	glColor3f(0, 1, 0);
	for(int i=0; i < ctPoints; i++) {
		btVector3 p = pPoints[i];
		vec3f pt = vec3f(p.x(), p.y(), p.z());
		glVertex3fv(pt.cptr());
	}
	glEnd();
	glPopAttrib();

	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
}

void SGBulletShape::timestep() {

	updateShape(m_ndisplace);
}

bool SGBulletShape::updateShape(const VolMesh& other) {
	if(m_lpShape == NULL)
		return false;

	if(!m_lpCollisionObj)
		return false;

	btConvexHullShape* pCHShape = reinterpret_cast<btConvexHullShape*>(m_lpShape);
	int ctPoints = pCHShape->getNumPoints();
	if((int)other.countNodes() != ctPoints)
		return false;
	btVector3* pPoints = pCHShape->getUnscaledPoints();

	for(int i=0; i < ctPoints; i++) {
		vec3d pos = other.const_nodeAt(i).pos;
		pPoints[i] = btVector3(pos.x, pos.y, pos.z);
	}

	this->setAABB(other.aabb());
	return true;
}

bool SGBulletShape::updateShape(const vec3f& v) {
	if(m_lpShape == NULL)
		return false;


	btConvexHullShape* pCHShape = reinterpret_cast<btConvexHullShape*>(m_lpShape);
	int ctPoints = pCHShape->getNumPoints();
	btVector3* pPoints = pCHShape->getUnscaledPoints();
	for(int i=0; i < ctPoints; i++) {
		pPoints[i] = pPoints[i] + btVector3(v.x, v.y, v.z);
	}


	//update
	for(int i=0; i < m_geometry.countVertices(); i++) {
		vec3f c = m_geometry.vertexAt(i);
		m_geometry.setVertex(i, c + v);
	}

	U32 szTotal = m_geometry.countVertices() * m_geometry.getVertexStep() * sizeof(float);
	GLMeshBuffer::updateVertexBuffer(0, szTotal, &m_geometry.vertices()[0]);


	m_aabb.translate(v);
	return true;
}




