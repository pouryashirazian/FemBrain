/*
 * SGPhysicsMesh.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: pourya
 */

#include "SGPhysicsMesh.h"

using namespace PS::SG;

SGPhysicsMesh::SGPhysicsMesh() {
}

SGPhysicsMesh::SGPhysicsMesh(const Geometry& g, float mass) {

	//1
	btQuaternion rotation(btVector3(1.0, 0.0, 0.0), 0.0);

	//2
	vec3f t = this->transform()->getTranslate();
	btVector3 position = btVector3(t.x, t.y, t.z);

	//3
	btDefaultMotionState* motionState = new btDefaultMotionState(
			btTransform(rotation, position));

	//4
	btScalar bodyMass = mass;
	btVector3 bodyInertia;
	m_lpShape = new btConvexHullShape();
	for (int i = 0; i < g.countVertices(); i++) {
		vec3f v = g.vertexAt(i);
		btVector3 btv = btVector3(v.x, v.y, v.z);
		((btConvexHullShape*) m_lpShape)->addPoint(btv);
	}
	m_lpShape->calculateLocalInertia(bodyMass, bodyInertia);

	//5
	btRigidBody::btRigidBodyConstructionInfo bodyCI =
			btRigidBody::btRigidBodyConstructionInfo(bodyMass, motionState,
					m_lpShape, bodyInertia);

	//6
	bodyCI.m_restitution = 1.0f;
	bodyCI.m_friction = 0.5f;

	//7
	m_lpRigidBody = new btRigidBody(bodyCI);

	//8
	m_lpRigidBody->setUserPointer((void*)this);

	//9
	m_lpRigidBody->setLinearFactor(btVector3(1, 1, 0));

}

SGPhysicsMesh::~SGPhysicsMesh() {

}

void SGPhysicsMesh::draw() {

}
