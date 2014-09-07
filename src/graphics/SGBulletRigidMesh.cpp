/*
 * SGBulletRigidMesh.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: pourya
 */

#include "SGBulletRigidMesh.h"

using namespace PS::SG;

SGBulletRigidMesh::SGBulletRigidMesh():SGMesh() {
	init();
}

SGBulletRigidMesh::SGBulletRigidMesh(const Geometry& g, float mass) {
	init();

	setup(g, mass);
}

SGBulletRigidMesh::SGBulletRigidMesh(const SGTransform& t, const Geometry& g, float mass) {
	init();
	this->transform()->copyFrom(t);

	setup(g, mass);
}

SGBulletRigidMesh::~SGBulletRigidMesh() {
	delete (m_lpRigidBody->getMotionState());
	SAFE_DELETE(m_lpRigidBody);
	SAFE_DELETE(m_lpShape);
}

void SGBulletRigidMesh::init() {
	m_lpRigidBody = NULL;
	m_lpShape = NULL;
	resetTransform();
}

void SGBulletRigidMesh::setup(const Geometry& g, float mass) {

	//setup mesh
	SGMesh::setup(g);

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
	btVector3 bodyInertia(0, 0, 0);
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

	//animate
	setAnimate(true);


	if(TheShaderManager::Instance().has("phong")) {
        m_spEffect = SmartPtrSGEffect(new SGEffect(TheShaderManager::Instance().get("phong")));
    }
}

void SGBulletRigidMesh::updateNodeTransformFromMotionState() {

	//btScalar m[16];
	btTransform trans;
	m_lpRigidBody->getMotionState()->getWorldTransform(trans);
	btVector3 t = trans.getOrigin();
	vec3f tt = vec3f(t.x(), t.y(), t.z());

	transform()->translate(tt - transform()->getTranslate());
	//trans.getOpenGLMatrix(m);
//	vec3f s = transform()->getScale();
//	mat44f mtxTR;
//	mtxTR.copyFrom(m);
//	transform()->set(mtxTR);
	//transform()->scale(s);
}

void SGBulletRigidMesh::updateMotionStateFromNodeTransform() {
	if(!m_lpRigidBody->getMotionState())
		return;

	vec3f t = transform()->getTranslate();

	btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(t.x, t.y, t.z));
	m_lpRigidBody->getMotionState()->setWorldTransform(trans);
}


void SGBulletRigidMesh::draw() {

	SGMesh::draw();
}

void SGBulletRigidMesh::timestep() {
	if(m_lpRigidBody == NULL)
		return;

	updateNodeTransformFromMotionState();
}




