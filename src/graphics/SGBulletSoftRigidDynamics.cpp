/*
 * World.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: pourya
 */

#include "SGBulletSoftRigidDynamics.h"

using namespace PS::SG;

SGBulletSoftRigidDynamics::SGBulletSoftRigidDynamics():SGNode() {
	init();
}

SGBulletSoftRigidDynamics::~SGBulletSoftRigidDynamics() {
	cleanup();
}

void SGBulletSoftRigidDynamics::init() {
	m_lpBroadPhase = new btDbvtBroadphase();
	m_lpCollisionConfig = new btDefaultCollisionConfiguration();

	m_lpDispatcher = new btCollisionDispatcher(m_lpCollisionConfig);
	m_lpSolver = new btSequentialImpulseConstraintSolver();

	m_lpDynamicsWorld = new btDiscreteDynamicsWorld(m_lpDispatcher, m_lpBroadPhase, m_lpSolver, m_lpCollisionConfig);
	m_lpDynamicsWorld->setGravity(btVector3(0, -10, 0));
}

void SGBulletSoftRigidDynamics::cleanup() {
	delete m_lpDynamicsWorld;
	delete m_lpSolver;
	delete m_lpCollisionConfig;
	delete m_lpDispatcher;
	delete m_lpBroadPhase;
}

void SGBulletSoftRigidDynamics::draw() {

}

void SGBulletSoftRigidDynamics::timestep() {
	m_lpDynamicsWorld->stepSimulation(1 / 60.f, 10);

}

bool SGBulletSoftRigidDynamics::addRigidBody(SGBulletRigidMesh* pMesh) {
	if(pMesh == NULL)
		return false;

	m_lpDynamicsWorld->addRigidBody(pMesh->getB3RigidBody());
	return true;
}

bool SGBulletSoftRigidDynamics::removeRigidBody(SGBulletRigidMesh* pMesh) {
	if(pMesh == NULL)
		return false;

	m_lpDynamicsWorld->removeRigidBody(pMesh->getB3RigidBody());
	return true;
}





