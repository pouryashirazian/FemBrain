/*
 * CollisionDetection.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: pourya
 */

#include "CollisionDetection.h"
#include "graphics/selectgl.h"

using namespace PS::SG;

struct btDrawingResult : public btCollisionWorld::ContactResultCallback
{
	virtual	btScalar addSingleResult(btManifoldPoint& cp,
			 	 	 	 	 	 	 const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
									 const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
	{
		glBegin(GL_LINES);
		glColor3f(0, 0, 0);

		btVector3 ptA = cp.getPositionWorldOnA();
		btVector3 ptB = cp.getPositionWorldOnB();

		glVertex3d(ptA.x(),ptA.y(),ptA.z());
		glVertex3d(ptB.x(),ptB.y(),ptB.z());
		glEnd();

		return 0;
	}
};

CollisionDetection::CollisionDetection() {
	setup();
}

CollisionDetection::~CollisionDetection() {
	cleanup();
}

bool CollisionDetection::add(SGBulletShape* pBulletShape) {
	if(pBulletShape == NULL)
		return false;
	m_vCollisionObjs.push_back(pBulletShape->getCollisionObj());
	m_lpCollisionWorld->addCollisionObject(pBulletShape->getCollisionObj());

	return true;
}

void CollisionDetection::setup() {
	m_lpCollisionConfiguration = new btDefaultCollisionConfiguration();
	m_lpDispatcher = new btCollisionDispatcher(m_lpCollisionConfiguration);
	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);

	m_lpBroadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);

	//SimpleBroadphase is a brute force alternative, performing N^2 aabb overlap tests
	//SimpleBroadphase*	broadphase = new btSimpleBroadphase;
	m_lpCollisionWorld = new btCollisionWorld(m_lpDispatcher, m_lpBroadphase, m_lpCollisionConfiguration);
}

void CollisionDetection::cleanup() {

	SAFE_DELETE(m_lpCollisionWorld);
	SAFE_DELETE(m_lpDispatcher);
	SAFE_DELETE(m_lpBroadphase);
}

void CollisionDetection::timestep() {
	if(!m_lpCollisionWorld)
		return;

	btVector3	worldBoundsMin,worldBoundsMax;

	m_lpCollisionWorld->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);

	vec3f lo(worldBoundsMin.x(), worldBoundsMin.y(), worldBoundsMin.z());
	vec3f hi(worldBoundsMax.x(), worldBoundsMax.y(), worldBoundsMax.z());
	this->setAABB(AABB(lo, hi));

	m_lpCollisionWorld->performDiscreteCollisionDetection();
}

void CollisionDetection::draw() {

	if(!m_lpCollisionWorld)
		return;

	DrawAABB(m_aabb, vec3f(1.0, 0.0, 0.0));


	m_lpCollisionWorld->debugDrawWorld();
	int numManifolds = m_lpCollisionWorld->getDispatcher()->getNumManifolds();

	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = m_lpCollisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
//		btCollisionObject* obA = const_cast<btCollisionObject*>(contactManifold->getBody0());
//		btCollisionObject* obB = const_cast<btCollisionObject*>(contactManifold->getBody1());

		int numContacts = contactManifold->getNumContacts();
		for (int j = 0; j < numContacts; j++) {
			btManifoldPoint& pt = contactManifold->getContactPoint(j);

			glBegin (GL_LINES);
			glColor3f(0, 0, 1);

			btVector3 ptA = pt.getPositionWorldOnA();
			btVector3 ptB = pt.getPositionWorldOnB();

			glVertex3d(ptA.x(), ptA.y(), ptA.z());
			glVertex3d(ptB.x(), ptB.y(), ptB.z());
			glEnd();
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();
	}


	glDisable (GL_TEXTURE_2D);
	/*
	for (U32 i = 0; i < m_vCollisionObjs.size(); i++) {
		m_lpCollisionWorld->debugDrawObject(m_vCollisionObjs[i]->getWorldTransform(),
											m_vCollisionObjs[i]->getCollisionShape(),
											btVector3(1, 1, 0));
	}
	*/

	btDrawingResult renderCallback;

	if(m_vCollisionObjs.size() > 0) {
		//collisionWorld->contactPairTest(&objects[0],&objects[1], renderCallback);
		m_lpCollisionWorld->contactTest(m_vCollisionObjs[0], renderCallback);
	}
}
