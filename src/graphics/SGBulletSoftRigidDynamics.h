/*
 * CollisionDetection.h
 *
 *  Created on: Aug 28, 2014
 *      Author: pourya
 */

#ifndef SGBULLETSOFTRIGIDYNAMICS_H
#define SGBULLETSOFTRIGIDYNAMICS_H

#include "btBulletDynamicsCommon.h"
#include "SGBulletRigidMesh.h"
#include "SGNode.h"

namespace PS {
namespace SG {


class SGBulletSoftRigidDynamics : public SGNode {
public:
	SGBulletSoftRigidDynamics();
	virtual ~SGBulletSoftRigidDynamics();

	bool addRigidBody(SGBulletRigidMesh* pMesh);
	bool removeRigidBody(SGBulletRigidMesh* pMesh);

	void draw();
	void timestep();

protected:
	void init();
	void cleanup();

private:
	btBroadphaseInterface* m_lpBroadPhase;
	btDefaultCollisionConfiguration* m_lpCollisionConfig;
	btCollisionDispatcher* m_lpDispatcher;
	btSequentialImpulseConstraintSolver* m_lpSolver;
	btDiscreteDynamicsWorld* m_lpDynamicsWorld;

};

}
}


#endif /* SGBULLETSOFTRIGIDYNAMICS_H */
