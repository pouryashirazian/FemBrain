/*
 * CollisionDetection.h
 *
 *  Created on: Sep 2, 2014
 *      Author: pourya
 */

#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include "btBulletCollisionCommon.h"
#include "graphics/SGMesh.h"
#include "SGBulletCDShape.h"

namespace PS {
namespace SG {

/*!
 * Compute collision and reaction forces between colliding bodies, also
 * takes care of friction forces.
 *
 */
class CollisionDetection : public SGMesh {
public:
	CollisionDetection();
	virtual ~CollisionDetection();

	void timestep();
	void draw();

	bool add(SGBulletShape* pBulletShape);

protected:
	void setup();
	void cleanup();

private:
	btDefaultCollisionConfiguration* m_lpCollisionConfiguration;
	btCollisionWorld* m_lpCollisionWorld;
	btCollisionDispatcher* m_lpDispatcher;
	btAxisSweep3*	m_lpBroadphase;
	vector<btCollisionObject*> m_vCollisionObjs;
};

}
}


#endif /* COLLISIONDETECTION_H_ */
