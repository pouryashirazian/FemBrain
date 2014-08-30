/*
 * SGPhysicsMesh.h
 *
 *  Created on: Aug 28, 2014
 *      Author: pourya
 */

#ifndef SGPHYSICSMESH_H_
#define SGPHYSICSMESH_H_

#include "SGMesh.h"
#include <btBulletDynamicsCommon.h>

using namespace PS;
using namespace PS::GL;

namespace PS {
namespace SG {

class SGPhysicsMesh : public SGMesh {

public:
	SGPhysicsMesh();
	SGPhysicsMesh(const Geometry& g, float mass);
	virtual ~SGPhysicsMesh();

	void draw();

protected:
	btRigidBody* m_lpRigidBody;
	btCollisionShape* m_lpShape;
};

}
}


#endif /* SGPHYSICSMESH_H_ */
