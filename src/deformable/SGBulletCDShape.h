/*
 * SGBulletCDShape.h
 *
 *  Created on: Sep 2, 2014
 *      Author: pourya
 */

#ifndef SGBULLETCDSHAPE_H_
#define SGBULLETCDSHAPE_H_

#include "graphics/SGMesh.h"
#include "VolMesh.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"


using namespace PS::MESH;

namespace PS {
namespace SG {

class SGBulletShape: public SGMesh {
public:
	SGBulletShape();
	SGBulletShape(const Geometry& g);
	virtual ~SGBulletShape();

	virtual void draw();
	virtual void timestep();

	void setup(const Geometry& g);
	void setNodalDisplacement(const vec3f& v);

	bool updateShape(const VolMesh& other);
	bool updateShape(const vec3f& v);
	btCollisionObject* getCollisionObj() const {return m_lpCollisionObj;}

private:
	void init();
	void cleanup();

private:
	btCollisionObject* m_lpCollisionObj;
	btCollisionShape* m_lpShape;
	vec3f m_ndisplace;

	Geometry m_geometry;
};

}
}

#endif /* SGBULLETCDSHAPE_H_ */
