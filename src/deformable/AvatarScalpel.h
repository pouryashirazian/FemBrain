/*
 * AvatarScalpel.h
 *
 *  Created on: Apr 6, 2014
 *      Author: pourya
 */

#ifndef AVATARSCALPEL_H_
#define AVATARSCALPEL_H_

#include "graphics/SGMesh.h"
#include "graphics/Gizmo.h"
#include "Deformable.h"

using namespace PS;
using namespace PS::SG;

/*!
 * Synopsis: Haptics Avatar guide
 */
class Scalpel : public SGMesh, public IGizmoListener {
public:
	Scalpel(Deformable* tissue);
	virtual ~Scalpel();

	void draw();

	//From Gizmo Manager
	void mousePress(int button, int state, int x, int y);
	void onTranslate(const vec3f& delta, const vec3f& pos);

protected:
	Deformable* m_lpTissue;

	//Outline mesh for easier view
	SGMesh m_outline;
};



#endif /* AVATARSCALPEL_H_ */
