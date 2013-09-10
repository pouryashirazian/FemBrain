/*
 * SceneNodeTransform.cpp
 *
 *  Created on: Jun 7, 2013
 *      Author: pourya
 */

#include "SceneNodeTransform.h"

SceneNodeTransform::SceneNodeTransform(bool bAutoUpdateBackward) {
	m_mtxForward.identity();
	m_mtxBackward.identity();
	m_bAutoUpdate = bAutoUpdateBackward;
}

SceneNodeTransform::SceneNodeTransform(const SceneNodeTransform* other) {
	m_mtxForward = other->m_mtxForward;
	m_mtxBackward = other->m_mtxBackward;
	m_bAutoUpdate = other->m_bAutoUpdate;
}

SceneNodeTransform::~SceneNodeTransform() {

}


void SceneNodeTransform::scale(const vec3f& s) {
	m_mtxForward.scale(s);
	if(m_bAutoUpdate)
		updateBackward();
}

void SceneNodeTransform::rotate(const quat& q) {
	mat44f work;
	q.toMatrix(work);
	m_mtxForward = m_mtxForward * work;
	if(m_bAutoUpdate)
		updateBackward();

}

void SceneNodeTransform::rotate(float angleDeg, const vec3f& axis) {
	quat q;
	q.fromAngleAxis(angleDeg, axis);
	this->rotate(q);
}

void SceneNodeTransform::translate(const vec3f& t) {
	m_mtxForward.translate(t);
	if(m_bAutoUpdate)
		updateBackward();
}

void SceneNodeTransform::updateBackward() {
	m_mtxBackward = m_mtxForward.inverted();
}

void SceneNodeTransform::reset() {
	m_mtxForward.identity();
}


