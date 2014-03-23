#ifndef AVATAR_H
#define AVATAR_H

#include "graphics/SGMesh.h"
#include "graphics/Gizmo.h"
#include "Deformable.h"

using namespace PS;
using namespace PS::SG;

/*!
 * Synopsis: Haptics Avatar guide
 */
class AvatarCube : public SGMesh, public IGizmoListener {
public:
	AvatarCube();
	AvatarCube(Deformable* tissue);
	virtual ~AvatarCube();

	void draw();

	//From Gizmo Manager
	void translate(const vec3f& delta, const vec3f& pos);

	vec3f lower() const;
	vec3f upper() const;
private:
	void setup();

protected:
	Deformable* m_lpTissue;
	vec3f m_lower;
	vec3f m_upper;

	SGMesh m_outline;

	AABB m_aabbAvatar;
	AABB m_model;
};

/*!
 * Synopsis: Scalpel is the tool to cut mesh
 * The path sweptby scalpel will be stored to perform cutting on the mesh
 */
class AvatarScalpel : public GLMeshBuffer{
public:
	AvatarScalpel();
	AvatarScalpel(const vec3d& lo, const vec3d& hi);
	virtual ~AvatarScalpel();

	vec3d lower() const {return m_lo;}
	vec3d upper() const {return m_hi;}
private:
	void setup(const vec3d& lo, const vec3d& hi);

private:
	vec3d m_lo;
	vec3d m_hi;
};

#endif
