#ifndef AVATAR_H
#define AVATAR_H

#include "graphics/SGMesh.h"

using namespace PS;
using namespace PS::SG;

/*!
 * Synopsis: Haptics Avatar guide
 */
class AvatarCube : public SGMesh {
public:
	AvatarCube();
	AvatarCube(const vec3d& lo, const vec3d& hi);
	virtual ~AvatarCube();

	vec3d lower() const {return m_lo;}
	vec3d upper() const {return m_hi;}
private:
	void setup(const vec3d& lo, const vec3d& hi);

private:
	vec3d m_lo;
	vec3d m_hi;
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
