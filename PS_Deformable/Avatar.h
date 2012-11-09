#ifndef AVATAR_H
#define AVATAR_H

#include "../PS_Graphics/PS_GLMeshBuffer.h"

/*!
 * Synopsis: Haptics Avatar guide
 */
class AvatarCube : public GLMeshBuffer{
public:
	AvatarCube();
	AvatarCube(const vec3d& lo, const vec3d& hi);
	virtual ~AvatarCube();

	void draw();

	vec3d lower() const {return m_lo;}
	vec3d upper() const {return m_hi;}
private:
	void setup(const vec3d& lo, const vec3d& hi);

private:
	U32 m_uShaderFill;
	U32 m_uShaderLine;
	vec3d m_lo;
	vec3d m_hi;
};

#endif
