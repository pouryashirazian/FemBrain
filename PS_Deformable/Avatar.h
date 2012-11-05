#ifndef AVATAR_H
#define AVATAR_H

#include "../PS_Graphics/PS_GLMeshBuffer.h"

/*!
 * Synopsis: Haptics Avatar
 */
class AvatarCube : public GLMeshBuffer{
public:
	AvatarCube();
	AvatarCube(const vec3f& lo, const vec3f& hi);
	virtual ~AvatarCube();

private:
	void setup(const vec3f& lo, const vec3f& hi);


};

#endif
