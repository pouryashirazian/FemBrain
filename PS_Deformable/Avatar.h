#ifndef AVATAR_H
#define AVATAR_H

#include "../PS_Graphics/PS_GLMeshBuffer.h"

/*!
 * Synopsis: Haptics Avatar
 */
class AvatarCube : public GLMeshBuffer{
public:
	AvatarCube();
	virtual ~AvatarCube();

private:
	void setup();


};

#endif
