#include "PS_Graphics/PS_GLMeshBuffer.h"

class SceneBox : public GLMeshBuffer {
public:
	SceneBox();
	SceneBox(const vec3f& lo, const vec3f& hi);
	virtual ~SceneBox();

	void setup(const vec3f& lo, const vec3f& hi);
};
