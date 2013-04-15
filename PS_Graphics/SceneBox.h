#include "PS_Graphics/PS_GLMeshBuffer.h"

class SceneBox : public GLMeshBuffer {
public:
	SceneBox();
	SceneBox(const vec3f& lo, const vec3f& hi);
	virtual ~SceneBox();

	void setup(const vec3f& lo, const vec3f& hi);

    //Sizes
    float width() const {return (m_hi.x - m_lo.x);}
    float height() const {return (m_hi.y - m_lo.y);}
    float depth() const {return (m_hi.z - m_lo.z);}
    vec3f lo() const { return m_lo;}
    vec3f hi() const { return m_hi;}
private:
    vec3f m_lo;
    vec3f m_hi;
};
