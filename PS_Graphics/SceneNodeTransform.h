#ifndef SCENENODETRANSFORM_H
#define SCENENODETRANSFORM_H

#include "PS_Matrix.h"
#include "PS_Vector.h"
#include "PS_Quaternion.h"

using namespace PS::MATH;

/*!
 * class representing transformation nodes in the scene graph.
 * This will decrease memory foot print since a pointer will determine
 * a scene node connection with a transformation node
 */
class SceneNodeTransform {
public:
    SceneNodeTransform(bool bAutoUpdateBackward = false);
    SceneNodeTransform(const SceneNodeTransform* other);
    virtual ~SceneNodeTransform();

    //Transform
    void scale(const vec3f& s);
    void rotate(const quat& q);
    void rotate(float angleDeg, const vec3f& axis);
    void translate(const vec3f& t);

    //Get Transform
    vec3f getScale() const { return m_mtxForward.getDiag().xyz();}
    vec3f getTranslate() const {
    	vec4f t = m_mtxForward.getCol(3);
    	return vec3f(t.x/t.w, t.y/t.w, t.z/t.w);
    }

    void reset();
    void updateBackward();


    const mat44f& forward() const {return m_mtxForward;}
    const mat44f& backward() const {return m_mtxBackward;}

protected:
    mat44f m_mtxForward;
    mat44f m_mtxBackward;
    bool m_bAutoUpdate;
};

#endif // SCENENODETRANSFORM_H
