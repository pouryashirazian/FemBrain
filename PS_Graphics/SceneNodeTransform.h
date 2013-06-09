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
    SceneNodeTransform();
    SceneNodeTransform(const SceneNodeTransform* other);
    virtual ~SceneNodeTransform();

    //Transform
    void scale(const vec3f& s);
    void rotate(const quat& q);
    void rotate(float angleDeg, const vec3f& axis);
    void translate(const vec3f& t);


    mat44f forward() const {return m_mtxForward;}

protected:
    mat44f m_mtxForward;
};

#endif // SCENENODETRANSFORM_H
