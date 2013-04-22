#ifndef SCENENODETRANSFORM_H
#define SCENENODETRANSFORM_H

#include "PS_Matrix.h"
#include "PS_Vector.h"

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
    virtual ~SceneNodeTransform() {}

protected:
    mat44f m_mtx;
};

#endif // SCENENODETRANSFORM_H
