#ifndef PS_MESHRENDER_H
#define PS_MESHRENDER_H

#include <vector>
#include "PS_GLMeshBuffer.h"
#include "PS_Mesh.h"

namespace PS {
namespace MESH {


class MeshRenderer : public GLMeshBuffer {
public:
    MeshRenderer();
    explicit MeshRenderer(const MeshNode* aNode);
    explicit MeshRenderer(const char* strFilePath, int iNode);
    virtual ~MeshRenderer();

    bool setup(const MeshNode* aNode);

    //Draws the bbox
    void drawAABB();

private:
    AABB m_aabb;
};


}
}

#endif // PS_MESHRENDER_H
