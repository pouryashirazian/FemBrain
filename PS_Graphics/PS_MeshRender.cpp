#include "PS_MeshRender.h"
#include <GL/glew.h>

namespace PS {
namespace MESH {

MeshRenderer::MeshRenderer() {

}

MeshRenderer::MeshRenderer(const MeshNode* aNode) {
    m_aabb = aNode->computeBoundingBox();
    setup(aNode);
}

MeshRenderer::MeshRenderer(const char* strFilePath, int iNode) {
    Mesh* lpMesh = new Mesh(strFilePath);    
    m_aabb = lpMesh->computeBoundingBox();
    setup(lpMesh->getNode(iNode));
    SAFE_DELETE(lpMesh);
}

MeshRenderer::~MeshRenderer() {
    this->cleanup();
}

bool MeshRenderer::setup(const MeshNode* aNode) {
    if(aNode == NULL)
        return false;
    if(aNode->countVertices() > 0)
        this->setupVertexAttribs(aNode->vertices(), aNode->getUnitVertex(), vatPosition);
    if(aNode->countNormals() > 0)
        this->setupVertexAttribs(aNode->normals(), 3, vatNormal);
    if(aNode->countTexCoords() > 0)
        this->setupVertexAttribs(aNode->texcoords(), aNode->getUnitTexCoord(), vatTexCoord);
    if(aNode->countFaces() > 0)
        this->setupIndexBufferObject(aNode->faceElements());

    return true;
}

void MeshRenderer::drawAABB() {
    DrawAABB(m_aabb.lower(), m_aabb.upper(), vec3f(0,0,1), 1.0f);
}


}
}
