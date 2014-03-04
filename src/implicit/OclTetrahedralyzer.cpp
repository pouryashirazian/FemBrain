/*
 * OclTetrahedralyzer.cpp

 *
 *  Created on: Dec 26, 2013
 *      Author: pourya
 */

#include "OclTetrahedralyzer.h"
#include "base/Logger.h"

using namespace PS;

namespace PS {
namespace SKETCH {


GPUTetra::GPUTetra():SG::SGMesh() {
    init();
}
    
GPUTetra::GPUTetra(const LinearBlobTree& blob):SG::SGMesh() {
    init();
    setBlob(blob);
}
    
GPUTetra::~GPUTetra() {
    GLMeshBuffer::cleanup();

}

void GPUTetra::init() {
    m_isLoaded = false;
}
    
bool GPUTetra::setBlob(const LinearBlobTree& blob) {
    if(blob.countPrimitives() == 0) {
        LogError("There is no primitives in the blobtree!");
        return false;
    }
    
    if(blob.countMtxNodes() == 0) {
        LogError("There is no mtx nodes in the blobtree!");
        return false;
    }

    m_blob = blob;
    m_aabb = blob.aabb();
    m_isLoaded = true;
    
    return true;
}
    
void GPUTetra::run() {
        
}

void GPUTetra::drawGrid() {
        
}
    
}
}



