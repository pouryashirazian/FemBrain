#ifndef SG_MESH_H
#define SG_MESH_H

#include "SceneGraph.h"
#include "GLMeshBuffer.h"

using namespace PS;
using namespace PS::GL;

namespace PS {
namespace SG {

class SGMesh : public SGNode, public GLMeshBuffer {
public:
	SGMesh():GLMeshBuffer() {}
	SGMesh(const Geometry& g):GLMeshBuffer(g) {
        this->setBBox(g.aabb());
    }

	virtual ~SGMesh() {
		GLMeshBuffer::cleanup();
	}
    
    void setup(const Geometry& g) {
        GLMeshBuffer::setup(g);
        this->setBBox(g.aabb());
    }

	virtual void draw() {
        //Bind the effect and then draw
        if(m_spEffect)
            m_spEffect->bind();
        
		GLMeshBuffer::draw();
        
        if(m_spEffect)
            m_spEffect->unbind();
	}
    
    virtual void drawNoEffect() {
		GLMeshBuffer::draw();
    }
};

}
}

#endif
