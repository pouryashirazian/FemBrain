/*
 * SGMesh.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: pourya
 */

#include "SGMesh.h"
#include "selectgl.h"


namespace PS {
namespace SG {

void SGMesh::draw() {
	if(m_spTransform) {
		glPushMatrix();
		glMultMatrixf(m_spTransform->forward().cptr());
	}


    if(m_spEffect)
        m_spEffect->bind();

    GLMeshBuffer::draw();

    if(m_spEffect)
        m_spEffect->unbind();

    if(m_spTransform)
    	glPopMatrix();
}

}
}


