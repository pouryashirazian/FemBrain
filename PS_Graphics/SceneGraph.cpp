/*
 * SceneGraph.cpp
 *
 *  Created on: Nov 4, 2012
 *      Author: pourya
 */
#include "SceneGraph.h"
#include <GL/glew.h>
#include "SceneBox.h"
#include "GroundMartrix.h"

SceneNode::SceneNode(): m_bVisible(true), m_lpEffect(NULL), m_lpTransform(NULL) {
}

SceneNode::~SceneNode() {

}

void SceneNode::drawBBox() const
{
	DrawAABB(m_bbox.lower(), m_bbox.upper(), vec3f(0,0,1), 1.0f);
}


////////////////////////////////////////////////////////
SceneGraph::SceneGraph(){
	m_stkModelView.top().identity();
	m_stkProjection.top().identity();
}


SceneGraph::~SceneGraph()
{

}

void SceneGraph::add(SceneNode *aNode) {
    if(aNode != NULL)
        m_vSceneNodes.push_back(aNode);
}

void SceneGraph::addSceneBox(const AABB& box) {
    SceneBox* lpBox = new SceneBox(box.lower(), box.upper());
    lpBox->setName("SceneBox");
    this->add(lpBox);
}

void SceneGraph::addGroundMatrix(int rows, int cols, float step) {
    GroundMatrix* lpMatrix = new GroundMatrix(rows, cols, step);
    lpMatrix->setName("GroundMatrix");
    this->add(lpMatrix);
}

SceneNode* SceneGraph::get(const char* name) const {
	for(U32 i=0; i<m_vSceneNodes.size(); i++) {
		if(m_vSceneNodes[i]->name() == string(name)) {
			return m_vSceneNodes[i];
		}
	}
	return NULL;
}

SceneNode* SceneGraph::last() const {
	U32 ctNodes = m_vSceneNodes.size();
	if(ctNodes > 0)
		return m_vSceneNodes[ctNodes-1];
	else
		return NULL;
}

void SceneGraph::draw()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //Camera
    m_camera.look();

    //Draw All Visible Nodes
    for(U32 i=0; i < m_vSceneNodes.size(); i++) {
        if(m_vSceneNodes[i]->isVisible())
            m_vSceneNodes[i]->draw();
    }

    //Cull SwapBuffers of the API such as
    //glutSwapBuffers
}

void SceneGraph::drawBBoxes() {
    //Draw All Visible Nodes
    for(U32 i=0; i < m_vSceneNodes.size(); i++) {
        if(m_vSceneNodes[i]->isVisible()) {
            AABB box = m_vSceneNodes[i]->bbox();
            if(box.isValid())
                DrawAABB(box.lower(), box.upper(), vec3f(0,0,1), 1.0f);
        }
    }
}

void SceneGraph::animate(U64 timer) {
    for(U32 i=0; i < m_vSceneNodes.size(); i++) {
        m_vSceneNodes[i]->animate(timer);
    }
}

mat44f SceneGraph::modelviewprojection() const
{
	mat44f mtxMVP = m_stkProjection.top() * m_stkModelView.top();
	return mtxMVP;
}

