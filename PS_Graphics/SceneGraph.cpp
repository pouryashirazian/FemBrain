/*
 * SceneGraph.cpp
 *
 *  Created on: Nov 4, 2012
 *      Author: pourya
 */
#include "SceneGraph.h"

SceneGraph::SceneGraph(){
	m_stkModelView.top().identity();
	m_stkProjection.top().identity();
}


SceneGraph::~SceneGraph()
{

}

void SceneGraph::draw() const
{
	for(U32 i=0; i < m_vSceneNodes.size(); i++)
		m_vSceneNodes[i]->draw();
}

mat44f SceneGraph::modelviewprojection() const
{
	mat44f mtxMVP = m_stkProjection.top() * m_stkModelView.top();
	return mtxMVP;
}

