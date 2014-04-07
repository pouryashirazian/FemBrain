/*
 * AvatarScalpel.cpp
 *
 *  Created on: Apr 6, 2014
 *      Author: pourya
 */

#include "AvatarScalpel.h"

Scalpel::Scalpel(Deformable* tissue):SGMesh(), IGizmoListener() {

	setName("scalpel");
	m_lpTissue = tissue;

	//Geometry
	Geometry g;
	g.init(3, 4, 2, ftTriangles);
	g.addCube(vec3f(-0.5), vec3f(0.5));
	g.addPerVertexColor(vec4f(0, 1, 0, 1), g.countVertices());
	SGMesh::setup(g);

	//Outline
	Geometry gWireframe;
	gWireframe.init(3, 4, 2, ftQuads);
	gWireframe.addCube(vec3f(-0.5), vec3f(0.5));
	m_outline.setup(gWireframe);
	m_outline.setWireFrameMode(true);


	resetTransform();
	if(TheShaderManager::Instance().has("phong")) {
        m_spEffect = SmartPtrSGEffect(new SGEffect(TheShaderManager::Instance().get("phong")));
    }

	//Add a header
	TheSceneGraph::Instance().headers()->addHeaderLine("scalpel", "scalpel");

}

Scalpel::~Scalpel() {
	SGMesh::cleanup();
}

void Scalpel::draw() {
    SGMesh::draw();

    //WireFrame
    m_spTransform->bind();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glDisable(GL_LIGHTING);
    glColor3f(0,0,0);
    	m_outline.draw();
    glEnable(GL_LIGHTING);
    glPopAttrib();

    m_spTransform->unbind();
}

//From Gizmo Manager
void Scalpel::mousePress(int button, int state, int x, int y) {

}

void Scalpel::onTranslate(const vec3f& delta, const vec3f& pos) {

}



