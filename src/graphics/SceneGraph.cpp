/*
 * SceneGraph.cpp
 *
 *  Created on: Nov 4, 2012
 *      Author: pourya
 */
#include <float.h>
#include "SceneGraph.h"
#include "selectgl.h"
#include "SGBox.h"
#include "SGFloor.h"
#include "SGTransform.h"
#include "base/Logger.h"

using namespace PS;

namespace PS {
    namespace SG {
        //////////////////////////////////////////////
        SGNode::SGNode(): m_name("unset"), m_visible(true),
            m_spEffect(std::make_shared<SGEffect>()),
            m_spTransform(std::make_shared<SGTransform>())
        {
        }
        
        SGNode::SGNode(const string& name, bool visible):m_name(name), m_visible(visible),
        m_spEffect(std::make_shared<SGEffect>()),
        m_spTransform(std::make_shared<SGTransform>())
        {
        }
        
        SGNode::~SGNode() {
        }
        
        void SGNode::drawBBox() const
        {
            DrawAABB(m_aabb.lower(), m_aabb.upper(), vec3f(0,0,1), 1.0f);
        }
        
        int SGNode::intersect(const Ray& r) {
            return m_aabb.intersect(r, 0.0f, FLT_MAX);
        }
        
        void SGNode::resetTransform() {
            if(m_spTransform == NULL)
                m_spTransform = SmartPtrSGTransform(new SGTransform());
            else
                m_spTransform->reset();
        }

        ////////////////////////////////////////////////////////
        SceneGraph::SceneGraph(){
            m_stkModelView.top().identity();
            m_stkProjection.top().identity();
        }
        
        
        SceneGraph::~SceneGraph()
        {
            cleanup();
        }
        
        void SceneGraph::cleanup() {
            for(int i=0; i<m_vSceneNodes.size(); i++)
                SAFE_DELETE(m_vSceneNodes[i]);
            m_vSceneNodes.resize(0);
        }
        
        void SceneGraph::add(SGNode *aNode) {
            if(aNode != NULL)
                m_vSceneNodes.push_back(aNode);
        }
        
        void SceneGraph::addSceneBox(const AABB& box) {
            SGBox* lpBox = new SGBox(box.lower(), box.upper());
            lpBox->setName("scenebox");
            this->add(lpBox);
        }
        
        void SceneGraph::addFloor(int rows, int cols, float step) {
            SGFloor* pFloor = new SGFloor(rows, cols, step);
            pFloor->setName("floor");
            this->add(pFloor);
        }
        
        SGNode* SceneGraph::get(const char* name) const {
            for(U32 i=0; i<m_vSceneNodes.size(); i++) {
                if(m_vSceneNodes[i]->name() == string(name)) {
                    return m_vSceneNodes[i];
                }
            }
            return NULL;
        }
        
        SGNode* SceneGraph::last() const {
            U32 ctNodes = (U32)m_vSceneNodes.size();
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
                    AABB box = m_vSceneNodes[i]->aabb();
                    if(box.isValid())
                        DrawAABB(box.lower(), box.upper(), vec3f(0,0,1), 1.0f);
                }
            }
        }
        
        void SceneGraph::timestep() {
            for(U32 i=0; i < m_vSceneNodes.size(); i++) {
                m_vSceneNodes[i]->timestep();
            }
        }
        
        mat44f SceneGraph::modelviewprojection() const
        {
            mat44f mtxMVP = m_stkProjection.top() * m_stkModelView.top();
            return mtxMVP;
        }
        
        void SceneGraph::mouseMove(int x, int y) {
            m_camera.mouseMove(x, y);
        }
        
        void SceneGraph::mouseWheel(int button, int dir, int x, int y) {
            m_camera.mouseWheel(button, dir, x, y);
        }
        
        void SceneGraph::mousePress(int button, int state, int x, int y) {
            // Wheel reports as button 3(scroll up) and button 4(scroll down)
            if ((button == 3) || (button == 4)) // It's a wheel event
            {
                // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
                if (state == GLUT_UP)
                    return; // Disregard redundant GLUT_UP events
                
                if (button == 3)
                    m_camera.setZoom(m_camera.getZoom() - 0.5);
                else
                    m_camera.setZoom(m_camera.getZoom() + 0.5);
                
                return;
            }
            
            m_camera.mousePress(button, state, x, y);
        }
        
        bool SceneGraph::screenToWorld(const vec3f &s, vec3f &w) {
            GLdouble ox, oy, oz;
            GLdouble mv[16];
            GLdouble pr[16];
            GLint vp[4];
            
            glGetDoublev(GL_MODELVIEW_MATRIX, mv);
            glGetDoublev(GL_PROJECTION_MATRIX, pr);
            glGetIntegerv(GL_VIEWPORT, vp);
            if(gluUnProject(s.x, s.y, s.z, mv, pr, vp, &ox, &oy, &oz) == GL_TRUE)
            {
                w = vec3f(ox, oy, oz);
                return true;
            }
            
            return false;
        }

        Ray  SceneGraph::screenToWorldRay(int x, int y) {
            
            vec4i vp = viewport();
            int screenWidth = vp[2];
            int screenHeight = vp[3];
            
            //Select Gizmo First
            vec3f ns(x, screenWidth - y, 0.0f);
            vec3f fs(x, screenHeight - y, 1.0f);
            vec3f nw;
            vec3f fw;
            Ray  r;
            
            if(screenToWorld(ns, nw) && screenToWorld(fs, fw))
            {
                vec3f dir = fw - nw;
                r.set(nw, dir.normalized());
            }
            else {
                LogError("Unable to convert screen to world!");
            }

            return r;
        }
    
        vec4i SceneGraph::viewport() const {
            GLint vp[4];
            glGetIntegerv( GL_VIEWPORT, vp );
            return vec4i(&vp[0]);
        }
}
}

