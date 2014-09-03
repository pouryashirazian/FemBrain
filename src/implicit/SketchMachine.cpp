/*
 * SketchMachine.cpp
 *
 *  Created on: Dec 25, 2013
 *      Author: pourya
 */

#include "SketchMachine.h"
#include "base/SettingsScript.h"
#include "base/Logger.h"
#include "base/Vec.h"
#include "graphics/selectgl.h"
#include "graphics/Gizmo.h"
#include "graphics/ArcBallCamera.h"
#include "graphics/SceneGraph.h"


using namespace PS::MATH;

namespace PS {
namespace SKETCH {

SketchMachine::SketchMachine():SGNode(), m_current(0), m_delay(DEFAULT_MACHINE_DELAY) {
	m_lpPolyModel = new GPUPoly();
    m_lpPolySelected = NULL;
	m_current = -1;

    m_lpTexFields = GLTexture::CheckerBoard();
    m_lpQuad = new SGQuad(4, 4, m_lpTexFields);
    
	//Add Global Blend Action
    TheSceneGraph::Instance().add(m_lpPolyModel);
}

SketchMachine::~SketchMachine() {
	this->clearQ();

    SAFE_DELETE(m_lpQuad);
    SAFE_DELETE(m_lpTexFields);
    SAFE_DELETE(m_lpPolyModel);
    SAFE_DELETE(m_lpPolySelected);
}

void SketchMachine::clearQ() {
	SAFE_DELETE(m_lpPolyModel);
	for(U32 i=0; i < m_vActions.size(); i++)
		SAFE_DELETE(m_vActions[i]);
	m_vActions.resize(0);
}

void SketchMachine::draw() {
	m_lpPolyModel->draw();
}

//Control
void SketchMachine::forward() {
	if(m_current == (int)(m_vActions.size() - 1)) {
		LogInfoArg2("All sketch actions are completed. Current=%d, QSize=%d", m_current, m_vActions.size());
		return;
	}

	//Perform
	if (m_vActions[m_current + 1]->execute()) {
		m_current++;
		sync();
	}
}

void SketchMachine::backward() {
	if(m_current < 0) {
		LogInfoArg2("All sketch actions are reversed. Current=%d, QSize=%d", m_current, m_vActions.size());
		return;
	}

	if(!m_vActions[m_current]->isReversible()) {
		LogErrorArg2("Current Action is not reversible! Name: %s, Current: %d", m_vActions[m_current]->name().c_str(), m_current);
		return;
	}

	if(m_vActions[m_current]->unexecute()) {
		m_current--;
		sync();
	}
}

SketchAction* SketchMachine::getAction(int index) const {
	assert(index >= 0 && index < (int)m_vActions.size());
	return m_vActions[index];
}

void SketchMachine::addAction(SketchAction* lpAction) {
	if(lpAction == NULL)
		return;

	//TODO: Add description to all actions
	LogInfoArg1("New action type: %s", lpAction->name().c_str());
	m_vActions.push_back(lpAction);


	//TODO: commit/rollback transaction required to perform actions
	if(lpAction->execute()) {
		m_current = (int)m_vActions.size()-1;
		sync();
	}
}

int SketchMachine::select(const Ray& r) {
	Range t(0.0f, FLT_MAX);
	Range hit;
	Range selHit(FLT_MAX, FLT_MAX);

	int idxSelected = -1;
	for(int i=0; i < (int)m_blob.countPrimitives(); i++) {
		 if(m_blob.getPrimAABB(i).intersect(r, t, hit)) {
			 if(hit.lower() < selHit.lower()) {
				 selHit = hit;
				 idxSelected = i;
				 return i;
			 }
		 }
	}

	return idxSelected;
}
    
void SketchMachine::sync() {
	//TODO: Needs a tracker object to control syncing

	this->setAABB(m_blob.aabb());
	m_lpPolyModel->setBlob(this->m_blob);
	m_lpPolyModel->run();


	vec3i g = m_lpPolyModel->voxelGridDim();
	char buf[1024];
	sprintf(buf, "Poly Grid [%d, %d, %d]", g.x, g.y, g.z);
	LogInfo(buf);
    

	/*
    vec3f dim = m_blob.aabb().extent();
    dim.z = 0;
    vec3f c = m_blob.aabb().center();
    vec3f lo = c - vec3f::mul(0.5, dim);
    vec3f hi = c + vec3f::mul(0.5, dim);
    m_lpPolyModel->computeFieldImage(lo, hi, m_lpTexFields);
    */
    
	glutPostRedisplay();
}

//load and Store Blob
bool SketchMachine::loadBlob(const AnsiStr& strFileName) {

	return m_blob.load(strFileName);
}

//load and store actions
int SketchMachine::loadActions(const AnsiStr& strFileName) {
	return 0;
}

int SketchMachine::storeActions(const AnsiStr& strFileName) {
	SettingsScript* lpScript = new SettingsScript(strFileName, SettingsScript::fmReadWrite);
	int done = 0;
	for(int i=0; i<(int)m_vActions.size(); i++) {
		if(m_vActions[i]->store(lpScript, i))
			done++;
	}
	SAFE_DELETE(lpScript);
	return done;
}

}
}


