/*
 * SketchAction.cpp
 *
 *  Created on: Dec 25, 2013
 *      Author: pourya
 */

#include "SketchAction.h"
#include "SketchMachine.h"
namespace PS{
namespace SKETCH {

SketchAction::SketchAction(): m_name("unset"), m_isReversible(false) {
	m_lpBlob = TheSketchMachine::Instance().blob();
	m_status = asPending;
}

SketchAction::SketchAction(string name, bool isReversible):m_name(name), m_isReversible(isReversible)
{
	m_lpBlob = TheSketchMachine::Instance().blob();
	m_status = asPending;
}

//Add Primitive
bool SketchAddPrim::execute() {
	assert(m_lpBlob != NULL);

	m_idxAddedPrim = m_lpBlob->actAddPrim(primPoint, 0, pos, dir, res, color);
	if(m_lpBlob->isPrimIndex(m_idxAddedPrim)) {
		m_status = asExecuted;
		return true;
	}
	else
		return false;
}

bool SketchAddPrim::unexecute() {
	assert(m_lpBlob != NULL);
	if(m_status != asExecuted)
		return false;

	bool res = m_lpBlob->actDeletePrim(m_idxAddedPrim);
	if(res) {
		m_status = asUnexecuted;
		return true;
	}
	else
		return false;
}

bool SketchAddPrim::load(SettingsScript* lpScript, int id) {
	return false;
}

bool SketchAddPrim::store(SettingsScript* lpScript, int id) {
	return false;
}

//Action
bool SketchAddOp::execute() {
	assert(m_lpBlob != NULL);
	m_idxAddedOp = m_lpBlob->actAddOp(opType, idxLC, idxRC, idxNext, res, flags);
	if(m_lpBlob->isOpIndex(m_idxAddedOp)) {
		m_status = asExecuted;
		return true;
	}
	else
		return false;
}

bool SketchAddOp::unexecute() {
	assert(m_lpBlob != NULL);

	if(m_lpBlob->actDeleteOp(m_idxAddedOp)) {
		m_status = asUnexecuted;
		return true;
	}
	else
		return false;
}

//IO
bool SketchAddOp::load(SettingsScript* lpScript, int id) {
	return false;
}

bool SketchAddOp::store(SettingsScript* lpScript, int id) {
	return false;
}


}
}

