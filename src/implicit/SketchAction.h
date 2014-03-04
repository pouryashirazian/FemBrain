/*
 * SketchAction.h
 *
 *  Created on: Dec 25, 2013
 *      Author: pourya
 */

#ifndef SKETCHACTION_H_
#define SKETCHACTION_H_

#include <string>
#include "base/Vec.h"
#include "base/Matrix.h"
#include "base/Quaternion.h"
#include "base/SettingsScript.h"
#include "LinearBlobTree.h"

using namespace PS;
using namespace PS::MATH;
using namespace std;

namespace PS {
namespace SKETCH {

/*!
 * Define an interface for all sketch actions
 */
class SketchAction {
public:
	SketchAction();
	SketchAction(string name, bool isReversible = true);
	virtual ~SketchAction(){}

	//Define the status of a command
	enum ActionStatus {asPending, asExecuted, asUnexecuted, asError};

	//Action
	virtual bool execute() = 0;
	virtual bool unexecute() = 0;

	//Properties
	string name() const {return m_name;}
	bool isReversible() const {return m_isReversible;}
	ActionStatus status() const { return m_status;}

	//IO
	virtual bool load(SettingsScript* lpScript, int id) = 0;
	virtual bool store(SettingsScript* lpScript, int id) = 0;
protected:
	ActionStatus m_status;
	string m_name;
	bool m_isReversible;
	LinearBlobTree* m_lpBlob;
};
///////////////////////////////////////////////////////////////////////////
//Affine Actions
class SketchTranslate : public SketchAction {
public:
	SketchTranslate():SketchAction("TRANSLATE", true) { }
	void setTranslate(const vec3f& t) {
		m_tForward = t;
	}

	//Action
	bool execute();
	bool unexecute();

	//IO
	bool load(SettingsScript* lpScript, int id);
	bool store(SettingsScript* lpScript, int id);
protected:
	vec3f m_tForward;
	vec3f m_tBackward;
};

/*!
 * Sketch Add Primitive
 */
class SketchAddPrim : public SketchAction {
public:
	SketchAddPrim():SketchAction("ADD PRIM", true) {
		primType = primNULL;
		idxParent = idxSibling = 0;
		m_idxAddedPrim = -1;
	}

	//Action
	bool execute();
	bool unexecute();

	//IO
	bool load(SettingsScript* lpScript, int id);
	bool store(SettingsScript* lpScript, int id);
protected:
	int m_idxAddedPrim;

public:
	PrimitiveType primType;
	int idxParent;
	int idxSibling;

	vec3f	pos;
	vec3f   dir;
	vec3f 	res;
	vec4f 	color;
};


/*!
 * Sketch Add Operator
 */
class SketchAddOp : public SketchAction {
public:
	SketchAddOp():SketchAction("ADD OP", true) {
		m_idxAddedOp = -1;
		opType = opBlend;
		idxLC = idxRC = idxNext = flags = 0;
	}

	//Action
	bool execute();
	bool unexecute();

	//IO
	bool load(SettingsScript* lpScript, int id);
	bool store(SettingsScript* lpScript, int id);
protected:
	int m_idxAddedOp;

public:
	OperatorType opType;
	int idxLC;
	int idxRC;
	int idxNext;
	vec3f res;
	int flags;
};

}
}



#endif /* SKETCHACTION_H_ */
