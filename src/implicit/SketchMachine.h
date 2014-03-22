/*
 * SketchMachine.h
 *
 *  Created on: Dec 25, 2013
 *      Author: pourya
 */

#ifndef SKETCHMACHINE_H_
#define SKETCHMACHINE_H_

#include <vector>
#include "LinearBlobTree.h"
#include "SketchAction.h"
#include "graphics/SGMesh.h"
#include "graphics/SGQuad.h"
#include "graphics/GLTexture.h"
#include "OclPolygonizer.h"
#include "loki/Singleton.h"

#define DEFAULT_MACHINE_DELAY 0

using namespace Loki;
using namespace PS;
using namespace PS::GL;
using namespace PS::SG;
using namespace PS::CL;

namespace PS {
namespace SKETCH {

/*!
 * Provides a queue for actions and performs them.
 * Records actions and provide storage for them.
 * Undoes actions
 */
class SketchMachine : public SGNode {
public:
	SketchMachine();
	virtual ~SketchMachine();
	void clearQ();

	//Draw
	void draw();

	//Control
	void setDelay(int delay) { m_delay = delay;}
	int getDelay() const {return m_delay;}
	void forward();
	void backward();

	//Action Management
	SketchAction* getAction(int index) const;
	void addAction(SketchAction* lpAction);
	int select(const Ray& r);
    
    //MousePress
//    void mousePress(int button, int state, int x, int y);
//    void mouseWheel(int button, int dir, int x, int y);
//    void mouseMove(int x, int y);

    //Get
    const LinearBlobTree* cblob() const {return &m_blob;}
    LinearBlobTree* blob() {return &m_blob;}
    GPUPoly* polygonizer() const {return m_lpPolyModel;}

	//Sync
	void sync();

	//load and Store Blob
	bool loadBlob(const AnsiStr& strFileName);

	//load and store actions
	int loadActions(const AnsiStr& strFileName);
	int storeActions(const AnsiStr& strFileName);
private:
	vector<SketchAction*> m_vActions;
	int m_current;
	int m_delay;
    
    GLTexture* m_lpTexFields;
    SGQuad* m_lpQuad;

	//Entire BlobTree Polygonizer
	GPUPoly* m_lpPolyModel;

	//Selected SubTree Polygonizer
	GPUPoly* m_lpPolySelected;

	//BlobTree
	LinearBlobTree m_blob;
};

typedef SingletonHolder<SketchMachine, CreateUsingNew, PhoenixSingleton> TheSketchMachine;
}
}

#endif /* SKETCHMACHINE_H_ */
