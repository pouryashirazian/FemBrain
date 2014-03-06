/*
 * SceneGraph.h
 *
 *  Created on: Nov 4, 2012
 *      Author: pourya
 */

#ifndef SCENEGRAPH_H_
#define SCENEGRAPH_H_

#include <string>
#include <memory>
#include "loki/Singleton.h"
#include "loki/Functor.h"
#include "loki/SmartPtr.h"
#include "base/CopyStack.h"
#include "base/AssetManager.h"

#include "TexManager.h"
#include "ShaderManager.h"
#include "SGEffect.h"
#include "SGTransform.h"
#include "ArcBallCamera.h"
#include "AABB.h"

using namespace Loki;
using namespace std;
using namespace PS;
using namespace PS::GL;
using namespace PS::MATH;


namespace PS {
namespace SG {


//SmartPtrSceneNodeTransform
/*
typedef Loki::SmartPtr<
		SGTransform,
		RefCounted,
		AllowConversion,
		AssertCheck,
		DefaultSPStorage > SmartPtrSGTransform;

typedef Loki::SmartPtr<
  MyClass,
  Loki::RefCounted,
  Loki::AllowConversion,
  Loki::AssertCheck,          // Loki::RejectNull,
  Loki::DefaultSPStorage      // Loki::MallocSPStorage
> MyClassPtr;
*/

/*!
 * \brief The SceneNode class is an element in the scenegraph can have
 * effect and transformation associated with.
 */
class SGNode
{
public:
	SGNode();
	SGNode(const string& name, bool visible = true);
	virtual ~SGNode();

    //Pure virtual function for all kids to override
	virtual void draw() = 0;    
	virtual void drawBBox() const;

    //Advances animation
    virtual void timestep() { }

    //Computes the bounding box of the model
	void setAABB(const AABB& box) { m_aabb = box;}
    virtual AABB aabb() const { return m_aabb;}

    //Selection
    virtual int intersect(const Ray& r);

    //Name for hashing and fetching of the nodes
    string name() const {return m_name;}
    void setName(const string& name) {m_name = name;}

    //Visibility
    bool isVisible() const {return m_visible;}
    void setVisible(bool visible) {m_visible = visible;}

    //Effect to be managed by asset and shadermanager collections
    SmartPtrSGEffect effect() const {return m_spEffect;}
    void setEffect(const SmartPtrSGEffect& spEffect) { m_spEffect = spEffect;}

    //Transformation
    SmartPtrSGTransform transform() const { return m_spTransform;}
    void setTransform(const SmartPtrSGTransform& spTransform) {
    	m_spTransform = spTransform;
    }
    void resetTransform();


    //TODO: IO to be able to read and write scene nodes to disk in very fast binary format
    bool read() {return false;}
    bool write() {return false;}
protected:
	//Each scene node should have its associated bounding box.
    //For selection, Hidden Surface Culling and Ray-tracing
	AABB m_aabb;

	//Attribs
    string m_name;
    bool m_visible;

    //bool m_bAnimated;
    std::shared_ptr<SGEffect> m_spEffect;
    std::shared_ptr<SGTransform> m_spTransform;
};

/*!
 * Scene Graph
 */
class SceneGraph {
public:
	SceneGraph();
	virtual ~SceneGraph();

    //Draws the entire scenegraph for traversing through the list of
    //nodes and calling individual draw methods
    void draw();
    void drawBBoxes();
    void timestep();

    //Nodes
    void add(SGNode* aNode);
    void addSceneBox(const AABB& box);
    void addFloor(int rows, int cols, float step = 1.0f);
    U32 count() const {return (U32)m_vSceneNodes.size();}
    SGNode* get(U32 index) const {return m_vSceneNodes[index];}
    SGNode* get(const char* name) const;
    SGNode* last() const;

    //void addAffineWidget();

    //Matrix Stacks
	CopyStack<mat44f>& stkProjection() {return m_stkProjection;}
	CopyStack<mat44f>& stkModelView() {return m_stkModelView;}
	mat44f modelviewprojection() const;

    //Camera
    ArcBallCamera& camera() { return m_camera;}
    
    //Mouse calls
    void mousePress(int button, int state, int x, int y);
    void mouseWheel(int button, int dir, int x, int y);
    void mouseMove(int x, int y);
    
    //Coordinate Conversion
    bool screenToWorld(const vec3f& s, vec3f& w);
    Ray  screenToWorldRay(int x, int y);
    vec4i viewport() const;

    //Timing and Profiling services

    //Object Selection

    //Surfaces

    //Save and Load view settings
    
protected:
    void cleanup();
private:
	CopyStack<mat44f> m_stkProjection;
	CopyStack<mat44f> m_stkModelView;
	std::vector<SGNode*> m_vSceneNodes;
    ArcBallCamera m_camera;
};

//Singleton Access to scene graph
typedef SingletonHolder<SceneGraph, CreateUsingNew, PhoenixSingleton> TheSceneGraph;

}
}

#endif /* SCENEGRAPH_H_ */
