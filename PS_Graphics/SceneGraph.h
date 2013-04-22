/*
 * SceneGraph.h
 *
 *  Created on: Nov 4, 2012
 *      Author: pourya
 */

#ifndef SCENEGRAPH_H_
#define SCENEGRAPH_H_

#include <string>
#include "loki/Singleton.h"
#include "loki/Functor.h"
#include "PS_CopyStack.h"
#include "AssetManager.h"
#include "ShaderManager.h"
#include "PS_ShaderGLSL.h"
#include "SceneNodeTransform.h"
#include "PS_ArcBallCamera.h"
#include "PS_Box.h"

using namespace std;
using namespace PS;
using namespace PS::MATH;

//class PS::MESH::MeshMaterial;

/*!
 * \brief Defines a 1,2,3 dimensional texture to be applied on the scene node for rendering.
 * provides mechanisms for reading and writing to/from png files using lodepng lib.
 */
class EffectTexture{
public:
    EffectTexture() {}
    virtual ~EffectTexture() {}
private:
    vec2i m_dim;
    U32 m_uiTexture;
};

/*!
 * The entire animation scene is modeled in a giant scene graph structure.
 * SceneGraph is serializable and supports mesh-based and non-mesh based models.
 * SceneGraph is generic and can be scripted through python bindings.
 * SceneGraph is highly optimized for fast rasterization.
 * For future work scenegraph will be sent to OpenCL Ray-Tracer for building high-quality output.
 */
class SceneNodeEffect{
public:
    SceneNodeEffect() {}
    virtual ~SceneNodeEffect() {}
    enum EffectType {etMaterial, etTexture, etCustomShader};



	EffectType getEffectType() const {return m_effectType;}
private:
	EffectType m_effectType;
	PS::SHADER::GLShaderProgram* m_lpShader;
	//PS::MESH::MeshMaterial* m_lpMaterial;
};


/*!
 * \brief The SceneNode class is an element in the scenegraph can have
 * effect and transformation associated with.
 */
class SceneNode {
public:
	SceneNode();
	virtual ~SceneNode();

    //Pure virtual function for all kids to override
	virtual void draw() = 0;    
	virtual void drawBBox() const;

    //Computes the bounding box of the model
	void setBBox(const AABB& box) { m_bbox = box;}
    virtual AABB bbox() const {
        return m_bbox;
    }

    //Advances animation using the timer value
    virtual void animate(U64 timer) {
        PS_UNUSED(timer);
    }


    //TODO: Method for marshalling this node in a compact data-structure
    //The compact structure will serve as input to high performance rendering
    //Algorithms such as: Ray-tracing
    void marshal() {}

    //Name for hashing and fetching of the nodes
    string name() const {return m_name;}
    void setName(const string& name) {m_name = name;}

    //Visibility
    bool isVisible() const {return m_bVisible;}
    void setVisible(bool visible) {m_bVisible = visible;}

    //Effect to be managed by asset and shadermanager collections
    SceneNodeEffect* effect() const {return m_lpEffect;}
    void setEffect(SceneNodeEffect* lpEffect) { m_lpEffect = lpEffect;}

    //Transformation
    SceneNodeTransform* transformation() const { return m_lpTransform;}

    //TODO: IO to be able to read and write scene nodes to disk in very fast binary format
    bool read() {return false;}
    bool write() {return false;}
protected:
	//Each scene node should have its associated bounding box.
    //For selection, Hidden Surface Culling and Ray-tracing
	AABB m_bbox;

	//Attribs
    string m_name;
    bool m_bVisible;
    //bool m_bAnimated;
    SceneNodeEffect* m_lpEffect;
    SceneNodeTransform* m_lpTransform;
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
    void animate(U64 timer);

    //Nodes
    void add(SceneNode* aNode);
    void addSceneBox(const AABB& box);
    void addGroundMatrix(int rows, int cols, float step = 1.0f);
    U32 count() const {return m_vSceneNodes.size();}
    SceneNode* get(U32 index) const {return m_vSceneNodes[index];}
    SceneNode* get(const char* name) const;
    SceneNode* last() const;

    //void addAffineWidget();

    //Matrix Stacks
	CopyStack<mat44f>& stkProjection() {return m_stkProjection;}
	CopyStack<mat44f>& stkModelView() {return m_stkModelView;}
	mat44f modelviewprojection() const;

    //Camera
    ArcBallCamera& camera() { return m_camera;}

    //Timing and Profiling services

    //Object Selection

    //Surfaces

    //Save and Load view settings
private:
	CopyStack<mat44f> m_stkProjection;
	CopyStack<mat44f> m_stkModelView;
	std::vector<SceneNode*> m_vSceneNodes;
    ArcBallCamera m_camera;
};

//Singleton Access to scene graph
typedef SingletonHolder<SceneGraph, CreateUsingNew, PhoenixSingleton> TheSceneGraph;

#endif /* SCENEGRAPH_H_ */
