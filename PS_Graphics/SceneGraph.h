/*
 * SceneGraph.h
 *
 *  Created on: Nov 4, 2012
 *      Author: pourya
 */

#ifndef SCENEGRAPH_H_
#define SCENEGRAPH_H_


#include "loki/Functor.h"
#include "PS_GLMeshBuffer.h"
#include "PS_Matrix.h"
#include "PS_Vector.h"
#include "PS_CopyStack.h"
#include "PS_ShaderGLSL.h"
#include "PS_Mesh.h"

using namespace PS;
using namespace PS::MATH;

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
	typedef enum EffectType {etMaterial, etTexture, etCustomShader};

	//Define a material to be applied on the model
	class EffectTexture{
	public:
		EffectTexture() {}
		virtual ~EffectTexture() {}
	private:
		vec2i m_dim;
		U32 m_uiTexture;
	};

	EffectType getEffectType() const {return m_effectType;}
private:
	EffectType m_effectType;
	PS::SHADER::GLShaderProgram* m_lpShader;
	PS::MESH::MeshMaterial* m_lpMaterial;
};

//Defines Base Functionality of a SceneNode
class SceneNode{
public:
	SceneNode();
	virtual ~SceneNode();

	virtual void draw() = 0;
	mat44f& transformation() {return m_transform;}
private:
	mat44f m_transform;
	vector<SceneNodeEffect*> m_vEffects;
};

/*!
 * Scene Graph
 */
class SceneGraph{
public:
	SceneGraph();
	virtual ~SceneGraph();

	void draw() const;

	CopyStack<mat44f>& stkProjection() {return m_stkProjection;}
	CopyStack<mat44f>& stkModelView() {return m_stkModelView;}

	mat44f modelviewprojection() const;
private:
	CopyStack<mat44f> m_stkProjection;
	CopyStack<mat44f> m_stkModelView;
	std::vector<SceneNode*> m_vSceneNodes;
};


#endif /* SCENEGRAPH_H_ */
