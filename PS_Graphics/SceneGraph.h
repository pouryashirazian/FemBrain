/*
 * SceneGraph.h
 *
 *  Created on: Nov 4, 2012
 *      Author: pourya
 */

#ifndef SCENEGRAPH_H_
#define SCENEGRAPH_H_

#include "PS_GLMeshBuffer.h"

#include "loki/Functor.h"



/*!
 * The entire animation scene is modeled in a giant scene graph structure.
 * SceneGraph is serializable and supports mesh-based and non-mesh based models.
 * SceneGraph is generic and can be scripted through python bindings.
 * SceneGraph is highly optimized for fast rasterization.
 * For future work scenegraph will be sent to OpenCL Ray-Tracer for building high-quality output.
 */


//Defines Base Functionality of a SceneNode
class SceneNode{

};

class SceneGraph{

};


#endif /* SCENEGRAPH_H_ */
