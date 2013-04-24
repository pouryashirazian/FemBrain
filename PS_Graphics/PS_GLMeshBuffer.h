/*
 * PS_MeshGLBuffer.h
 *
 *  Created on: Sep 29, 2012
 *      Author: pourya
 */

#ifndef PS_MESHGLBUFFER_H_
#define PS_MESHGLBUFFER_H_

#include "../PS_Base/PS_MathBase.h"
#include "PS_Vector.h"
#include "PS_GLFuncs.h"
#include "SceneGraph.h"
#include <vector>

using namespace std;
using namespace PS;
using namespace PS::MATH;

/*
#define GL_LINES 0x0001
#define GL_LINE_LOOP 0x0002
#define GL_POINT_BIT 0x00000002
#define GL_CLIENT_VERTEX_ARRAY_BIT 0x00000002
#define GL_LINE_STRIP 0x0003
#define GL_LINE_BIT 0x00000004
#define GL_TRIANGLES 0x0004
#define GL_TRIANGLE_STRIP 0x0005
#define GL_TRIANGLE_FAN 0x0006
#define GL_QUADS 0x0007
#define GL_QUAD_STRIP 0x0008
#define GL_POLYGON_BIT 0x00000008
#define GL_POLYGON 0x0009
*/

enum VertexAttribType {vatPosition, vatNormal, vatColor, vatTexCoord, vatCount};

enum FaceType {ftPoints = 0x0000, ftLines = 0x0001, ftLineLoop = 0x0002, ftLineStrip = 0x0003,
			   ftTriangles = 0x0004, ftTriangleStrip = 0x0005, ftTriangleFan = 0x0006,
			   ftQuads = 0x0007, ftQuadStrip = 0x0008, ftPolygon = 0x0009};

enum ShaderEffectType {setFixedFunction, setCustom};

/*!
 * Synopsis: GLMeshBuffer Simplifies drawing geometries using GPU Buffers.
 * Types of buffer objects are: Vertex, Color, TexCoord, Normal and Index
 */
class GLMeshBuffer : public SceneNode {
public:
	static const U32 INVALID_GLBUFFER = ~0;

	GLMeshBuffer();
	virtual ~GLMeshBuffer();

	//Setup Buffer Objects
	void setupVertexAttribs(const vector<float>& arrAttribs, int step = 3, VertexAttribType attribKind = vatPosition);
	void setupPerVertexColor(const vec4f& color, U32 ctVertices, int step = 4);
	void setupIndexBufferObject(const vector<U32>& arrIndex, int faceMode = ftTriangles);


	//Effect
	void setShaderEffectProgram(U32 glEffect);
	U32 getShaderEffectProgram() const {return m_uShaderEffectProgram;}

	void setEffectType(ShaderEffectType effect) { m_effectType = effect;}
	ShaderEffectType getEffectType() const {return m_effectType;}

	bool getWireFrameMode() const {return m_bWireFrame;}
	void setWireFrameMode(bool bSet) { m_bWireFrame = bSet;}

	int getFaceMode() const {return m_faceMode;}
	void setFaceMode(int fmode) {
		m_faceMode = fmode;
		m_ctFaceElements = m_ctVertices;
	}

	U32 countVertices() const {return m_ctVertices;}
	U32 countTriangles() const {return m_ctFaceElements/3;}
	U32 countFaceElements() const {return m_ctFaceElements;}

	//Validity
	bool isFaceBufferValid() const { return m_isValidIndex;}
	bool isVertexBufferValid(VertexAttribType attribType = vatPosition) const;

	//Steps
	U32 faceStep() const {return m_stepFace;}
	U32 vertexAttribStep(VertexAttribType attribType = vatPosition) const;

	//Buffer Objects
	U32 indexBufferObjectGL() const { return m_iboFaces;}
	U32 vertexBufferObjectGL(VertexAttribType attribType = vatPosition) const;


	//Reads Mesh
	bool readbackMeshVertexAttribGL(VertexAttribType attrib, U32& count, vector<float>& values) const;
	bool readbackMeshFaceGL(U32& count, vector<U32>& elements) const;


	//Draw
	virtual void draw();

	//Mesh buffer to draw normals
    static GLMeshBuffer* PrepareMeshBufferForDrawingNormals(float len, U32 ctVertices, U32 fstep,
                                                             const vector<float>& arrVertices,
                                                             const vector<float>& arrNormals);

    //Convert from Homogenous to XYZ and reverse
    static U32 ConvertFloat4ToFloat3(const vector<float>& arrInFloat4, vector<float>& arrOutFloat3);
    static U32 ConvertFloat3ToFloat4(const vector<float>& arrInFloat3, vector<float>& arrOutFloat4);


protected:
	//Releases all buffer objects for rendering
	virtual void cleanup();


protected:

	//Index to the buffer objects
	U32 m_vboVertex;
	U32 m_vboColor;
	U32 m_vboNormal;
	U32 m_vboTexCoord;
	U32 m_iboFaces;

	//Flags
	bool m_isValidVertex;
	bool m_isValidNormal;
	bool m_isValidColor;
	bool m_isValidTexCoord;
	bool m_isValidIndex;

	bool m_bWireFrame;
	U32 m_uShaderEffectProgram;
	ShaderEffectType m_effectType;

	//Count
	U32 m_ctFaceElements;
	U32 m_ctVertices;

	int m_stepVertex;
	int m_stepColor;
	int m_stepTexCoord;
	int m_stepFace;
	int m_faceMode;
};


#endif /* PS_MESHGLBUFFER_H_ */
