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
#include <vector>

using namespace std;
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

enum VertexAttribType {vatPosition, vatNormal, vatColor, vatTexCoord};

enum FaceType {ftLines = 0x0001, ftLineLoop = 0x0002, ftLineStrip = 0x0003,
			   ftTriangles = 0x0004, ftTriangleStrip = 0x0005, ftTriangleFan = 0x0006,
			   ftQuads = 0x0007, ftQuadStrip = 0x0008, ftPolygon = 0x0009};

enum ShaderEffectType {setFixedFunction, setCustom};

/*!
 * Synopsis: GLMeshBuffer Simplifies drawing geometries using GPU Buffers.
 * Types of buffer objects are: Vertex, Color, TexCoord, Normal and Index
 */
class GLMeshBuffer{
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

	//Draw
	virtual void draw();

protected:
	//Releases all buffer objects for rendering
	void cleanup();


protected:

	//Index to the buffer objects
	U32 vboVertex;
	U32 vboColor;
	U32 vboNormal;
	U32 vboTexCoord;
	U32 iboFaces;


private:
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

	//Flags
	bool m_isValidVertex;
	bool m_isValidNormal;
	bool m_isValidColor;
	bool m_isValidTexCoord;
	bool m_isValidIndex;
};


#endif /* PS_MESHGLBUFFER_H_ */
