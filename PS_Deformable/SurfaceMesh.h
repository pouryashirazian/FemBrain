#ifndef PS_SURFACEMESH_H
#define PS_SURFACEMESH_H

#include <vector>
#include "PS_Graphics/PS_Vector.h"
#include "PS_Graphics/PS_Box.h"
#include "PS_Graphics/PS_GLMeshBuffer.h"

using namespace std;
using namespace PS;
using namespace PS::MATH;


namespace PS {
namespace FEM {

/*!
 * Synopsis: Surface mesh is the BRep for the volume mesh
 * The same vertex list is used in Surface mesh and Volume Mesh
 */


class SurfaceMesh : public SceneNode {
public:
	enum DRAW_TYPE {dtNone = 0, dtFaces = 1, dtEdges = 2, dtVertices = 4, dtFixedVertices = 8, dtPickedVertices = 16};

	SurfaceMesh();
	SurfaceMesh(const char* chrObjFilePath);
	virtual ~SurfaceMesh();

	void draw();

	//Draw Mode
	void setDrawMode(int mode);
	int getDrawMode() const;

	//Count
	U32 getVertexCount() const;
	U32 getFaceCount() const;


	//Access
	vec3d vertexAt(U32 idx) const;
	vec3d vertexRestPosAt(U32 idx) const;
	vec3d normalAt(U32 idx) const;
	vec3u32 faceAt(U32 idx) const;
	/*!
	 * Returns the vertex associated with the corner of the face
	 */
	vec3d faceVertexAt(U32 idxFace, U8 idxWhichCorner) const;

	//Finds the closest vertex to a query point
	int findClosestVertex(const vec3d& query, double& dist, vec3d& outP);

	//Apply Displacements
	void applyDisplacements(double * u);

	//Topology Modification
	void resetToRest();
	bool removeVertices(const vector<U32>& vertices);
	bool removeTriangles(const vector<U32>& triangles);

	bool addVertices(const vector<double>& vertices, const vector<double>& normals);
	bool addTriangles(const vector<U32>& elements);

	//Topology Helpers
	int trianglesAroundVertex(U32 idxVertex, vector<U32>& outTriangles);

private:
	bool readFromDisk(const char* chrObjFilePath);
	bool computeAABB();
	void setupDrawBuffers();
	void cleanupDrawBuffers();

private:
	GLMemoryBuffer* m_lpMemVertices;
	GLMemoryBuffer* m_lpMemNormal;
	GLMemoryBuffer* m_lpMemFaces;
	int m_drawMode;


	//Vertices
	vector<double> m_vRestPos;
	vector<double> m_vCurPos;
	vector<double> m_vNormals;

	//Face Elements (Triangles or Quads)
	vector<U32> m_faces;
	vector<U32> m_vFixedVertices;
};

}
}

#endif
