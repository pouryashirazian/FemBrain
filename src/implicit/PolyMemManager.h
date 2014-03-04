/*
 * PS_PolyMemManager.h
 *
 *  Created on: 2012-01-16
 *      Author: pourya
 */

#ifndef PS_POLYMEMMANAGER_H_
#define PS_POLYMEMMANAGER_H_
#include "Polygonizer.h"

namespace PS{
namespace SIMDPOLY{

//Vertex Buffer Objects To Draw
struct MESH_BUFFER_OBJECTS{
	//Index to the buffer objects
	U32 vboVertex;
	U32 vboColor;
	U32 vboNormal;
	U32 iboFaces;

	//Count
	U32 ctTriangles;
	U32 ctVertices;

	bool bIsValid;

	//releases all buffer objects for rendering
	void cleanup();
};


//Structure Holding Polygonization Partition Units
class PolyMPUs{
public:
	PolyMPUs()	{ init();}
	~PolyMPUs() {cleanup();}

	void init();
	bool allocate(const vec3i& workDim);
	void cleanup();

	int setLowerVertex(const vec3f& start, float mpuSide);

	//U32 memUsage() const

	vec3i getWorkDim() const {return m_workDim;}
	U32 countWorkUnits() const {return m_ctWorkUnits;}
private:
	U32 m_ctAllocated;
	vec3i m_workDim;
	U32 m_ctWorkUnits;

public:
	MPU* lpMPUs;
	MPUGLOBALMESH globalMesh;
};


//SimdPoly
class SimdPoly{
public:
	SimdPoly();
	~SimdPoly() {cleanup();}

	void allocate();
	void cleanup();
	void printModelInfo();
	//void printBBoxInfo();

	//Read Model
	bool readModel(const char* lpFilePath);

	/*!
	 * Create a BlobTree of spheres.
	 * @param ctSpheres Number of spheres to produce.
	 */
	void createRandomSpheresModel(U32 ctSpheres);

	/*!
	 * Prepares bounding boxes of all primitives and operators in the BlobTree.
	 * Compute the union of all primitive bounding boxes as the model BBox.
	 * Counts MPUs needed and allocates them.
	 * @param cellsize the cubic cell size for polygonization
	 * @return success or an error code
	 */
	int prepareBBoxes(float cellsize);

	/*!
	 * Polygonizes a BlobTree model using SIMD optimized polygonizer and records all
	 * related stats.
	 * @param cellsize the cubic cell side length for polygonization
	 * @param bScalarRun running code in scalar mode (with no SIMD optimization)
	 * @param lpProcessStats pointer to an array of MPUSTATS to record poylgonization statistics.
	 * @return success or an error code
	 */
	int polygonize(float cellsize, bool bScalarRun,
				   MPUSTATS* lpProcessStats);

	//Produces Single Mesh vertex and element buffer objects
	bool extractSingleMeshObject();

	/*!
	 * Draw the mesh using accelerated memory buffer objects in OpenGL
	 * @param bDrawWireFrame if set draws in wireframe mode default is false
	 */
	void drawMesh(bool bDrawWireFrame = false);

	//Draw Mesh Normals for Debug
	void drawMeshNormals();


	int measureQuality() const;

	//Access
	PolyMPUs* getMPUs() {return &m_polyMPUs;}
	SOABlobPrims* getBlobPrims() const {return m_lpBlobPrims;}
	SOABlobOps* getBlobOps() const {return m_lpBlobOps;}
	SOABlobNodeMatrices* getMtxNode() const {return m_lpMtxNode;}
	SOABlobBoxMatrices* getMtxBox() const {return m_lpMtxBox;}

private:

	int prepareOpBBox(int idxOp, vec3f& boxLo, vec3f& boxHi);

private:
	SOABlobPrims* m_lpBlobPrims;
	SOABlobOps* m_lpBlobOps;
	SOABlobNodeMatrices* m_lpMtxNode;
	SOABlobBoxMatrices* m_lpMtxBox;
	PolyMPUs m_polyMPUs;
	MESH_BUFFER_OBJECTS m_outputMesh;

	void* m_lpMemBlock;
	U32 m_szInputData;
};



}
}

#endif /* PS_POLYMEMMANAGER_H_ */
