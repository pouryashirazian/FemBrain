/*
 * PS_PolyMemManager.cpp
 *
 *  Created on: 2012-01-16
 *      Author: pourya
 */
#include "PS_PolyMemManager.h"
#include "PS_ReadSceneModel.h"
#include "PS_MATRIX4.h"

#include <vector>
#include <GL/glew.h>

namespace PS{
namespace SIMDPOLY{
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void MESH_BUFFER_OBJECTS::cleanup()
{
	if(bIsValid)
	{
		glDeleteBuffers(1, &vboVertex);
		glDeleteBuffers(1, &vboColor);
		glDeleteBuffers(1, &vboNormal);
		glDeleteBuffers(1, &iboFaces);
		bIsValid = false;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void PolyMPUs::init()
{
	lpMPUs = NULL;
	m_ctAllocated = 0;
	m_workDim 	  = svec3i(0, 0, 0);
	m_ctWorkUnits = 0;
}

bool PolyMPUs::allocate(const svec3i& workDim)
{
	U32 ctNeeded = workDim.x * workDim.y * workDim.z;
	this->m_workDim = workDim;
	this->m_ctWorkUnits = ctNeeded;
	if((lpMPUs != NULL)&&(m_ctAllocated >= ctNeeded))
	{
		printf("Already have enough memory for MPUs. Needed=%u Allocated=%u\n", m_ctWorkUnits, m_ctAllocated);
		return false;
	}
	cleanup();
	printf("Allocating memory for MPUs. Dim=[%d %d %d], Needed=%u \n", m_workDim.x, m_workDim.y, m_workDim.z, m_ctWorkUnits);

	lpMPUs = AllocAligned<MPU>(ctNeeded);

	U32 szPerComponent = PS_SIMD_PADSIZE(MPU_MESHPART_VERTEX_STRIDE * ctNeeded);
	//void* lpVertexMemBlock = AllocAligned(szPerComponent * 3);
	//globalMesh.vPos = reinterpret_cast<float*>((U64)m_lpMemBlock + sizeof(SOABlobPrims));
	globalMesh.vColor = AllocAligned<float>(szPerComponent);
	globalMesh.vNorm  = AllocAligned<float>(szPerComponent);
	globalMesh.vPos   = AllocAligned<float>(szPerComponent);

	U32 szPerTriangle = PS_SIMD_PADSIZE(MPU_MESHPART_TRIANGLE_STRIDE * ctNeeded);
	globalMesh.vTriangles = AllocAligned<U16>(szPerTriangle);

	m_ctAllocated = ctNeeded;
	return true;
}

int PolyMPUs::setLowerVertex(const svec3f& start, float mpuSide)
{
	if((m_ctAllocated == 0)||(m_ctWorkUnits == 0))
		return 0;

	U32 idxMPU = 0;
	//Create all MPUs
	//Create all intersecting MPUs
	for (int i = 0; i < m_workDim.x; i++) {
		for (int j = 0; j < m_workDim.y; j++) {
			for (int k = 0; k < m_workDim.z; k++)
			{
				lpMPUs[idxMPU].bboxLo = vadd3f(start, svec3f((float) i * mpuSide, (float) j * mpuSide, (float) k * mpuSide));
				lpMPUs[idxMPU].idxGlobalID = idxMPU;
				idxMPU++;
			}
		}
	}

	return idxMPU;
}

void PolyMPUs::cleanup()
{
	printf("Free memory for MPUs.\n");
	FreeAligned(lpMPUs);
	lpMPUs = 0;

	FreeAligned(globalMesh.vColor);
	FreeAligned(globalMesh.vNorm);
	FreeAligned(globalMesh.vPos);
	FreeAligned(globalMesh.vTriangles);
	globalMesh.vColor = NULL;
	globalMesh.vNorm = NULL;
	globalMesh.vPos = NULL;
	globalMesh.vTriangles = NULL;

	m_ctAllocated = 0;
}



//////////////////////////////////////////////////////////////////////////////////////////////
SimdPoly::SimdPoly()
{
	m_lpMemBlock = NULL;
	m_szInputData = 0;
}

bool SimdPoly::readModel(const char* lpFilePath)
{
	ModelReader* reader = new ModelReader(*this);
	int res = reader->read(lpFilePath);
	SAFE_DELETE(reader);

	return (res == MODELREAD_SUCCESS);
}

void SimdPoly::createRandomSpheresModel(U32 ctSpheres)
{
	svec3f animBoxLo = svec3f(-2.0f, -2.0f, -2.0f);
	svec3f animBoxHi = svec3f(2.0f, 2.0f, 2.0f);
	svec3f pos;
	svec3f color;

	//Should be less than PADSIZE
	m_lpBlobPrims->count = ctSpheres;
	for(U32 i=0; i<m_lpBlobPrims->count; i++)
	{
		pos.x = RandRangeT<float>(animBoxLo.x, animBoxHi.x);
		pos.y = RandRangeT<float>(animBoxLo.y, animBoxHi.y);
		pos.z = RandRangeT<float>(animBoxLo.z, animBoxHi.z);

		color = pos;
		vnormalize3f(color);

		m_lpBlobPrims->posX[i] = pos.x;
		m_lpBlobPrims->posY[i] = pos.y;
		m_lpBlobPrims->posZ[i] = pos.z;

		m_lpBlobPrims->colorX[i] = color.x;
		m_lpBlobPrims->colorY[i] = color.y;
		m_lpBlobPrims->colorZ[i] = color.z;
	}
}

void SimdPoly::printModelInfo()
{
	if(m_lpMemBlock == NULL) return;
	printf("Prims# %d, Ops# %d, Prim Matrices# %d, Box Matrices# %d.\n",
	m_lpBlobPrims->count, m_lpBlobOps->count, m_lpMtxNode->count, m_lpMtxBox->count);
}

void SimdPoly::allocate()
{
	printf("Allocating memory for input Blobtree.\n");
	m_szInputData = sizeof(SOABlobPrims) + sizeof(SOABlobOps) + sizeof(SOABlobNodeMatrices) + sizeof(SOABlobBoxMatrices) + PS_SIMD_ALIGN_SIZE * 4;
	m_lpMemBlock = AllocAligned(m_szInputData);

	m_lpBlobPrims 		= reinterpret_cast<SOABlobPrims*>(m_lpMemBlock);
	m_lpBlobOps 		= reinterpret_cast<SOABlobOps*>((U64)m_lpMemBlock + sizeof(SOABlobPrims));
	m_lpMtxNode 	= reinterpret_cast<SOABlobNodeMatrices*>((U64)m_lpBlobOps + sizeof(SOABlobOps));
	m_lpMtxBox = reinterpret_cast<SOABlobBoxMatrices*>((U64)m_lpMtxNode + sizeof(SOABlobNodeMatrices));
}

void SimdPoly::cleanup()
{
	printf("Free memory for input Blobtree.\n");
	FreeAligned(m_lpMemBlock);
	m_lpMemBlock = NULL;
	m_polyMPUs.cleanup();
	m_outputMesh.cleanup();
}

//Bounding Boxes computation and MPU Allocation
int SimdPoly::prepareBBoxes(float cellsize)
{
	svec3f opBoxLo, opBoxHi;
	int res = PrepareAllBoxes(*m_lpBlobOps, *m_lpBlobPrims, *m_lpMtxBox);

	//Prepare MPUs Needed for processing
	svec3i dim = CountMPUNeeded(cellsize, m_lpBlobPrims->bboxLo, m_lpBlobPrims->bboxHi);
	m_polyMPUs.allocate(dim);
	m_polyMPUs.setLowerVertex(m_lpBlobPrims->bboxLo, (GRID_DIM - 1)* cellsize);

	return res;
}




//Polygonize
int SimdPoly::polygonize(float cellsize, bool bScalarRun, MPUSTATS* lpProcessStats)
{
	return Polygonize(cellsize,
					  bScalarRun,
					  *m_lpBlobOps,
					  *m_lpBlobPrims,
					  *m_lpMtxNode,
					  m_polyMPUs.countWorkUnits(),
					  m_polyMPUs.lpMPUs,
					  &m_polyMPUs.globalMesh,
					  lpProcessStats);
}

//Extract a single mesh from all those MPUs
bool SimdPoly::extractSingleMeshObject()
{
	U32 ctMPUs = m_polyMPUs.countWorkUnits();
	if(ctMPUs == 0)
		return false;

	//Cleanup previous mesh
	m_outputMesh.cleanup();

	U32* arrVertexCount = new U32[ctMPUs];
	U32* arrFaceCount = new U32[ctMPUs];

	U32* arrVertexCountScanned = new U32[ctMPUs + 1];
	U32* arrFaceCountScanned = new U32[ctMPUs + 1];

	//Scan Arrays of Vertices Count and Faces Count
	for(U32 i=0; i < ctMPUs; i++)
	{
		arrVertexCount[i] = m_polyMPUs.lpMPUs[i].ctVertices;
		arrFaceCount[i] = m_polyMPUs.lpMPUs[i].ctTriangles;
	}

	//Prefix Sum Scan
	arrVertexCountScanned[0] = 0;
	arrFaceCountScanned[0] = 0;
	for(U32 i=1; i <= ctMPUs; i++)
	{
		arrVertexCountScanned[i] = arrVertexCountScanned[i - 1] + arrVertexCount[i - 1];
		arrFaceCountScanned[i] = arrFaceCountScanned[i - 1] + arrFaceCount[i - 1];
	}

	U32 ctTriangles = m_outputMesh.ctTriangles = arrFaceCountScanned[ctMPUs];
	U32 ctVertices = m_outputMesh.ctVertices = arrVertexCountScanned[ctMPUs];
	float* arrVertices = new float[ctVertices * 3];
	float* arrNormals  = new float[ctVertices * 3];
	float* arrColors   = new float[ctVertices * 3];
	U32* arrFaces = new U32[ctTriangles * 3];

	for(U32 i=0; i < ctMPUs; i++)
	{
		if(arrVertexCount[i] > 0)
		{
			memcpy(&arrVertices[arrVertexCountScanned[i] * 3], &m_polyMPUs.globalMesh.vPos[i * MPU_MESHPART_VERTEX_STRIDE], sizeof(float) * 3 * arrVertexCount[i]);
			memcpy(&arrColors[arrVertexCountScanned[i] * 3], &m_polyMPUs.globalMesh.vColor[i * MPU_MESHPART_VERTEX_STRIDE], sizeof(float) * 3 * arrVertexCount[i]);
			memcpy(&arrNormals[arrVertexCountScanned[i] * 3], &m_polyMPUs.globalMesh.vNorm[i * MPU_MESHPART_VERTEX_STRIDE], sizeof(float) * 3 * arrVertexCount[i]);

			U32 offsetFace = arrFaceCountScanned[i] * 3;
			U32 offsetVertex = arrVertexCountScanned[i];
			U32 jMax = arrFaceCount[i]*3;
			for(U32 j=0; j<jMax; j++)
			{
				arrFaces[offsetFace + j] = m_polyMPUs.globalMesh.vTriangles[i * MPU_MESHPART_TRIANGLE_STRIDE + j] + offsetVertex;
			}
		}
	}




	//Buffers
	glGenBuffers(1, &m_outputMesh.vboVertex);
	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboVertex);
	glBufferData(GL_ARRAY_BUFFER, ctVertices*3 * sizeof(float), arrVertices, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &m_outputMesh.vboColor);
	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboColor);
	glBufferData(GL_ARRAY_BUFFER, ctVertices*3 * sizeof(float), arrColors, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &m_outputMesh.vboNormal);
	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboNormal);
	glBufferData(GL_ARRAY_BUFFER, ctVertices*3 * sizeof(float), arrNormals, GL_DYNAMIC_DRAW);


	glGenBuffers(1, &m_outputMesh.iboFaces);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_outputMesh.iboFaces);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ctTriangles *3 * sizeof(U32), arrFaces, GL_DYNAMIC_DRAW);


	m_outputMesh.bIsValid = true;

	SAFE_DELETE(arrVertexCount);
	SAFE_DELETE(arrFaceCount);
	SAFE_DELETE(arrVertexCountScanned);
	SAFE_DELETE(arrFaceCountScanned);

	SAFE_DELETE(arrColors);
	SAFE_DELETE(arrVertices);
	SAFE_DELETE(arrNormals);
	SAFE_DELETE(arrFaces);
	return 1;

}

void SimdPoly::drawMeshNormals()
{
	if(!m_outputMesh.bIsValid)	return;

	U32 ctMPUs = m_polyMPUs.countWorkUnits();

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);
	for(U32 i=0; i<ctMPUs; i++)
	{
		U32 ctVertices = m_polyMPUs.lpMPUs[i].ctVertices;
		if(ctVertices > 0)
		{
			for(U32 j=0; j<ctVertices; j++)
			{
				svec3f s = vload3f(&m_polyMPUs.globalMesh.vPos[i*MPU_MESHPART_VERTEX_STRIDE + j*3]);
				svec3f n = vload3f(&m_polyMPUs.globalMesh.vNorm[i*MPU_MESHPART_VERTEX_STRIDE + j*3]);
				svec3f e = vadd3f(s, vscale3f(0.3f, n));

				glVertex3f(s.x, s.y, s.z);
				glVertex3f(e.x, e.y, e.z);
			}
		}

	}

	glEnd();
	glPopAttrib();
}

void SimdPoly::drawMesh(bool bDrawWireFrame)
{
	if(!m_outputMesh.bIsValid)	return;

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	if(bDrawWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboColor);
	glColorPointer(3, GL_FLOAT, 0, 0);
	glEnableClientState(GL_COLOR_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboNormal);
	glNormalPointer(GL_FLOAT, 0, 0);
	glEnableClientState(GL_NORMAL_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, m_outputMesh.vboVertex);
	glVertexPointer(3, GL_FLOAT, 0, 0);
	glEnableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_outputMesh.iboFaces);
	glEnableClientState(GL_ELEMENT_ARRAY_BUFFER);

	glDrawElements(GL_TRIANGLES, (GLsizei)m_outputMesh.ctTriangles * 3, GL_UNSIGNED_INT, (GLvoid*)0);

	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_ELEMENT_ARRAY_BUFFER);

	glPopAttrib();
}


int SimdPoly::measureQuality() const
{
	/*
	std::vector<float> arrMinNormalAngle;
	std::vector<float> arrMaxNormalAngle;
	std::vector<float> arrAvgNormalAngle;

	U32 ctTriangles = 0;
	U32 ctVertices = 0;
	for(U32 i=0; i<m_polyMPUs.ctUsed; i++)
	{
		ctTriangles += m_polyMPUs.lpMPUs[i].ctTriangles;
		ctVertices += m_polyMPUs.lpMPUs[i].ctVertices;
	}

	for(U32 i=0; i<m_polyMPUs.ctUsed; i++)
	{
		m_polyMPUs.lpMPUs[i].triangles
	}
	*/
	return 0;
}



























}
}

