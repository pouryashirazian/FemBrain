/*
 * Cutting.cpp
 *
 *  Created on: Sep 25, 2013
 *      Author: pourya
 */
#include <utility>
#include "Cutting.h"
#include "base/FileDirectory.h"
#include "base/Logger.h"
#include "base/Profiler.h"
#include "base/DebugUtils.h"
#include "graphics/Intersections.h"
#include "Cutting_CPU.h"

using namespace PS;
using namespace PS::DEBUGTOOLS;
using namespace PS::FILESTRINGUTILS;

namespace PS {
namespace FEM {


Cutting::Cutting(Deformable* lpDeformableModel) {
	m_lpDrawEdgePoints = NULL;
	m_lpDrawFacePoints = NULL;
	m_lpDeformable = lpDeformableModel;
	m_isMemBuffersLoaded = false;
	setup();
}

Cutting::~Cutting() {
	cleanup();
}

void Cutting::setup() {
	AnsiStr strCodePath = ExtractOneLevelUp(ExtractFilePath(GetExePath()));
	AnsiStr strCuttingFP = strCodePath + AnsiStr("PS_Shaders/Cutting.cl");

	//Compute Device
	m_lpDevice = TheCLManager::Instance().device();
	int prg = m_lpDevice->addProgramFromFile(strCuttingFP, true);
	assert(prg >= 0);

	//Setup Kernels
	m_lpKernelFacePoints = m_lpDevice->addKernel(prg, "ComputePerTetFaceIntersections");
	m_lpKernelEdgePoints = m_lpDevice->addKernel(prg, "ComputePerTetEdgeIntersections");
	m_lpKernelCentroids = m_lpDevice->addKernel(prg, "ComputePerTetCentroids");
	m_lpKernelTest = m_lpDevice->addKernel(prg, "ComputeSegmentTriIntersections");

	//Create Memory Buffers
	createMemBuffers();


	//Setup Sum Scan
	m_lpOclSumScan = new SumScan();
}

void Cutting::cleanup() {
	SAFE_DELETE(m_lpDevice);
	cleanupMemBuffers();
}

void Cutting::cleanupMemBuffers() {
	if(!m_isMemBuffersLoaded)
		return;

	clReleaseMemObject(m_inMemTetMeshVertices);
	clReleaseMemObject(m_inMemTetMeshIndices);
	clReleaseMemObject(m_inMemMeshInfo);

	clReleaseMemObject(m_inoutMemEdgeFlags);
	clReleaseMemObject(m_inoutMemEdgePoints);

	clReleaseMemObject(m_inoutMemFaceFlags);
	clReleaseMemObject(m_inoutMemFacePoints);
}

bool Cutting::createMemBuffers() {
	if(!m_lpDeformable)
		return false;
	//ProfileAuto();

	//Cleanup Mem Buffers
	this->cleanupMemBuffers();

	//TetMesh Stats
	m_meshInfo.ctTets = m_lpDeformable->getMesh()->countCells();
	m_meshInfo.ctVertices = m_lpDeformable->getMesh()->countNodes();


	U32 szVertexBuffer = sizeof(float) * 4 * m_meshInfo.ctVertices;
	U32 szElementsBuffer = sizeof(U32) * 4 * m_meshInfo.ctTets;
	U32 szFacePointsBuffer = sizeof(float) * 4 * m_meshInfo.ctTets * 4;
	U32 szEdgePointsBuffer = sizeof(float) * 4 * m_meshInfo.ctTets * 6;

	//Create FacePoints VBO
	SAFE_DELETE(m_lpDrawFacePoints);
	m_lpDrawFacePoints = new  GLMemoryBuffer(mbtPosition, GL_DYNAMIC_DRAW, 4, GL_FLOAT, szFacePointsBuffer, NULL);

	//Create EdgePoints VBO
	SAFE_DELETE(m_lpDrawEdgePoints);
	m_lpDrawEdgePoints = new GLMemoryBuffer(mbtPosition, GL_DYNAMIC_DRAW, 4, GL_FLOAT, szEdgePointsBuffer, NULL);


	//Buffers
	m_inMemTetMeshVertices = m_lpDevice->createMemBuffer(szVertexBuffer, PS::CL::memReadOnly);
	m_inMemTetMeshIndices = m_lpDevice->createMemBuffer(szElementsBuffer, PS::CL::memReadOnly);
	m_inMemMeshInfo = m_lpDevice->createMemBuffer(sizeof(MeshInfo), PS::CL::memReadOnly);

	//Face
	m_inoutMemFaceFlags = m_lpDevice->createMemBuffer(sizeof(U32) * m_meshInfo.ctTets * 4, PS::CL::memReadWrite);
	m_inoutMemFacePoints = m_lpDevice->createMemBuffer(szFacePointsBuffer, PS::CL::memReadWrite);

	//Edge
	m_inoutMemEdgeFlags = m_lpDevice->createMemBuffer(sizeof(U32) * m_meshInfo.ctTets * 6, PS::CL::memReadWrite);
	m_inoutMemEdgePoints = m_lpDevice->createMemBuffer(szEdgePointsBuffer, PS::CL::memReadWrite);


	//Elements
	U32* arrIndices = new U32[m_meshInfo.ctTets * 4];
	for(U32 el=0; el<m_meshInfo.ctTets; el++) {
		for(U32 i=0; i<4; i++) {
			arrIndices[el*4 + i] = m_lpDeformable->getMesh()->getVertexIndex(el, i);
		}
	}

	m_vMeshTets.assign(arrIndices, arrIndices + m_meshInfo.ctTets * 4);

	//Vertices
	m_vMeshVertices.resize(m_meshInfo.ctVertices * 3);

	float* arrVertices = new float[m_meshInfo.ctVertices * 4];
	for(U32 i = 0; i<m_meshInfo.ctVertices; i++) {
		vec3d v = m_lpDeformable->getMesh()->vertexAt(i);
		arrVertices[i * 4] = static_cast<float>(v.x);
		arrVertices[i * 4 + 1] = static_cast<float>(v.y);
		arrVertices[i * 4 + 2] = static_cast<float>(v.z);
		arrVertices[i * 4 + 3] = 1.0f;

		//Mesh Vertices
		m_vMeshVertices[i * 3] = v.x;
		m_vMeshVertices[i * 3 + 1] = v.y;
		m_vMeshVertices[i * 3 + 2] = v.z;
	}

	//Copy to GPU Memory
	if(!m_lpDevice->enqueueWriteBuffer(m_inMemTetMeshVertices, szVertexBuffer, arrVertices))
		return false;

	if(!m_lpDevice->enqueueWriteBuffer(m_inMemTetMeshIndices, szElementsBuffer, arrIndices))
		return false;

	if(!m_lpDevice->enqueueWriteBuffer(m_inMemMeshInfo, sizeof(MeshInfo), &m_meshInfo))
		return false;


	m_isMemBuffersLoaded = true;

	//Cleanup
	SAFE_DELETE(arrVertices);
	SAFE_DELETE(arrIndices);

	return true;
}

int Cutting::computeFaceIntersections(const vec3d& s0, const vec3d& s1) {
	if(!m_isMemBuffersLoaded)
		return -1;

	size_t arrLocalIndex[3];
	size_t arrGlobalIndex[3];
	arrGlobalIndex[0] = m_meshInfo.ctTets;
	arrGlobalIndex[1] = 0;
	arrGlobalIndex[2] = 0;

	ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelFacePoints->getKernelWorkGroupSize(), arrLocalIndex);
	ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);

	//Scalpel Edge
	float scalpelEdge[8];
	scalpelEdge[0] = static_cast<float>(s0.x);
	scalpelEdge[1] = static_cast<float>(s0.y);
	scalpelEdge[2] = static_cast<float>(s0.z);
	scalpelEdge[3] = 1.0f;
	scalpelEdge[4] = static_cast<float>(s1.x);
	scalpelEdge[5] = static_cast<float>(s1.y);
	scalpelEdge[6] = static_cast<float>(s1.z);
	scalpelEdge[7] = 1.0f;

	cl_mem inMemScalpelEdge = m_lpDevice->createMemBuffer(sizeof(float) * 8, PS::CL::memReadOnly);
	if(!m_lpDevice->enqueueWriteBuffer(inMemScalpelEdge, 8 * sizeof(float), &scalpelEdge[0]))
		return -1;

	/*
__kernel void ComputePerTetFaceIntersections(__global float4* arrInVertices,
				       	     __global U32* arrInTets,
					     __constant struct MeshInfo* minfo,
					     __global float4* inScalpelEdge,
					     __global U32* arrOutFaceFlags,
					     __global float4* arrOutFacePoint)
	 */
	//FacePoints Buffer
	cl_mem glMemFacePoints = m_lpDevice->createMemBufferFromGL(m_lpDrawFacePoints->handle(), PS::CL::memReadWrite);
	m_lpDevice->enqueueAcquireGLObject(1, &glMemFacePoints);

	m_lpKernelFacePoints->setArg(0, sizeof(cl_mem), &m_inMemTetMeshVertices);
	m_lpKernelFacePoints->setArg(1, sizeof(cl_mem), &m_inMemTetMeshIndices);
	m_lpKernelFacePoints->setArg(2, sizeof(cl_mem), &m_inMemMeshInfo);
	m_lpKernelFacePoints->setArg(3, sizeof(cl_mem), &inMemScalpelEdge);
	m_lpKernelFacePoints->setArg(4, sizeof(cl_mem), &m_inoutMemFaceFlags);
	m_lpKernelFacePoints->setArg(5, sizeof(cl_mem), &glMemFacePoints);

	//Compute range kernel
	m_lpDevice->enqueueNDRangeKernel(m_lpKernelFacePoints, 1, arrGlobalIndex, arrLocalIndex);
	m_lpDevice->enqueueReleaseGLObject(1, &glMemFacePoints);


	//Wait for all commands to complete
	m_lpDevice->finishAllCommands();

	//Cleanup local mem
	clReleaseMemObject(inMemScalpelEdge);
	clReleaseMemObject(glMemFacePoints);



	//Sum-Scan to count number of intersections
	U32 ctFacePointBuffer = m_meshInfo.ctTets * 4;

	U32* lpFacePointOffsets = new U32[ctFacePointBuffer];
	m_lpDevice->enqueueReadBuffer(m_inoutMemFaceFlags, sizeof(U32) * ctFacePointBuffer, lpFacePointOffsets);
	m_lpDevice->finishAllCommands();
	//PrintArray(lpFacePointOffsets, 500);

	//SumScan to compute face-points
	m_ctFacePoints = m_lpOclSumScan->compute(lpFacePointOffsets, ctFacePointBuffer);

	if(m_ctFacePoints > 0) {
		LogInfoArg1("FACEPOINTS COMPUTED: %d", m_ctFacePoints);
	}


	SAFE_DELETE(lpFacePointOffsets);

	return m_ctFacePoints;
}

int Cutting::computeFaceCentroids() {
	if(!m_isMemBuffersLoaded)
		return -1;

	ProfileAuto();

	size_t arrLocalIndex[3];
	size_t arrGlobalIndex[3];
	arrGlobalIndex[0] = m_meshInfo.ctTets;
	arrGlobalIndex[1] = 0;
	arrGlobalIndex[2] = 0;

	ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelCentroids->getKernelWorkGroupSize(), arrLocalIndex);
	ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);


	/*
	__kernel void ComputePerTetCentroids(__global float4* arrInVertices,
					     __global U32* arrInTets,
					     __constant struct MeshInfo* minfo,
				             __global U32* arrOutFaceFlags,
				             __global float4* arrOutFaceCentroids)
	*/
	//Vertex
	U32 szFaceFlagsBuffer = sizeof(U32) * m_meshInfo.ctTets * 4;
	U32 szFacePointsBuffer = sizeof(float) * 4 * m_meshInfo.ctTets * 4;


	//FacePoints Buffer
	cl_mem outMemFacePoints = m_lpDevice->createMemBufferFromGL(m_lpDrawFacePoints->handle(), PS::CL::memReadWrite);
	m_lpDevice->enqueueAcquireGLObject(1, &outMemFacePoints);

	m_lpKernelCentroids->setArg(0, sizeof(cl_mem), &m_inMemTetMeshVertices);
	m_lpKernelCentroids->setArg(1, sizeof(cl_mem), &m_inMemTetMeshIndices);
	m_lpKernelCentroids->setArg(2, sizeof(cl_mem), &m_inMemMeshInfo);
	m_lpKernelCentroids->setArg(3, sizeof(cl_mem), &m_inoutMemFaceFlags);
	m_lpKernelCentroids->setArg(4, sizeof(cl_mem), &outMemFacePoints);

	//Compute range kernel
	m_lpDevice->enqueueNDRangeKernel(m_lpKernelCentroids, 1, arrGlobalIndex, arrLocalIndex);

	//Release
	m_lpDevice->enqueueReleaseGLObject(1, &outMemFacePoints);

	//Wait for all commands to complete
	m_lpDevice->finishAllCommands();

	//READ-Back Test
	//U32 ctProcessed = m_lpOclSumScan->compute(m_inoutMemFaceFlags, m_meshInfo.ctTets * 4);
	//printf("Number of faces affected: %d\n", ctProcessed);
	/*
	U32 szFaceFlagsBuffer = sizeof(U32) * m_meshInfo.ctTets * 4;
	U32 szFacePointsBuffer = sizeof(float) * 4 * m_meshInfo.ctTets * 4;

	U32* arrFaceFlags = new U32[m_meshInfo.ctTets * 4];
	float* arrFacePoints = new float[m_meshInfo.ctTets * 4 * 4];

	m_lpDevice->enqueueReadBuffer(m_inoutMemFaceFlags, szFaceFlagsBuffer, arrFaceFlags);
	m_lpDevice->enqueueReadBuffer(m_inoutMemFacePoints, szFacePointsBuffer, arrFacePoints);
	m_lpDevice->finishAllCommands();

	//PrintArray(arrFaceFlags, 200);
	//U32 ctProcessed = m_lpOclSumScan->compute(arrFaceFlags, m_meshInfo.ctTets * 4);
	//FacePoints
	SAFE_DELETE(m_lpDrawFacePoints);
	m_lpDrawFacePoints = new  GLMemoryBuffer(mbtPosition, GL_DYNAMIC_DRAW, 4, GL_FLOAT, szFacePointsBuffer, &arrFacePoints[0]);


	SAFE_DELETE(arrFaceFlags);
	SAFE_DELETE(arrFacePoints);
	*/

	clReleaseMemObject(outMemFacePoints);

	return 1;
}

int Cutting::computeFaceSegmentIntersectionTest() {
	if(!m_isMemBuffersLoaded)
		return -1;

	size_t arrLocalIndex[3];
	size_t arrGlobalIndex[3];
	arrGlobalIndex[0] = 1;
	arrGlobalIndex[1] = 0;
	arrGlobalIndex[2] = 0;

	ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelTest->getKernelWorkGroupSize(), arrLocalIndex);
	ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);

	//Triangle
	float tri[12];
	tri[0] = -1.0f;
	tri[1] = 0.0f;
	tri[2] = -1.0f;
	tri[3] = 1.0f;

	tri[4] = 1.0f;
	tri[5] = 0.0f;
	tri[6] = -1.0f;
	tri[7] = 1.0f;

	tri[8] = 0.0f;
	tri[9] = 0.0f;
	tri[10] = 1.0f;
	tri[11] = 1.0f;

	cl_mem inMemVertices = m_lpDevice->createMemBuffer(sizeof(float) * 12, PS::CL::memReadOnly);
	if(!m_lpDevice->enqueueWriteBuffer(inMemVertices, 12 * sizeof(float), tri))
		return -1;

	//Scalpel Edge
	float scalpelEdge[8];
	scalpelEdge[0] = 0.0f;
	scalpelEdge[1] = -1.0f;
	scalpelEdge[2] = 0.0f;
	scalpelEdge[3] = 1.0f;

	scalpelEdge[4] = 0.0f;
	scalpelEdge[5] = 1.0f;
	scalpelEdge[6] = 0.0f;
	scalpelEdge[7] = 1.0f;

	cl_mem inMemScalpelEdge = m_lpDevice->createMemBuffer(sizeof(float) * 8, PS::CL::memReadOnly);
	if(!m_lpDevice->enqueueWriteBuffer(inMemScalpelEdge, 8 * sizeof(float), &scalpelEdge[0]))
		return -1;

	//MeshInfo
	MeshInfo minfo;
	minfo.ctTets = 1;
	minfo.ctVertices = 3;
	cl_mem inMemMeshInfo = m_lpDevice->createMemBuffer(sizeof(MeshInfo), PS::CL::memReadOnly);
	if(!m_lpDevice->enqueueWriteBuffer(inMemMeshInfo, sizeof(MeshInfo), &minfo))
		return -1;

	//FacePoints
	cl_mem inoutMemFacePoints = m_lpDevice->createMemBuffer(sizeof(float) * 4 * minfo.ctTets, PS::CL::memReadWrite);

	/*
	__kernel void ComputeSegmentTriIntersections(__global float4* arrInVertices,
						     __global float4 segment[2],
						     __constant struct MeshInfo* minfo,
						     __global float4* arrOutFacePoint)
						     */
	m_lpKernelTest->setArg(0, sizeof(cl_mem), &inMemVertices);
	m_lpKernelTest->setArg(1, sizeof(cl_mem), &inMemScalpelEdge);
	m_lpKernelTest->setArg(2, sizeof(cl_mem), &inMemMeshInfo);
	m_lpKernelTest->setArg(3, sizeof(cl_mem), &inoutMemFacePoints);

	//Compute range kernel
	m_lpDevice->enqueueNDRangeKernel(m_lpKernelTest, 1, arrGlobalIndex, arrLocalIndex);

	//Wait for all commands to complete
	m_lpDevice->finishAllCommands();


	//Read-back results
	//Ground truth
	vec3f s0, s1;
	vec3f p[3];
	vec3f xyz, uvw;
	float t;

	s0.load(&scalpelEdge[0]);
	s1.load(&scalpelEdge[4]);
	for(int i=0;i<3; i++)
		p[i].load(&tri[i * 4]);

	int res = PS::INTERSECTIONS::IntersectSegmentTriangleF(s0, s1, p, t, uvw, xyz);

	float* arrFacePoints = new float[4 * minfo.ctTets];
	m_lpDevice->enqueueReadBuffer(inoutMemFacePoints, sizeof(float) * 4 * minfo.ctTets, arrFacePoints);
	m_lpDevice->finishAllCommands();
	PrintArrayF(arrFacePoints, 4 * minfo.ctTets);
	SAFE_DELETE(arrFacePoints);



	//Cleanup
	clReleaseMemObject(inMemVertices);
	clReleaseMemObject(inMemScalpelEdge);
	clReleaseMemObject(inMemMeshInfo);
	clReleaseMemObject(inoutMemFacePoints);

	return 1;
}

int Cutting::computeEdgeIntersections(const vec3d sweptQuad[4]) {
	if(!m_isMemBuffersLoaded)
		return -1;

	size_t arrLocalIndex[3];
	size_t arrGlobalIndex[3];
	arrGlobalIndex[0] = m_meshInfo.ctTets;
	arrGlobalIndex[1] = 0;
	arrGlobalIndex[2] = 0;

	ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelEdgePoints->getKernelWorkGroupSize(), arrLocalIndex);
	ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);

	//Compute swept quad
	float quad[16];
	for(int i=0; i<4; i++) {
		quad[i*4] = static_cast<float>(sweptQuad[i].x);
		quad[i*4 + 1] = static_cast<float>(sweptQuad[i].y);
		quad[i*4 + 2] = static_cast<float>(sweptQuad[i].z);
		quad[i*4 + 3] = 1.0f;
	}
	cl_mem inMemSweptQuad = m_lpDevice->createMemBuffer(sizeof(float) * 16, PS::CL::memReadOnly);
	if(!m_lpDevice->enqueueWriteBuffer(inMemSweptQuad, 16 * sizeof(float), quad))
		return -1;


	/*
	__kernel void ComputePerTetEdgeIntersections(__global float4* arrInVertices,
				       	     __global U32* arrInTets,
					     __global float4 inSweptQuad[4],
					     __constant struct MeshInfo* minfo,
					     __global U32* arrOutEdgeFlags,
					     __global float4* arrOutEdgePoints)
	 */
	m_lpKernelEdgePoints->setArg(0, sizeof(cl_mem), m_inMemTetMeshVertices);
	m_lpKernelEdgePoints->setArg(1, sizeof(cl_mem), m_inMemTetMeshIndices);
	m_lpKernelEdgePoints->setArg(2, sizeof(cl_mem), inMemSweptQuad);
	m_lpKernelEdgePoints->setArg(3, sizeof(cl_mem), m_inMemMeshInfo);
	m_lpKernelEdgePoints->setArg(4, sizeof(cl_mem), m_inoutMemEdgeFlags);
	m_lpKernelEdgePoints->setArg(5, sizeof(cl_mem), m_inoutMemEdgePoints);

	//Compute range kernel
	m_lpDevice->enqueueNDRangeKernel(m_lpKernelEdgePoints, 1, arrGlobalIndex, arrLocalIndex);

	//Wait for all commands to complete
	m_lpDevice->finishAllCommands();

	clReleaseMemObject(m_inMemMeshInfo);

	return 1;
}

int Cutting::performCut(const vec3d& edge0, const vec3d& edge1) {
	//ProfileAuto();
	//Select a swept surface for edge cutting test
	const double minSweptLength = 0.01;
	m_isSweptQuadValid = false;


	m_sweptQuad[0] = edge0;
	m_sweptQuad[1] = edge1;
	if (m_vCuttingPathEdge0.size() > 1) {
		//Loop over the path from the recently added to the first one
		for (int i = (int) m_vCuttingPathEdge0.size() - 1; i >= 0; i--) {
			double d = vec3d::distance(m_vCuttingPathEdge0[i], edge0);
			//double d2 = (m_vCuttingPath[i].first - edge0).length2();
			if (d >= minSweptLength) {
				m_sweptQuad[2] = m_vCuttingPathEdge0[i];
				m_sweptQuad[3] = m_vCuttingPathEdge1[i];
				m_isSweptQuadValid = true;
				break;
			}
		}
	}

	//Insert new scalpal position into buffer
	const U32 maxNodes = 512;
	vec3d a1, a2;

	m_vCuttingPathEdge0.push_back(edge0);
	m_vCuttingPathEdge1.push_back(edge1);

	if (m_vCuttingPathEdge0.size() > maxNodes)
		m_vCuttingPathEdge0.erase(m_vCuttingPathEdge0.begin());
	if (m_vCuttingPathEdge1.size() > maxNodes)
		m_vCuttingPathEdge1.erase(m_vCuttingPathEdge1.begin());


	if(m_isSweptQuadValid)
		performCut(edge0, edge1, m_sweptQuad);

	return 1;
}

int Cutting::performCut(const vec3d& edge0, const vec3d& edge1, vec3d sweptQuad[4]) {
	m_ctFacePoints = 0;
	m_ctEdgePoints = 0;

	//computeFaceCentroids();
	//computeFaceSegmentIntersectionTest();

	//2.Compute Face Intersections
	int res = computeFaceIntersections(edge0, edge1);

	vector<double> outFacePoints;
	ComputeFacePointsWhenCuttingTetMesh(edge0, edge1,
										m_vMeshVertices, m_vMeshTets,
										m_meshInfo.ctVertices, m_meshInfo.ctTets, m_ctFacePoints, outFacePoints);

	//3.Compute Edge Intersections
	/*
	res = computeEdgeIntersections(sweptQuad);
	if(res > 0) {

		U32 ctEdgePointBuffer = m_meshInfo.ctTets * 6;

		U32* lpEdgePointOffsets = new U32[ctEdgePointBuffer];
		m_lpDevice->enqueueReadBuffer(m_inoutMemEdgePoints, sizeof(U32) * ctEdgePointBuffer, lpEdgePointOffsets);
		PrintArray(lpEdgePointOffsets, ctEdgePointBuffer);

		//SumScan to compute edge-points
		m_ctEdgePoints = m_lpOclSumScan->compute(lpEdgePointOffsets, ctEdgePointBuffer);
		PrintArray(lpEdgePointOffsets, ctEdgePointBuffer);

		SAFE_DELETE(lpEdgePointOffsets);
	}
	*/

	return m_ctFacePoints + m_ctEdgePoints;
}

void Cutting::draw() {
	//Draw Scalpel history
	glPushAttrib(GL_ALL_ATTRIB_BITS);
		glLineWidth(1.0f);
		glColor3d(0.0, 1.0, 0.0);
		glBegin(GL_LINES);
		for(U32 i=0; i< m_vCuttingPathEdge0.size(); i+=16) {
			glVertex3dv(m_vCuttingPathEdge0[i].cptr());
			glVertex3dv(m_vCuttingPathEdge1[i].cptr());
		}
		glEnd();
	glPopAttrib();

	//Draw Swept Quad
	//if(m_isSweptQuadValid)
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColor3d(0.7, 0.7, 0.7);
			glBegin(GL_QUADS);
				glVertex3dv(m_sweptQuad[0].cptr());
				glVertex3dv(m_sweptQuad[1].cptr());
				glVertex3dv(m_sweptQuad[3].cptr());
				glVertex3dv(m_sweptQuad[2].cptr());
			glEnd();
		glPopAttrib();
	}

	if(m_lpDrawFacePoints) {
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glPointSize(2.0f);
		glColor3f(1.0f, 0.0f, 0.0f);
			m_lpDrawFacePoints->attach();
			glDrawArrays(GL_POINTS, 0, m_meshInfo.ctTets * 4);
			m_lpDrawFacePoints->detach();
		glPopAttrib();
	}


}

}
}



