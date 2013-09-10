/*
 * PS_RBF.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: pourya
 */
#include "PS_RBF.h"

#include "PS_Base/PS_Logger.h"
#include "PS_Base/PS_FileDirectory.h"
#include "_CellConfigTable.h"
#include "_CellConfigTableCompact.h"
#include "PS_Graphics/PS_DebugUtils.h"
#include "PS_Graphics/CLMeshBuffer.h"
#include "PS_Graphics/PS_GLFuncs.h"

#include <algorithm>
#include <ctime>
#include <cstdlib>
#undef Success

//Include from 3rdparty source code directory instead of installation
#include  "3rdparty/eigen/Eigen/Dense"

using namespace PS::FILESTRINGUTILS;
using Eigen::MatrixXd;

namespace PS {
namespace HPC {

FastRBF::FastRBF():m_ctTotalInterpolationNodes(0), m_offSurfaceLen(DEFAULT_OFFSURFACE_LEN) {
	init();
}

FastRBF::FastRBF(GPUPoly* lpGPUPoly):m_ctTotalInterpolationNodes(0), m_offSurfaceLen(DEFAULT_OFFSURFACE_LEN) {
	init();
	setupFromImplicit(lpGPUPoly);
}

FastRBF::FastRBF(Mesh* lpMesh):m_ctTotalInterpolationNodes(0), m_offSurfaceLen(DEFAULT_OFFSURFACE_LEN) {
	init();
	setupFromMesh(lpMesh);
}


FastRBF::~FastRBF() {
	GLMeshBuffer::cleanup();

	//Cleanup rest pos vertices
	clReleaseMemObject(m_inoutMemRestPos);

	//Clear MC Tables
	clReleaseMemObject(m_inMemTriangleTable);
	clReleaseMemObject(m_inMemVertexCountTable);

	//Clear Mem Objects
	SAFE_DELETE(m_lpGPU);
	SAFE_DELETE(m_lpOclSumScan);
}

int FastRBF::init() {
	//Reading kernels for opencl
	DAnsiStr strCodePath = ExtractFilePath(GetExePath());
	strCodePath = ExtractOneLevelUp(strCodePath);

	DAnsiStr strPolyFP = strCodePath + DAnsiStr("PS_Shaders/RBFRendering.cl");

	LogInfo("1.Setup compute device. Prefer AMD GPU.");
	m_lpGPU = new ComputeDevice(ComputeDevice::dtGPU, true, false, "AMD");
	m_lpGPU->printInfo();

	LogInfo("2.Compile OpenCL polygonizer program.");
	ComputeProgram* lpProgramPoly = m_lpGPU->tryLoadBinaryThenCompile(strPolyFP.cptr());
	assert(lpProgramPoly != NULL);


	//Build Kernel
	LogInfo("3.Setup Kernel Functions.");

	//Polygonizer
//	m_lpKernelComputeEdgeTable = lpProgramPoly->addKernel("ComputeEdgeTable");
//	m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeEdgeTable);
//
//	m_lpKernelComputeVertexAttribs = lpProgramPoly->addKernel("ComputeVertexAttribs");
//	m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeVertexAttribs);
//
//	m_lpKernelComputeCellConfigs = lpProgramPoly->addKernel("ComputeCellConfigs");
//	m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeCellConfigs);
//
//	m_lpKernelComputeElements = lpProgramPoly->addKernel("ComputeElements");
//	m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeElements);

	//Apply Deformations to Polygonized Mesh
	m_lpKernelApplyDeformations = lpProgramPoly->addKernel("ApplyVertexDeformations");
	m_lpGPU->getKernelWorkgroupSize(m_lpKernelApplyDeformations);

	m_lpKernelComputeFieldArray = lpProgramPoly->addKernel("ComputeRBFPrimFieldArray");
	m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeFieldArray);

//	m_lpKernelComputeFieldArray = lpProgramPoly->addKernel("ComputeFieldArray");
//	m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeFieldArray);

//	m_lpKernelComputeOffSurfacePointsAndFields = lpProgramPoly->addKernel("ComputeOffSurfacePointsAndFields");
//	m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeOffSurfacePointsAndFields);

	//Load Marching Cubes tables
	cl_image_format imageFormat;
	imageFormat.image_channel_order = CL_R;
	imageFormat.image_channel_data_type = CL_UNSIGNED_INT8;

#ifdef CL_API_SUFFIX__VERSION_1_2
	cl_int errNum;
	cl_image_desc desc;
	desc.image_type = CL_MEM_OBJECT_IMAGE2D;
	desc.image_width = 256;
	desc.image_height = 1;
	desc.image_depth = 1;
	desc.image_array_size = 1;
	desc.image_row_pitch = 0;
	desc.num_mip_levels = 0;
	desc.num_samples = 0;
	desc.buffer = NULL;

	m_inMemVertexCountTable = clCreateImage(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
				  	  	  	  	  	     &imageFormat, &desc, (void*) g_numVerticesTableCompact, &errNum );
	if(errNum != CL_SUCCESS)
		return ERR_GPUPOLY_TRITABLE_NOT_READ;

	desc.image_width = 16;
	desc.image_height = 256;
	m_inMemTriangleTable = clCreateImage(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
												&imageFormat, &desc, (void*) g_triTableCompact, &errNum);
#else
	cl_int errNum;
	m_inMemVertexCountTable = clCreateImage2D(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
												   &imageFormat, 256, 1, 0, (void*) g_numVerticesTableCompact, &errNum);
	if(errNum != CL_SUCCESS)
	{
		LogErrorArg1("Failed to create VertexCountTable! Ocl Error: %s", ComputeDevice::oclErrorString(errNum));
		return ERR_GPUPOLY_VERTEXTABLE_NOT_READ;
	}

	//Triangle Count Table
	m_inMemTriangleTable = clCreateImage2D(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
										   &imageFormat, 16, 256, 0, (void*) g_triTableCompact, &errNum);
	if(errNum != CL_SUCCESS)
	{
		LogErrorArg1("Failed to create TriableTable! Ocl Error: %s", ComputeDevice::oclErrorString(errNum));
		return ERR_GPUPOLY_TRITABLE_NOT_READ;
	}
#endif

	m_lpOclSumScan = new SumScan();
	return 1;
}

//Polygonize RBF Function to create the mesh
int FastRBF::polygonize(float cellsize) {

	return -1;
}

//My rand func
template<typename T>
T myrandom (T i) { return std::rand()%i;}

bool FastRBF::randomizeInterpolationNodes() {
	if(m_ctTotalInterpolationNodes == 0 || m_interpolationNodes.size() == 0)
		return false;
	assert(m_ctTotalInterpolationNodes * 3 == m_interpolationNodes.size());

	std::srand(U32 (std::time(0)));
	std::vector<U32> vIndices;
	vIndices.resize(m_ctTotalInterpolationNodes);
	for(U32 i=0; i<m_ctTotalInterpolationNodes; i++)
		vIndices[i] = i;

	//using built-in random generator
	std::random_shuffle(vIndices.begin(), vIndices.end());
	//PS::DEBUG::PrintArray(&vIndices[0], 200);

	//std::random_shuffle(vIndices.begin(), vIndices.end(), myrandom<U32>);
	vector<float> tempNodes;
	vector<float> tempFields;
	tempNodes.resize(m_interpolationNodes.size());
	tempFields.resize(m_ctTotalInterpolationNodes);
	for(U32 i=0; i<m_ctTotalInterpolationNodes; i++) {
		U32 idx = vIndices[i];
		tempNodes[i*3] = m_interpolationNodes[idx*3];
		tempNodes[i*3 + 1] = m_interpolationNodes[idx*3 + 1];
		tempNodes[i*3 + 2] = m_interpolationNodes[idx*3 + 2];
		tempFields[i] = m_fields[idx];
	}

	m_interpolationNodes.assign(tempNodes.begin(), tempNodes.end());
	m_fields.assign(tempFields.begin(), tempFields.end());
	return true;
}

bool FastRBF::copyVertexBufferToRestPos() {

	if(m_ctVertices == 0)
		return false;
	if(m_stepVertex != 4) {
		LogError("Vertices should be in the homogenous format XYZW for the displacement kernel to work!");
		return false;
	}

	U32 szVertexBuffer = m_ctVertices * m_stepVertex * sizeof(float);
	m_inoutMemRestPos = m_lpGPU->createMemBuffer(szVertexBuffer, ComputeDevice::memReadWrite);

	//Get memory handle to gl vertex
	cl_mem inMemMeshVertex = m_lpGPU->createMemBufferFromGL(m_vboVertex, ComputeDevice::memReadOnly);

	m_lpGPU->enqueueAcquireGLObject(1, &inMemMeshVertex);
	m_lpGPU->enqueueCopyBuffer(inMemMeshVertex, m_inoutMemRestPos, 0, 0, szVertexBuffer);
	m_lpGPU->enqueueReleaseGLObject(1, &inMemMeshVertex);
	m_lpGPU->finishAllCommands();

	clReleaseMemObject(inMemMeshVertex);

	return true;
}

bool FastRBF::setupFromImplicit(GPUPoly* lpGPUPoly) {
	if(!lpGPUPoly)
		return false;
	//Setup mesh
	this->cleanup();
	CLMeshBuffer::CopyMeshBufferCL(m_lpGPU, lpGPUPoly, this);

	//Copy vertex buffer to rest pos
	copyVertexBufferToRestPos();

	//Read-back vertices and normals
	U32 ctVertices;
	vector<float> vSurfXYZ;
	vector<float> normals;
	lpGPUPoly->readBackNormals(ctVertices, vSurfXYZ, normals);

	//U32 ctOffsurfaceVertices = 2*ctVertices;
	vector<float> vOffSurfXYZF;
	vOffSurfXYZF.resize(2 * ctVertices * 4);
	for(U32 i=0; i < ctVertices; i++) {
		vec3f ptOut = vec3f(&vSurfXYZ[i * 3]) + vec3f(&normals[i * 3]) * m_offSurfaceLen;
		vec3f ptIn = vec3f(&vSurfXYZ[i * 3]) - vec3f(&normals[i * 3]) * m_offSurfaceLen;

		vOffSurfXYZF[i * 8] = ptOut.x;
		vOffSurfXYZF[i * 8 + 1] = ptOut.y;
		vOffSurfXYZF[i * 8 + 2] = ptOut.z;
		vOffSurfXYZF[i * 8 + 3] = 1.0;

		vOffSurfXYZF[i * 8 + 4] = ptIn.x;
		vOffSurfXYZF[i * 8 + 5] = ptIn.y;
		vOffSurfXYZF[i * 8 + 6] = ptIn.z;
		vOffSurfXYZF[i * 8 + 7] = 1.0;
	}

	lpGPUPoly->computeFieldArray(ctVertices * 2, 4, vOffSurfXYZF);

//	U32 ctOffsurfaceVertices;
//	vector<float> vOffSurfXYZF;
//	m_lpGPUPoly->computeOffSurfacePointsAndFields(2, 0.1f, ctOffsurfaceVertices, vOffSurfXYZF);
//	printf("\nVertices: %d, OFFSURF Size: %d\n", ctOffsurfaceVertices, vOffSurfXYZF.size());
//	PrintArrayF(&vOffSurfXYZF[0], vOffSurfXYZF.size());

//	vec3f ptOut = vec3f(&vOffSurfXYZF[0]);
//	vec3f ptIn = vec3f(&vOffSurfXYZF[4]);
//	vec3f ptSurf = vec3f(&vSurfXYZ[0]);
//	printf("\n Dist1: %.2f, Dist2: %.2f \n", vec3f::distance(ptSurf, ptOut), vec3f::distance(ptSurf, ptIn));
//	printf("\n Out and In Vertices: \n");
//	PrintArrayF(&vOffSurfXYZF[0], 8);
//	printf("\n Surface Vertex: \n");
//	PrintArrayF(&vSurfXYZ[0], 3);

	m_interpolationNodes.reserve(ctVertices * 2 * 3);
	m_fields.reserve(ctVertices*2);

	//Every other vertex will have its off-surface points
	for(U32 i=0; i<ctVertices; i++) {
		m_interpolationNodes.push_back(vSurfXYZ[i * 3]);
		m_interpolationNodes.push_back(vSurfXYZ[i * 3 + 1]);
		m_interpolationNodes.push_back(vSurfXYZ[i * 3 + 2]);
		m_fields.push_back(ISO_VALUE - ISO_VALUE);
		if(i%2 == 0) {
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8]);
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 1]);
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 2]);
			m_fields.push_back(vOffSurfXYZF[i*8 + 3] - ISO_VALUE);

			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 4]);
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 5]);
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 6]);
			m_fields.push_back(vOffSurfXYZF[i*8 + 7] - ISO_VALUE);
		}
	}

	//Print Fields
	//PS::DEBUG::PrintArrayF(&m_fields[0], 200);


	//Number of interpolation m_centers
	m_ctTotalInterpolationNodes = m_interpolationNodes.size() / 3;
	return computeInterpolationFunction(MATHMIN(m_ctTotalInterpolationNodes, DEFAULT_START_CENTERS_COUNT));
}

bool FastRBF::setupFromMesh(Mesh* lpMesh) {
	if(lpMesh == NULL)
		return false;

	if(lpMesh->countNodes() == 0)
		return false;

	//Read-back vertices and normals
	MeshNode* lpNode = lpMesh->getNode(0);
	if(lpNode->countVertices() == 0)
		return false;
	if(lpNode->getUnitFace() != 3) {
		LogErrorArg1("Expected a triangle mesh but received a mesh with %d corners.", lpNode->getUnitFace());
		return false;
	}


	//Setup Mesh
	this->m_bbox = lpNode->computeBoundingBox();
	this->cleanup();

	//Convert to homogenous format since we apply deformations in OpenCL using float4
	vector<float> vSurfXYZW;
	U32 ctVertices = 0;
	if(lpNode->getUnitVertex() == 3)
		ctVertices = GLMeshBuffer::ConvertFloat3ToFloat4(lpNode->vertices(), vSurfXYZW);
	else {
		assert(lpNode->getUnitVertex() == 4);
		lpNode->getVertexAttrib(ctVertices, vSurfXYZW, vatPosition);
	}

	if(lpNode->countVertices() > 0)
		this->setupVertexAttribs(vSurfXYZW, 4, vatPosition);

	if(lpNode->countNormals() > 0)
		this->setupVertexAttribs(lpNode->normals(), lpNode->getUnitNormal(), vatNormal);

	if(lpNode->countFaces() > 0)
		this->setupIndexBufferObject(lpNode->faceElements(), ftTriangles);

	//Copy to rest pos
	copyVertexBufferToRestPos();


	//Surface points and offsurface points
	vector<float> vOffSurfXYZF;
	vector<float> normals;
	U32 ctNormals = 0;

	vOffSurfXYZF.resize(2 * ctVertices * 4);
	lpNode->getVertexAttrib(ctNormals, normals, vatNormal);

	//Loop over vertices
	for (U32 i = 0; i < ctVertices; i++) {
		vec3f ptOut = vec3f(&vSurfXYZW[i * 4]) + vec3f(&normals[i * 3]) * m_offSurfaceLen;
		vec3f ptIn = vec3f(&vSurfXYZW[i * 4]) - vec3f(&normals[i * 3]) * m_offSurfaceLen;

		//The more we distance from skeleton the field becomes smaller so negative
		vOffSurfXYZF[i * 8] = ptOut.x;
		vOffSurfXYZF[i * 8 + 1] = ptOut.y;
		vOffSurfXYZF[i * 8 + 2] = ptOut.z;
		vOffSurfXYZF[i * 8 + 3] = - m_offSurfaceLen;

		//And closer positive
		vOffSurfXYZF[i * 8 + 4] = ptIn.x;
		vOffSurfXYZF[i * 8 + 5] = ptIn.y;
		vOffSurfXYZF[i * 8 + 6] = ptIn.z;
		vOffSurfXYZF[i * 8 + 7] = m_offSurfaceLen;
	}

	//Copy input to centers and fields arrays
	m_interpolationNodes.reserve(ctVertices * 2 * 3);
	m_fields.reserve(ctVertices*2);

	//Every other vertex will have its off-surface points
	for(U32 i=0; i<ctVertices; i++) {
		m_interpolationNodes.push_back(vSurfXYZW[i * 4]);
		m_interpolationNodes.push_back(vSurfXYZW[i * 4 + 1]);
		m_interpolationNodes.push_back(vSurfXYZW[i * 4 + 2]);
		m_fields.push_back(0.0f);
		if(i%2 == 0) {
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8]);
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 1]);
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 2]);
			m_fields.push_back(vOffSurfXYZF[i*8 + 3]);

			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 4]);
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 5]);
			m_interpolationNodes.push_back(vOffSurfXYZF[i*8 + 6]);
			m_fields.push_back(vOffSurfXYZF[i*8 + 7]);
		}
	}

	//Number of interpolation m_centers
	m_ctTotalInterpolationNodes = m_interpolationNodes.size() / 3;
	return computeInterpolationFunction(MATHMIN(m_ctTotalInterpolationNodes, DEFAULT_START_CENTERS_COUNT));
}

bool FastRBF::computeInterpolationFunction(U32 start_count, U32 iAttempt, U32 step, float fitting_accuracy) {

	//Use shuffling
	randomizeInterpolationNodes();

	m_ctCenters = start_count;
	MatrixXd A(start_count, start_count);

	//Fill Matrix with node distances
	vec3f xi, xj;
	U32 ctElements = 0;
	//Row
	for(U32 i=0; i<start_count; i++) {
		xi = vec3f(&m_interpolationNodes[i*3]);

		//Col
		for(U32 j=i; j<start_count; j++) {
			xj = vec3f(&m_interpolationNodes[j*3]);
			A(i, j) = vec3f::distance(xi, xj);
			A(j, i) = A(i, j);
			ctElements ++;
		}
	}

	MatrixXd F(start_count, 1);
	for(U32 i=0; i<start_count; i++) {
		F(i, 0) = m_fields[i];
	}
//	std::cout << "A Matrix: " << m_ctCenters << " x " << m_ctCenters << endl;
//	std::cout << A << endl;
//	std::cout << "F Matrix: " << start_count << " x 1" << endl;
//	std::cout << F << endl;

	//Solve System
	MatrixXd Lamda = A.colPivHouseholderQr().solve(F);
	m_lambda.resize(Lamda.rows());
	for(U32 i=0; i < Lamda.rows(); i++)
		m_lambda[i] = Lamda(i, 0);

	m_solutionError = (A*Lamda - F).norm() / F.norm();
	printf("Error expected to be: %f\n", m_solutionError);


	//Compute Residual Error
	float fitting_error = computeMaxResidualError();
	if(fitting_error > fitting_accuracy) {
		LogInfoArg2("Fitting failed with max fitting error = %f, Attempt#%d", fitting_error, iAttempt);

		if(start_count == m_ctTotalInterpolationNodes || iAttempt >= MAX_GREEDY_ATTEMPTS)
			return false;
		else
		{
			U32 next = MATHMIN(start_count + step, m_ctTotalInterpolationNodes);
			return computeInterpolationFunction(next, iAttempt + 1, step*2);
		}
	}

	return true;
}

float FastRBF::computeMaxResidualError() const {

	float residual = 0.0f;
	for(U32 i=0; i<m_ctTotalInterpolationNodes; i++) {
		vec3f v = vec3f(&m_interpolationNodes[i*3]);
		float error = Absolutef(m_fields[i] - fieldRBF(v));
		residual = MATHMAX(error, residual);
	}
	return residual;
}

bool FastRBF::testWithVoxelGridSamples(GPUPoly* lpGPUPoly) {
	if(lpGPUPoly == NULL)
		return false;

	vec4u dim;
	vector<float> arrXYZF;
	bool bres = lpGPUPoly->readBackVoxelGridSamples(dim, arrXYZF);
	if(!bres)
		return false;

	float error = 0.0f;
	for(U32 i=0; i < dim.w; i++)
	{
		vec4f xyzf = vec4f(&arrXYZF[i * 4]);

		float interpolatedField = fieldRBF(xyzf.xyz());
		error += Absolutef(interpolatedField - xyzf.w);
	}

	return (error < 0.5);
}


float FastRBF::fieldRBF(const vec3f& v, float iso) const{
	if(m_ctCenters == 0)
		return 0.0f;

	float sum = 0.0f;
	for(U32 i=0; i < m_ctCenters; i++)
		sum += m_lambda[i] * vec3f::distance(v, vec3f(&m_interpolationNodes[i * 3]));

	//Return field value in the original format
	return (sum + iso);
}

vec3f FastRBF::gradientRBF(const vec3f& v) const {

	return vec3f(0,0,0);
}



bool FastRBF::applyFemDisplacements(U32 dof, double* displacements) {

	if(!m_isValidVertex)
		return false;

	size_t arrLocalIndex[3];
	size_t arrGlobalIndex[3];
	arrGlobalIndex[0] = m_ctVertices;
	arrGlobalIndex[1] = 0;
	arrGlobalIndex[2] = 0;
	ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelApplyDeformations->getKernelWorkGroupSize(), arrLocalIndex);
	ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);

	U32 szVertexBuffer = m_ctVertices * 4 * sizeof(float);
	vector<float> homogenousDisplacements;
	homogenousDisplacements.resize(m_ctVertices * 4);
	for(U32 i=0; i < m_ctVertices; i++) {
		homogenousDisplacements[i * 4] = displacements[i * 3];
		homogenousDisplacements[i * 4 + 1] = displacements[i * 3 + 1];
		homogenousDisplacements[i * 4 + 2] = displacements[i * 3 + 2];
		homogenousDisplacements[i * 4 + 3] = 0;
	}

	//Copy displacements
	cl_mem inMemDisplacements = m_lpGPU->createMemBuffer(szVertexBuffer, ComputeDevice::memReadOnly);
	m_lpGPU->enqueueWriteBuffer(inMemDisplacements, szVertexBuffer, &homogenousDisplacements[0]);

	//Vertices
	cl_mem outMemMeshVertices = m_lpGPU->createMemBufferFromGL(m_vboVertex, ComputeDevice::memWriteOnly);
	m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshVertices);


	//		__kernel void ApplyVertexDeformations(U32 ctVertices,
	//											  __global float4* arrInRestPos,
	//											  __global float4* arrInDeformation,
	//											  __global float4* arrOutMeshVertex)
	m_lpKernelApplyDeformations->setArg(0, sizeof(U32), (void *)&m_ctVertices);
	m_lpKernelApplyDeformations->setArg(1, sizeof(cl_mem), &m_inoutMemRestPos);
	m_lpKernelApplyDeformations->setArg(2, sizeof(cl_mem), &inMemDisplacements);
	m_lpKernelApplyDeformations->setArg(3, sizeof(cl_mem), &outMemMeshVertices);

	m_lpGPU->enqueueNDRangeKernel(m_lpKernelApplyDeformations, 1, arrGlobalIndex, arrLocalIndex);
	m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshVertices);

	m_lpGPU->finishAllCommands();


	//Release all mem objects
	clReleaseMemObject(inMemDisplacements);
	clReleaseMemObject(outMemMeshVertices);
	homogenousDisplacements.resize(0);


	return true;
}

bool FastRBF::computeFieldArray() {
//	__kernel void ComputeRBFField(U32 ctCenters,
//								  __global float4* arrInterpolationNodesLambda,
//								  U32 ctVertices,
//								  __global float4* arrInOutVertexFields)

	return false;
}

GLMeshBuffer* FastRBF::prepareMeshBufferNormals() {
	return CLMeshBuffer::PrepareMeshBufferNormals(m_lpGPU, this, m_offSurfaceLen);
}

bool FastRBF::readbackMeshV3T3(U32& ctVertices, vector<float>& vertices,
								   U32& ctElements, vector<U32>& elements) {

	U32 fstep;
	vector<float> arrTemp;
	CLMeshBuffer::ReadbackMeshVertexAttribCL(m_lpGPU, this, vatPosition, ctVertices, fstep, arrTemp);
	if(fstep > 3) {
		vertices.resize(ctVertices * 3);
		for(U32 i=0; i<ctVertices; i++) {
			vertices[i*3] = arrTemp[i*fstep];
			vertices[i*3 + 1] = arrTemp[i*fstep + 1];
			vertices[i*3 + 2] = arrTemp[i*fstep + 2];
		}
	}
	else
		vertices.assign(arrTemp.begin(), arrTemp.end());


	U32 istep;
	CLMeshBuffer::ReadbackMeshFaceCL(m_lpGPU, this, ctElements, istep, elements);
	if(istep != 3)
		return false;
	return true;
}

int FastRBF::intersects(const vector<vec3f>& vertices,
						  vector<bool>& collisions,
						  vector<float>& penetrations,
						  int& idxMaxPenetrated) {

	int ctIntersected = 0;
	collisions.resize(vertices.size());
	penetrations.resize(vertices.size());
	float fMaxPenetration = 0.0f;
	idxMaxPenetrated = -1;

	m_collision.resize(0);
	m_penetration.resize(0);
	for(U32 i=0; i<vertices.size(); i++) {
		if(intersects(vertices[i], penetrations[i])) {
			m_collision.push_back(vertices[i]);
			m_penetration.push_back(penetrations[i]);

			collisions[i] = true;
			ctIntersected++;
		}
		else
			collisions[i] = false;


		if(penetrations[i] > fMaxPenetration) {
			fMaxPenetration = penetrations[i];
			idxMaxPenetrated = i;
		}
	}

	return ctIntersected;
}

//bool intersects(const vector<vec3f>& v, const vector<bool>& crossed) const;
bool FastRBF::intersects(const AABB& box) {

	//Return immediately if the two bounding boxes doesnot collide
	if(!m_bbox.intersect(box))
		return false;

	m_collision.resize(0);
	m_penetration.resize(0);
	vec3f dims = box.extent();
	float penetration;
	for(int i=0; i<8; i++) {
		vec3f v = box.lower() + vec3f::mul(vec3f((i & 0x04) >> 2, (i & 0x02) >> 1, i & 0x01), dims);
		if(intersects(v, penetration)) {
			m_collision.push_back(v);
			m_penetration.push_back(penetration);
		}
	}

	return (m_collision.size() > 0);
}

bool FastRBF::intersects(const vec3f& v, float& penetration) const {
	//IF the actual implicit function is available the inverse of wyvill func
	//can produce the iso distance
	float f = fieldRBF(v);
	penetration = 0.0f;
	if(f > 0.0f) {
		penetration = f;
		return true;
	}
	else
		return false;
}

void FastRBF::resetCollision() {
	m_collision.resize(0);
	m_penetration.resize(0);
}

void FastRBF::drawCollision() const {
	if(m_collision.size() == 0)
		return;

	//Compute Collision Face Center
	vec3f c = m_collision[0];
	for(U32 i=1; i<m_collision.size(); i++)
		c = c + m_collision[i];
	c = c * (1.0f / (float)m_collision.size());
	//Draw Center
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPointSize(5.0f);
	glColor3f(0.0f, 1.0f, 0.0f);
	glBegin(GL_POINTS);
		glVertex3fv(&c.e[0]);
	glEnd();
	glPopAttrib();


	//Draw Collision Points
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPointSize(5.0f);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for(U32 i=0; i<m_collision.size(); i++)
		glVertex3fv(&m_collision[i].e[0]);
	glEnd();
	glPopAttrib();

}
}
}

