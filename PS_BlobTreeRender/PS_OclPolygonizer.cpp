/*
 * PS_OclPolygonizer.cpp
 *
 *  Created on: 2012-02-20
 *      Author: pourya
 */
#include "PS_OclPolygonizer.h"
#include "PS_ComputeDevice.h"
#include "PS_OclScan.h"

#include "PS_FileDirectory.h"
#include "PS_Logger.h"
#include "PS_ReadSceneModel.h"
#include "_CellConfigTableGPU.h"


#include <iostream>
#include <GL/glew.h>
//#include <CL/cl_gl.h>

#define __NO_STD_VECTOR // Use cl::vector and cl::string and
#define __NO_STD_STRING // not STL versions, more on this later


using namespace std;
using namespace PS;
using namespace PS::FILESTRINGUTILS;
using namespace PS::HPC;


#define DATA_SIZE (1024*512)
#define ERR_GPUPOLY_KERNEL_NOT_BUILT -1
#define ERR_GPUPOLY_BUFFER_NOT_WRITTEN -2
#define ERR_GPUPOLY_BUFFER_NOT_READ -3
#define ERR_GPUPOLY_TRITABLE_NOT_READ -4
#define ERR_GPUPOLY_VERTEXTABLE_NOT_READ -5


const char *KernelSource = "\n"		      \
        "__kernel void square(                    \n" \
        "   __global float* input,                \n" \
        "   __global float* output,               \n" \
        "   const unsigned int count)             \n" \
        "{                                        \n" \
        "   int i = get_global_id(0);             \n" \
        "   if(i < count)                         \n" \
        "       output[i] = input[i] * input[i];  \n" \
        "}                                        \n" \
        "\n";

namespace PS{
namespace HPC{
	void GPUPoly::cleanup()
	{
		m_outMesh.cleanup();
	}

	bool GPUPoly::readModel(const char* lpFilePath)
	{
		SOABlobPrims tempPrims;
		SOABlobOps tempOps;
		ModelReader mr(&tempPrims, &tempOps, &m_mtxNode, &m_mtxBox);
		if(mr.read(lpFilePath) == MODELREAD_SUCCESS)
		{
			//Set Header
			m_arrHeader[0] = tempPrims.bboxLo.x;
			m_arrHeader[1] = tempPrims.bboxLo.y;
			m_arrHeader[2] = tempPrims.bboxLo.z;
			m_arrHeader[3] = 1.0f;
			m_arrHeader[4] = tempPrims.bboxHi.x;
			m_arrHeader[5] = tempPrims.bboxHi.y;
			m_arrHeader[6] = tempPrims.bboxHi.z;
			m_arrHeader[7] = 1.0f;

			m_arrHeader[8] = static_cast<float>(tempPrims.count);
			m_arrHeader[9] = static_cast<float>(tempOps.count);
			m_arrHeader[10] = static_cast<float>(m_mtxNode.count);
			m_arrHeader[11] = DEFAULT_CELL_SIZE;

			//Count
			m_ctPrims = tempPrims.count;
			m_ctOps = tempOps.count;

			//Prims
			for(U32 i=0; i < tempPrims.count; i++)
			{
				//Update Parent/Link and Link Properties
				m_arrPrims[i * DATASIZE_PRIMITIVE + 0] = static_cast<float>(tempPrims.type[i]);
				m_arrPrims[i * DATASIZE_PRIMITIVE + 1] = static_cast<float>(tempPrims.idxMatrix[i]);

				//Pos
				m_arrPrims[i * DATASIZE_PRIMITIVE + 4] = tempPrims.posX[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 5] = tempPrims.posY[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 6] = tempPrims.posZ[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 7] = 0.0f;

				//Dir
				m_arrPrims[i * DATASIZE_PRIMITIVE + 8] = tempPrims.dirX[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 9] = tempPrims.dirY[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 10] = tempPrims.dirZ[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 11] = 0.0f;

				//Res
				m_arrPrims[i * DATASIZE_PRIMITIVE + 12] = tempPrims.resX[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 13] = tempPrims.resY[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 14] = tempPrims.resZ[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 15] = 0.0f;

				//Color
				m_arrPrims[i * DATASIZE_PRIMITIVE + 16] = tempPrims.colorX[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 17] = tempPrims.colorY[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 18] = tempPrims.colorZ[i];
				m_arrPrims[i * DATASIZE_PRIMITIVE + 19] = 1.0f;
			}

			//Ops
			for(U32 i=0; i < tempOps.count; i++)
			{
				//Update Parent/Link and Link Properties
				m_arrOps[i * DATASIZE_OPERATOR + 0] = static_cast<float>(tempOps.type[i]);
				m_arrOps[i * DATASIZE_OPERATOR + 1] = static_cast<float>(tempOps.opFlags[i]);

				U32 opLeftRightChild = (tempOps.opLeftChild[i] << 16) | tempOps.opRightChild[i];
				m_arrOps[i * DATASIZE_OPERATOR + 2] = static_cast<float>(opLeftRightChild);
				//m_arrOps[i * DATASIZE_OPERATOR + 3] = static_cast<float>();

				m_arrOps[i * DATASIZE_OPERATOR + 4] = static_cast<float>(tempOps.resX[i]);
				m_arrOps[i * DATASIZE_OPERATOR + 5] = static_cast<float>(tempOps.resY[i]);
				m_arrOps[i * DATASIZE_OPERATOR + 6] = static_cast<float>(tempOps.resZ[i]);
				m_arrOps[i * DATASIZE_OPERATOR + 7] = static_cast<float>(tempOps.resW[i]);
			}

			//Update Parent/Link and Link Properties
			SIMPLESTACK<MAX_TREE_NODES> stkOps;
			stkOps.push(0);
			while(!stkOps.empty())
			{
				U16 idxOp = stkOps.top();
				stkOps.pop();

				U32 opFlags = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS]);
				bool isRange =(bool)((opFlags & ofChildIndexIsRange) >> 2);
				bool isLCOp = (bool)((opFlags & ofLeftChildIsOp) >> 1);
				bool isRCOp = (bool)(opFlags & ofRightChildIsOp);

				U32 idxKids = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_CHILDREN]);
				U16 idxLC   = idxKids >> 16;
				U16 idxRC   = idxKids & 0xFFFF;

				if(isRange)
				{
					printf("READER: Range is not supported in GPU yet!");
					return false;
				}


				U32 linkLC = (idxOp << 16) | idxRC;
				//LINK FLAGS: LC = 1, isRCOP
				U16 linkFlagLC = (isRCOp == 1)?3:2;

				U32 linkRC = (idxOp << 16) | idxLC;
				//LINK FLAGS: LC = 0, isRCOP
				U16 linkFlagRC = (isLCOp == 1)?1:0;

				//LC
				if(isLCOp)
				{
					m_arrOps[idxLC * DATASIZE_OPERATOR + OFFSET_OP_PARENT_LINK] = static_cast<float>(linkLC);
					U32 flags = static_cast<U32>(m_arrOps[idxLC * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS]);
					flags |= (linkFlagLC << 16);
					m_arrOps[idxLC * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS] = (float)flags;
					stkOps.push(idxLC);
				}
				else
				{
					m_arrPrims[idxLC * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT_LINK] = static_cast<float>(linkLC);
					U32 flags = static_cast<U32>(m_arrPrims[idxLC * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT_LINK]);
					flags |= (linkFlagLC << 16);
					m_arrPrims[idxLC * DATASIZE_PRIMITIVE + OFFSET_PRIM_LINK_FLAGS] = (float)flags;
				}


				//RC
				if(isRCOp)
				{
					m_arrOps[idxRC * DATASIZE_OPERATOR + OFFSET_OP_PARENT_LINK] = static_cast<float>(linkRC);
					U32 flags = static_cast<U32>(m_arrOps[idxRC * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS]);
					flags |= (linkFlagRC << 16);
					m_arrOps[idxRC * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS] = (float)flags;
					stkOps.push(idxRC);
				}
				else
				{
					m_arrPrims[idxRC * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT_LINK] = static_cast<float>(linkRC);
					U32 flags = static_cast<U32>(m_arrPrims[idxRC * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT_LINK]);
					flags |= (linkFlagRC << 16);
					m_arrPrims[idxRC * DATASIZE_PRIMITIVE + OFFSET_PRIM_LINK_FLAGS] = (float)flags;
				}

			}

			return true;
		}

		return false;
	}

	void GPUPoly::drawMesh(bool bWireFrameMode)
	{
		if(!m_outMesh.bIsValid)	return;


		glBindBuffer(GL_ARRAY_BUFFER, m_outMesh.vboColor);
		glColorPointer(4, GL_FLOAT, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);

		glBindBuffer(GL_ARRAY_BUFFER, m_outMesh.vboVertex);
		glVertexPointer(4, GL_FLOAT, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);

		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshBuffers.iboFaces);
		//glEnableClientState(GL_ELEMENT_ARRAY_BUFFER);
		//glDrawElements(GL_TRIANGLES, (GLsizei)meshBuffers.ctTriangles * 3, GL_UNSIGNED_INT, (GLvoid*)0);
		glDrawArrays(GL_POINTS, 0, m_ctCells);

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		//glDisableClientState(GL_ELEMENT_ARRAY_BUFFER);
	}

	int GPUPoly::run(float cellsize)
	{
		m_arrHeader[OFFSET_CELLSIZE] = cellsize;
		m_outMesh.cleanup();

		DAnsiStr strFP = ExtractFilePath(GetExePath());
		strFP = ExtractOneLevelUp(strFP);
		strFP += DAnsiStr("/AA_Shaders/Polygonizer.cl");

		LogInfo("1.Setup compute device. Prefer AMD GPU.");
		//Algorithm Overview
		//1.First we compute the fields in the grid of 8x8x8
		//2.Then all the cells within each grid are evaluated and the number of output vertices are determined.
		//3.Empty cells are discarded
		//4.Triangles are computed to the output mesh
		//Create a GPU Compute Device
		PS::HPC::ComputeDevice* lpGPU = new ComputeDevice(ComputeDevice::dtGPU, true, "AMD");
		lpGPU->printInfo();
		

		//Create the OCL Scan Primitive
		//PS::HPC::Scan* lpScanner = new PS::HPC::Scan(lpGPU);
		//lpScanner->scanExclusiveLarge()
		LogInfo("2.Compile OpenCL program.");
		ComputeProgram* lpProgram = lpGPU->addProgramFromFile(strFP.cptr());
		if(lpProgram == NULL)
			return ERR_GPUPOLY_KERNEL_NOT_BUILT;

		DAnsiStr strLogPath = ExtractFilePath(GetExePath()) + DAnsiStr("OpenCL_Framework.log");
		TheEventLogger::Instance().setOutFilePath(strLogPath.cptr());
		TheEventLogger::Instance().flush();

		//Build Kernel
		ComputeKernel* lpKernelCellConfig = lpProgram->addKernel("ComputeConfigIndexVertexCount");

		//BBOX
		svec3f lower = svec3f(m_arrHeader[0], m_arrHeader[1], m_arrHeader[2]);
		svec3f upper = svec3f(m_arrHeader[4], m_arrHeader[5], m_arrHeader[6]);
		svec3f temp = vscale3f(1.0f/ cellsize, vsub3f(upper, lower));

		U32 arrNeeded[4];
		arrNeeded[0] = (U32)ceil(temp.x) + 1;
		arrNeeded[1] = (U32)ceil(temp.y) + 1;
		arrNeeded[2] = (U32)ceil(temp.z) + 1;
		arrNeeded[3] = 0;
		m_ctCells = arrNeeded[0] * arrNeeded[1] * arrNeeded[2];

		//Buffers
		glGenBuffers(1, &m_outMesh.vboVertex);
		glBindBuffer(GL_ARRAY_BUFFER, m_outMesh.vboVertex);
		glBufferData(GL_ARRAY_BUFFER, m_ctCells * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshVertex = clCreateFromGLBuffer(lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_outMesh.vboVertex, NULL);

		glGenBuffers(1, &m_outMesh.vboColor);
		glBindBuffer(GL_ARRAY_BUFFER, m_outMesh.vboColor);
		glBufferData(GL_ARRAY_BUFFER, m_ctCells * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshColor = clCreateFromGLBuffer(lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_outMesh.vboColor, NULL);

		//Load Marching Cubes tables
		cl_image_format imageFormat;
		imageFormat.image_channel_order = CL_R;
		imageFormat.image_channel_data_type = CL_UNSIGNED_INT8;

		//Triangle and Vertex Count Tables
		cl_int errNum;
		cl_mem inMemTriTable = clCreateImage2D(lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
											   &imageFormat, 16, 256, 0, (void*) g_triTableGPU, &errNum);
		if(errNum != CL_SUCCESS)					
			return ERR_GPUPOLY_TRITABLE_NOT_READ;
		
		cl_mem inMemVertexCountTable = clCreateImage2D(lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
													   &imageFormat, 256, 1, 0, (void*) g_numVertsTableGPU, &errNum);
		if(errNum != CL_SUCCESS)
			return ERR_GPUPOLY_VERTEXTABLE_NOT_READ;
		
		//Input Pos
		cl_mem inMemHeader;
		cl_mem inMemOps;
		cl_mem inMemPrims;		
		cl_mem inMemMtx;
		cl_mem inMemNeededCells;

		// Create the device memory vectors
		inMemHeader = lpGPU->createMemBuffer(sizeof(float) * DATASIZE_HEADER, ComputeDevice::memReadOnly);
		inMemOps  	= lpGPU->createMemBuffer(sizeof(float) * DATASIZE_OPERATOR * m_ctOps, ComputeDevice::memReadOnly);
		inMemPrims  = lpGPU->createMemBuffer(sizeof(float) * DATASIZE_PRIMITIVE * m_ctPrims, ComputeDevice::memReadOnly);		
		inMemMtx	= lpGPU->createMemBuffer(sizeof(float) * PRIM_MATRIX_STRIDE * m_mtxNode.count, ComputeDevice::memReadOnly);
		inMemNeededCells = lpGPU->createMemBuffer(sizeof(int) * 4, ComputeDevice::memReadOnly);

		// Transfer the input vector into device memory.
		//Header
		if(!lpGPU->enqueueWriteBuffer(inMemHeader, sizeof(float) * DATASIZE_HEADER, m_arrHeader))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Ops
		if(!lpGPU->enqueueWriteBuffer(inMemOps, sizeof(float) * DATASIZE_OPERATOR * m_ctOps, m_arrOps))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Prims
		if(!lpGPU->enqueueWriteBuffer(inMemPrims, sizeof(float) * DATASIZE_PRIMITIVE * m_ctPrims, m_arrPrims))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Matrix
		if(!lpGPU->enqueueWriteBuffer(inMemMtx, sizeof(float) * PRIM_MATRIX_STRIDE * m_mtxNode.count, m_mtxNode.matrix))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Needed
		if(!lpGPU->enqueueWriteBuffer(inMemNeededCells, sizeof(int) * 4, arrNeeded))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

			/*
			glGenBuffers(1, &m_meshBuffer.vboNormal);
			glBindBuffer(GL_ARRAY_BUFFER, m_meshBuffer.vboNormal);
			glBufferData(GL_ARRAY_BUFFER, MAX_MPU_VERTEX_COUNT * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
			cl_mem inMemMeshNormal = clCreateFromGLBuffer(lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_meshBuffer.vboNormal, NULL);

			glGenBuffers(1, &m_meshBuffer.iboFaces);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_meshBuffer.iboFaces);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_MPU_TRIANGLE_COUNT * 3 * sizeof(U32), 0, GL_DYNAMIC_DRAW);
			cl_mem inMemMeshFaces = clCreateFromGLBuffer(lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_meshBuffer.iboFaces, NULL);
		 */
			/*
		__kernel void ComputeConfigIndexVertexCount(const uint ctTotalCells,
			__global uchar* arrOutCellConfig,
			__global uchar* arrOutVertexCount,
			__global float* arrInHeader,
			__global float* arrInOps,										 
			__global float* arrInPrims,											 
			__global float* arrInMtxNode,
			const __global uint* arrInNeededCells,
			__read_only image2d_t texInVertexCountTable)
			*/

		// Set the arguments to the compute kernel
		//lpKernelCellConfig->setArg(0, sizeof(cl_mem), &inMem);
		//lpKernelCellConfig->setArg(1, sizeof(cl_mem), &inMem);
		//lpKernelCellConfig->setArg(2, sizeof(cl_mem), &inMem);
		lpKernelCellConfig->setArg(3, sizeof(cl_mem), &inMemHeader);
		lpKernelCellConfig->setArg(4, sizeof(cl_mem), &inMemOps);
		lpKernelCellConfig->setArg(5, sizeof(cl_mem), &inMemPrims);
		lpKernelCellConfig->setArg(6, sizeof(cl_mem), &inMemMtx);
		lpKernelCellConfig->setArg(7, sizeof(cl_mem), &inMemNeededCells);
		lpKernelCellConfig->setArg(8, sizeof(cl_mem), &inMemVertexCountTable);


		//Acquire Mesh for Writing
		clEnqueueAcquireGLObjects(lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueAcquireGLObjects(lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);


		size_t local;
		// Get the maximum work group size for executing the kernel on the device
		cl_int err = clGetKernelWorkGroupInfo(lpKernelCellConfig->getKernel(),
				lpGPU->getDevice(),
				CL_KERNEL_WORK_GROUP_SIZE,
				sizeof(local), &local, NULL);
		if (err != CL_SUCCESS) {
			cerr << "Error: Failed to retrieve kernel work group info! "
					<<  err << endl;
			exit(1);
		}

		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
		size_t szNeeded[3];
		for(int i=0; i<3; i++)
			szNeeded[i] = arrNeeded[i];

		err = clEnqueueNDRangeKernel(lpGPU->getCommandQ(), lpKernelCellConfig->getKernel(),
				3, NULL, szNeeded, NULL,
				0, NULL, NULL);
		if (err) {
			char buffer[1024];
			sprintf(buffer, "Error: Failed to execute kernel! (%s)", ComputeDevice::oclErrorString(err));
			cerr << buffer << endl;
			return EXIT_FAILURE;
		}

		// Wait for all commands to complete
		lpGPU->finishAllCommands();

		//Release Mesh for Writing
		clEnqueueReleaseGLObjects(lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueReleaseGLObjects(lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);

		m_outMesh.bIsValid = true;


		// Shutdown and cleanup
		clReleaseMemObject(inMemTriTable);
		clReleaseMemObject(inMemVertexCountTable);

		clReleaseMemObject(inMemHeader);
		clReleaseMemObject(inMemOps);
		clReleaseMemObject(inMemPrims);		
		clReleaseMemObject(inMemMtx);
		clReleaseMemObject(inMemNeededCells);


		SAFE_DELETE(lpGPU);
		return 1;
	}

	int Run_MeshCentroidKernel(float* arrVertices, U32* arrFaces, 
		U32 ctVertices, U32 ctFaces,
		float*& lpCentriods)
	{

		return 1;
	}



	int Run_SphereDistKernel()
	{
		DAnsiStr strFP = ExtractOneLevelUp(ExtractFilePath(GetExePath()));
		strFP += DAnsiStr("/AA_Shaders/SphereDist.cl");
		PS::HPC::ComputeDevice* lpGPU = new ComputeDevice(ComputeDevice::dtGPU);
		lpGPU->printInfo();

		ComputeProgram* lpProgram = lpGPU->addProgramFromFile(strFP.cptr());
		if(lpProgram)
		{
			ComputeKernel* lpKernel = lpProgram->addKernel("distSphere");

			//Input Pos
			float* arrPosX = new float[DATA_SIZE];
			float* arrPosY = new float[DATA_SIZE];
			float* arrPosZ = new float[DATA_SIZE];

			//Output Dist
			float* arrDist = new float[DATA_SIZE];

			//Number of correct results returned
			unsigned int correct;
			cl_mem inMemPosX;
			cl_mem inMemPosY;
			cl_mem inMemPosZ;
			cl_mem outMemDist;

			// Fill the vector with random float values
			U32 count = DATA_SIZE;
			for(U32 i = 0; i < count; i++)
			{
				arrPosX[i] = rand() / (float)RAND_MAX;
				arrPosY[i] = rand() / (float)RAND_MAX;
				arrPosZ[i] = rand() / (float)RAND_MAX;
			}


			// Create the device memory vectors
			inMemPosX = lpGPU->createMemBuffer(sizeof(float) * count, ComputeDevice::memReadOnly);
			inMemPosY = lpGPU->createMemBuffer(sizeof(float) * count, ComputeDevice::memReadOnly);
			inMemPosZ = lpGPU->createMemBuffer(sizeof(float) * count, ComputeDevice::memReadOnly);
			outMemDist = lpGPU->createMemBuffer(sizeof(float) * count, ComputeDevice::memWriteOnly);

			// Transfer the input vector into device memory.
			//PosX
			if(!lpGPU->enqueueWriteBuffer(inMemPosX, sizeof(float) * count, arrPosX))
				return -1;

			//PosY
			if(!lpGPU->enqueueWriteBuffer(inMemPosY, sizeof(float) * count, arrPosY))
				return -1;

			//PosZ
			if(!lpGPU->enqueueWriteBuffer(inMemPosZ, sizeof(float) * count, arrPosZ))
				return -1;

			// Set the arguments to the compute kernel
			lpKernel->setArg(0, sizeof(cl_mem), &inMemPosX);
			lpKernel->setArg(1, sizeof(cl_mem), &inMemPosY);
			lpKernel->setArg(2, sizeof(cl_mem), &inMemPosZ);
			lpKernel->setArg(3, sizeof(cl_mem), &outMemDist);
			lpKernel->setArg(4, sizeof(U32), &count);

			size_t local;
			size_t global;
			// Get the maximum work group size for executing the kernel on the device
			cl_int err = clGetKernelWorkGroupInfo(lpKernel->getKernel(),
				lpGPU->getDevice(),
				CL_KERNEL_WORK_GROUP_SIZE,
				sizeof(local), &local, NULL);
			if (err != CL_SUCCESS) {
				cerr << "Error: Failed to retrieve kernel work group info! "
					<<  err << endl;
				exit(1);
			}

			// Execute the kernel over the vector using the
			// maximum number of work group items for this device
			global = count;
			err = clEnqueueNDRangeKernel(lpGPU->getCommandQ(), lpKernel->getKernel(),
				1, NULL, &global, &local,
				0, NULL, NULL);
			if (err) {
				cerr << "Error: Failed to execute kernel!" << endl;
				return EXIT_FAILURE;
			}

			// Wait for all commands to complete
			lpGPU->finishAllCommands();

			// Read back the results from the device to verify the output
			if(!lpGPU->enqueueReadBuffer(outMemDist, sizeof(float) * count, arrDist))
			{
				return -1;
			}

			// Validate our results
			//
			correct = 0;
			float current;
			float v;
			for(U32 i = 0; i < count; i++)
			{
				current = arrDist[i];
				v =	arrPosX[i] * arrPosX[i] + arrPosY[i] * arrPosY[i] + arrPosZ[i] * arrPosZ[i];
				if(FLOAT_EQ(current, v))
					correct++;
			}

			// Print a brief summary detailing the results
			cout << "Computed " << correct << "/" << count << " correct values" << endl;
			cout << "Computed " << 100.f * (float)correct/(float)count
				<< "% correct values" << endl;

			// Shutdown and cleanup
			SAFE_DELETE_ARRAY(arrPosX);
			SAFE_DELETE_ARRAY(arrPosY);
			SAFE_DELETE_ARRAY(arrPosZ);
			SAFE_DELETE_ARRAY(arrDist);

			clReleaseMemObject(inMemPosX);
			clReleaseMemObject(inMemPosY);
			clReleaseMemObject(inMemPosZ);
			clReleaseMemObject(outMemDist);
		}


		SAFE_DELETE(lpGPU);
		return 1;
	}

/*

	int GPUPoly::run(float cellsize)
	{
		m_meshBuffer.cleanup();

		DAnsiStr strFP = ExtractFilePath(GetExePath());
		strFP = ExtractOneLevelUp(strFP);
		strFP += DAnsiStr("/AA_Shaders/Polygonizer.cl");
		PS::HPC::ComputeDevice* lpGPU = new ComputeDevice(ComputeDevice::dtGPU, true);
		lpGPU->printInfo();

		ComputeProgram* lpProgram = lpGPU->addProgramFromFile(strFP.cptr());
		if(lpProgram == NULL)
			return ERR_GPUPOLY_KERNEL_NOT_BUILT;

		ComputeKernel* lpKernel = lpProgram->addKernel("poly");

		//Buffers
		glGenBuffers(1, &m_meshBuffer.vboVertex);
		glBindBuffer(GL_ARRAY_BUFFER, m_meshBuffer.vboVertex);
		glBufferData(GL_ARRAY_BUFFER, DATA_SIZE * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshVertex = clCreateFromGLBuffer(lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_meshBuffer.vboVertex, NULL);

		glGenBuffers(1, &m_meshBuffer.vboColor);
		glBindBuffer(GL_ARRAY_BUFFER, m_meshBuffer.vboColor);
		glBufferData(GL_ARRAY_BUFFER, DATA_SIZE * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshColor = clCreateFromGLBuffer(lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_meshBuffer.vboColor, NULL);


		//Input Pos
		cl_mem inMemPosX;
		cl_mem inMemPosY;
		cl_mem inMemPosZ;

		// Fill the vector with random float values
		U32 count = DATA_SIZE;
		float* arrPosX = new float[DATA_SIZE];
		float* arrPosY = new float[DATA_SIZE];
		float* arrPosZ = new float[DATA_SIZE];
		for(U32 i = 0; i < DATA_SIZE; i++)
		{
			arrPosX[i] = rand() / (float)RAND_MAX;
			arrPosY[i] = rand() / (float)RAND_MAX;
			arrPosZ[i] = rand() / (float)RAND_MAX;
		}


		// Create the device memory vectors
		inMemPosX = lpGPU->createMemBuffer(sizeof(float) * count, ComputeDevice::memReadOnly);
		inMemPosY = lpGPU->createMemBuffer(sizeof(float) * count, ComputeDevice::memReadOnly);
		inMemPosZ = lpGPU->createMemBuffer(sizeof(float) * count, ComputeDevice::memReadOnly);

		// Transfer the input vector into device memory.
		//PosX
		if(!lpGPU->enqueueWriteBuffer(inMemPosX, sizeof(float) * count, arrPosX))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//PosY
		if(!lpGPU->enqueueWriteBuffer(inMemPosY, sizeof(float) * count, arrPosY))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//PosZ
		if(!lpGPU->enqueueWriteBuffer(inMemPosZ, sizeof(float) * count, arrPosZ))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;


		// Set the arguments to the compute kernel
		lpKernel->setArg(0, sizeof(cl_mem), &outMemMeshVertex);
		lpKernel->setArg(1, sizeof(cl_mem), &outMemMeshColor);
		lpKernel->setArg(2, sizeof(cl_mem), &inMemPosX);
		lpKernel->setArg(3, sizeof(cl_mem), &inMemPosY);
		lpKernel->setArg(4, sizeof(cl_mem), &inMemPosZ);
		lpKernel->setArg(5, sizeof(U32), &count);

		//Acquire Mesh for Writing
		clEnqueueAcquireGLObjects(lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueAcquireGLObjects(lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);


		size_t local;
		size_t global;
		// Get the maximum work group size for executing the kernel on the device
		cl_int err = clGetKernelWorkGroupInfo(lpKernel->getKernel(),
				lpGPU->getDevice(),
				CL_KERNEL_WORK_GROUP_SIZE,
				sizeof(local), &local, NULL);
		if (err != CL_SUCCESS) {
			cerr << "Error: Failed to retrieve kernel work group info! "
					<<  err << endl;
			exit(1);
		}

		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
		global = count;
		err = clEnqueueNDRangeKernel(lpGPU->getCommandQ(), lpKernel->getKernel(),
				1, NULL, &global, &local,
				0, NULL, NULL);
		if (err) {
			char buffer[1024];
			sprintf(buffer, "Error: Failed to execute kernel! (%s)", ComputeDevice::oclErrorString(err));
			cerr << buffer << endl;
			return EXIT_FAILURE;
		}

		// Wait for all commands to complete
		lpGPU->finishAllCommands();

		//Release Mesh for Writing
		clEnqueueReleaseGLObjects(lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueReleaseGLObjects(lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);

		m_meshBuffer.bIsValid = true;


		// Shutdown and cleanup
		SAFE_DELETE_ARRAY(arrPosX);
		SAFE_DELETE_ARRAY(arrPosY);
		SAFE_DELETE_ARRAY(arrPosZ);

		clReleaseMemObject(inMemPosX);
		clReleaseMemObject(inMemPosY);
		clReleaseMemObject(inMemPosZ);



		SAFE_DELETE(lpGPU);
		return 1;
	}
	*/

}
}


int Run_ArrayTestKernel()
{
    int devType=CL_DEVICE_TYPE_GPU;

    cl_int err;     // error code returned from api calls

    size_t global;  // global domain size for our calculation
    size_t local;   // local domain size for our calculation

    cl_platform_id cpPlatform; // OpenCL platform
    cl_device_id device_id;    // compute device id
    cl_context context;        // compute context
    cl_command_queue commands; // compute command queue
    cl_program program;        // compute program
    cl_kernel kernel;          // compute kernel

    // Connect to a compute device
    err = clGetPlatformIDs(1, &cpPlatform, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to find a platform!" << endl;
        return EXIT_FAILURE;
    }

    //Get Platform Info
    char buffer[1024];
    err = clGetPlatformInfo(cpPlatform, CL_PLATFORM_NAME, sizeof(buffer), buffer, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_PLATFORM_NAME" << endl;
        return EXIT_FAILURE;
    }
    cout << buffer << endl;


    // Get a device of the appropriate type
    err = clGetDeviceIDs(cpPlatform, devType, 1, &device_id, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to create a device group!" << endl;
        return EXIT_FAILURE;
    }

    err = clGetDeviceInfo(device_id, CL_DEVICE_NAME, sizeof(buffer), buffer, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_DEVICE_NAME" << endl;
        return EXIT_FAILURE;
    }
    cout << buffer << endl;
    err = clGetDeviceInfo(device_id, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(buffer), buffer, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_DEVICE_MAX_COMPUTE_UNITS" << endl;
        return EXIT_FAILURE;
    }
    cout << buffer << endl;



    // Create a compute context
    context = clCreateContext(0, 1, &device_id, NULL, NULL, &err);
    if (!context) {
        cerr << "Error: Failed to create a compute context!" << endl;
        return EXIT_FAILURE;
    }

    // Create a command commands
    commands = clCreateCommandQueue(context, device_id, 0, &err);
    if (!commands) {
        cerr << "Error: Failed to create a command queue!" << endl;
        return EXIT_FAILURE;
    }

    // Create the compute program from the source buffer
    program = clCreateProgramWithSource(context, 1,
                                        (const char **) &KernelSource,
                                        NULL, &err);
    if (!program) {
        cerr << "Error: Failed to create compute program!" << endl;
        return EXIT_FAILURE;
    }

    // Build the program executable
    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS) {
        size_t len;
        char buffer[2048];

        cerr << "Error: Failed to build program executable!" << endl;
        clGetProgramBuildInfo(program, device_id, CL_PROGRAM_BUILD_LOG,
                              sizeof(buffer), buffer, &len);
        cerr << buffer << endl;
        exit(1);
    }

    // Create the compute kernel in the program
    kernel = clCreateKernel(program, "square", &err);
    if (!kernel || err != CL_SUCCESS) {
        cerr << "Error: Failed to create compute kernel!" << endl;
        exit(1);
    }

    // create data for the run
    float* data = new float[DATA_SIZE];    // original data set given to device
    float* results = new float[DATA_SIZE]; // results returned from device
    unsigned int correct;               // number of correct results returned
    cl_mem input;                       // device memory used for the input array
    cl_mem output;                      // device memory used for the output array

    // Fill the vector with random float values
    U32 count = DATA_SIZE;
    for(U32 i = 0; i < count; i++)
        data[i] = rand() / (float)RAND_MAX;

    // Create the device memory vectors
    //
    input = clCreateBuffer(context,  CL_MEM_READ_ONLY,
                           sizeof(float) * count, NULL, NULL);
    output = clCreateBuffer(context, CL_MEM_WRITE_ONLY,
                            sizeof(float) * count, NULL, NULL);
    if (!input || !output) {
        cerr << "Error: Failed to allocate device memory!" << endl;
        exit(1);
    }

    // Transfer the input vector into device memory
    err = clEnqueueWriteBuffer(commands, input,
                               CL_TRUE, 0, sizeof(float) * count,
                               data, 0, NULL, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to write to source array!" << endl;
        exit(1);
    }

    // Set the arguments to the compute kernel
    err = 0;
    err  = clSetKernelArg(kernel, 0, sizeof(cl_mem), &input);
    err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &output);
    err |= clSetKernelArg(kernel, 2, sizeof(unsigned int), &count);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to set kernel arguments! " << err << endl;
        exit(1);
    }

    // Get the maximum work group size for executing the kernel on the device
    err = clGetKernelWorkGroupInfo(kernel, device_id,
                                   CL_KERNEL_WORK_GROUP_SIZE,
                                   sizeof(local), &local, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to retrieve kernel work group info! "
             <<  err << endl;
        exit(1);
    }

    // Execute the kernel over the vector using the
    // maximum number of work group items for this device
    global = count;
    err = clEnqueueNDRangeKernel(commands, kernel,
                                 1, NULL, &global, &local,
                                 0, NULL, NULL);
    if (err) {
        cerr << "Error: Failed to execute kernel!" << endl;
        return EXIT_FAILURE;
    }

    // Wait for all commands to complete
    clFinish(commands);

    // Read back the results from the device to verify the output
    err = clEnqueueReadBuffer( commands, output,
                               CL_TRUE, 0, sizeof(float) * count,
                               results, 0, NULL, NULL );
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to read output array! " <<  err << endl;
        exit(1);
    }

    // Validate our results
    correct = 0;
    for(U32 i = 0; i < count; i++) {
        if(results[i] == data[i] * data[i])
            correct++;
    }

    // Print a brief summary detailing the results
    cout << "Computed " << correct << "/" << count << " correct values" << endl;
    cout << "Computed " << 100.f * (float)correct/(float)count
         << "% correct values" << endl;

    // Shutdown and cleanup
    delete [] data; delete [] results;

    clReleaseMemObject(input);
    clReleaseMemObject(output);
    clReleaseProgram(program);
    clReleaseKernel(kernel);
    clReleaseCommandQueue(commands);
    clReleaseContext(context);

    return 0;
}

