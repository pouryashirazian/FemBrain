/*
 * PS_OclPolygonizer.cpp
 *
 *  Created on: 2012-02-20
 *      Author: pourya
 */
#include "PS_OclPolygonizer.h"
#include "../PS_Graphics/PS_OclScan.h"
#include "../PS_Base/PS_FileDirectory.h"
#include "../PS_Base/PS_Logger.h"
#include "PS_ReadSceneModel.h"
#include "_CellConfigTable.h"
#include "_CellConfigTableCompact.h"
#include "../PS_Graphics/PS_DebugUtils.h"
#include "../PS_Graphics/PS_OclSumScan.h"

#include <iostream>
#include <GL/glew.h>
//#include <CL/cl_gl.h>

#define __NO_STD_VECTOR // Use cl::vector and cl::string and
#define __NO_STD_STRING // not STL versions, more on this later


using namespace std;
using namespace PS;
using namespace PS::DEBUG;
using namespace PS::FILESTRINGUTILS;
using namespace PS::HPC;

#define SUCCESS 1
#define ERR_GPUPOLY_KERNEL_NOT_BUILT -1
#define ERR_GPUPOLY_BUFFER_NOT_WRITTEN -2
#define ERR_GPUPOLY_BUFFER_NOT_READ -3
#define ERR_GPUPOLY_TRITABLE_NOT_READ -4
#define ERR_GPUPOLY_VERTEXTABLE_NOT_READ -5
#define ERR_GPUPOLY_FIELDSTABLE	-6
#define ERR_NO_CROSSED_CELLS -6


#define MAX_VERTICES_COUNT_PER_CELL		15
#define MAX_TRIANGLES_COUNT_PER_CELL	5





namespace PS{
namespace HPC{

	GPUPoly::GPUPoly() : GLMeshBuffer()
	{
		init();		
	}

	GPUPoly::GPUPoly(const char* lpFilePath) : GLMeshBuffer()
	{
		init();
		this->readModel(lpFilePath);
	}

	GPUPoly::~GPUPoly()
	{
		GLMeshBuffer::cleanup();

		//Clear Mem Objects
		if(m_bModelLoaded)
		{
			clReleaseMemObject(m_inMemHeader);
			clReleaseMemObject(m_inMemOps);
			clReleaseMemObject(m_inMemPrims);
			clReleaseMemObject(m_inMemMtx);
		}

		clReleaseMemObject(m_inMemTriangleTable);
		clReleaseMemObject(m_inMemVertexCountTable);
		SAFE_DELETE(m_lpGPU);
		SAFE_DELETE(m_lpOclSumScan);
	}

	
	bool GPUPoly::readModel(const char* lpFilePath)
	{
		SOABlobPrims tempPrims;
		SOABlobOps tempOps;
		ModelReader mr(&tempPrims, &tempOps, &m_mtxNode, &m_mtxBox);
		if(mr.read(lpFilePath) != MODELREAD_SUCCESS)
			return false;

		//Set Header
		m_bboxLo = tempPrims.bboxLo;
		m_bboxHi = tempPrims.bboxHi;
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
			m_arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE] = static_cast<float>(tempPrims.type[i]);
			m_arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_IDX_MATRIX] = static_cast<float>(tempPrims.idxMatrix[i]);
			m_arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_LINK_FLAGS] = 0;
			m_arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT_LINK] = 0;

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
		if(tempOps.count > 0)
		{
			for(U32 i=0; i < tempOps.count; i++)
			{
				//Update Parent/Link and Link Properties
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_TYPE] = static_cast<float>(tempOps.type[i]);

				U32 opLeftRightChild = (tempOps.opLeftChild[i] << 16) | tempOps.opRightChild[i];
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_CHILDREN] = static_cast<float>(opLeftRightChild);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS] = static_cast<float>(tempOps.opFlags[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_PARENT_LINK] = 0;

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
					U32 flags = static_cast<U32>(m_arrOps[idxLC * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS]);
					flags |= (linkFlagLC << 16);
					m_arrOps[idxLC * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS] = (float)flags;
					m_arrOps[idxLC * DATASIZE_OPERATOR + OFFSET_OP_PARENT_LINK] = static_cast<float>(linkLC);

					stkOps.push(idxLC);
				}
				else
				{
					m_arrPrims[idxLC * DATASIZE_PRIMITIVE + OFFSET_PRIM_LINK_FLAGS] = (float)(linkFlagLC << 16);
					m_arrPrims[idxLC * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT_LINK] = static_cast<float>(linkLC);
				}


				//RC
				if(isRCOp)
				{
					U32 flags = static_cast<U32>(m_arrOps[idxRC * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS]);
					flags |= (linkFlagRC << 16);
					m_arrOps[idxRC * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS] = (float)flags;
					m_arrOps[idxRC * DATASIZE_OPERATOR + OFFSET_OP_PARENT_LINK] = static_cast<float>(linkRC);
					stkOps.push(idxRC);
				}
				else
				{
					m_arrPrims[idxRC * DATASIZE_PRIMITIVE + OFFSET_PRIM_LINK_FLAGS] = (float)(linkFlagRC << 16);
					m_arrPrims[idxRC * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT_LINK] = static_cast<float>(linkRC);
				}

			}
		}

		//const int corner1[12]    = {LBN,LTN,LBN,LBF,RBN,RTN,RBN,RBF,LBN,LBF,LTN,LTF};
		//const int corner2[12]    = {LBF,LTF,LTN,LTF,RBF,RTF,RTN,RTF,RBN,RBF,RTN,RTF};
		//const int edgeaxis[12]   = {AAZ,AAZ,AAY,AAY,AAZ,AAZ,AAY,AAY,AAX,AAX,AAX,AAX};
		for(int i=0; i<12; i++)
		{
			m_cellParam.corner1[i] = (U8)corner1[i];
			m_cellParam.corner2[i] = (U8)corner2[i];
			m_cellParam.edgeaxis[i] = (U8)edgeaxis[i];
		}		

		//Model memories
		U32 ctOpsToCopy = MATHMAX(m_ctOps, 1);
		U32 ctPrimsToCopy = MATHMAX(m_ctPrims, 1);

		m_inMemHeader = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_HEADER, ComputeDevice::memReadOnly);
		m_inMemOps    = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, ComputeDevice::memReadOnly);
		m_inMemPrims  = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, ComputeDevice::memReadOnly);
		m_inMemMtx	  = m_lpGPU->createMemBuffer(sizeof(float) * PRIM_MATRIX_STRIDE * m_mtxNode.count, ComputeDevice::memReadOnly);

		//Header
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemHeader, sizeof(float) * DATASIZE_HEADER, m_arrHeader))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Ops
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemOps, sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, m_arrOps))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Prims
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemPrims, sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, m_arrPrims))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Matrix
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemMtx, sizeof(float) * PRIM_MATRIX_STRIDE * m_mtxNode.count, m_mtxNode.matrix))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;
		m_bModelLoaded = true;

		return SUCCESS;
	}


	void GPUPoly::drawBBox()
	{
		DrawBox(m_bboxLo, m_bboxHi, svec3f(0,0,1), 1.0f);
	}

	void GPUPoly::DrawBox(const svec3f& lo, const svec3f& hi, const svec3f& color, float lineWidth)
	{
		float l = lo.x; float r = hi.x;
		float b = lo.y; float t = hi.y;
		float n = lo.z; float f = hi.z;

		GLfloat vertices [][3] = {{l, b, f}, {l, t, f}, {r, t, f},
								  {r, b, f}, {l, b, n}, {l, t, n},
								  {r, t, n}, {r, b, n}};

		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColor3f(color.x, color.y, color.z);
			glLineWidth(lineWidth);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glBegin(GL_QUADS);
				glVertex3fv(vertices[0]); glVertex3fv(vertices[3]); glVertex3fv(vertices[2]); glVertex3fv(vertices[1]);
				glVertex3fv(vertices[4]); glVertex3fv(vertices[5]); glVertex3fv(vertices[6]); glVertex3fv(vertices[7]);
				glVertex3fv(vertices[3]); glVertex3fv(vertices[0]); glVertex3fv(vertices[4]); glVertex3fv(vertices[7]);
				glVertex3fv(vertices[1]); glVertex3fv(vertices[2]); glVertex3fv(vertices[6]); glVertex3fv(vertices[5]);
				glVertex3fv(vertices[2]); glVertex3fv(vertices[3]); glVertex3fv(vertices[7]); glVertex3fv(vertices[6]);
				glVertex3fv(vertices[5]); glVertex3fv(vertices[4]); glVertex3fv(vertices[0]); glVertex3fv(vertices[1]);
			glEnd();
		glPopAttrib();
	}

	//Code to produce number of vertices table
	void GPUPoly::ProduceNumVerticesTable(const char* chrOutput)
	{
		ofstream ofs;
		ofs.open(chrOutput, ios::out);
		for(size_t i=0; i < 256; i++)
		{
			int ctVertices = 0;
			for(int j=0; j<16; j++)
			{
				if(g_triTableCompact[i][j] != 255)
					ctVertices++;
				else
					break;
			}
			ofs << "\t" << ctVertices << "," << '\0' << endl;
		}
		ofs.close();
	}


	int GPUPoly::init()
	{
		//Vertex Shader Code
		const char * lpVertexShaderCode =
			"varying vec3 N;"
			"varying vec3 V; "
			"void main(void) {"
			"gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;"
			"gl_FrontColor = gl_Color;"
			"N = normalize(gl_NormalMatrix * gl_Normal);"
			"V = vec3(gl_ModelViewMatrix * gl_Vertex); }";

		//Fragment Shader Code
		const char* lpFragShaderCode =
			"varying vec3 N;"
			"varying vec3 V;"
			"void main(void) {"
			"vec3 L = normalize(gl_LightSource[0].position.xyz - V);"
			"vec3 E = normalize(-V);"
			"vec3 R = normalize(-reflect(L, N));"
			"vec4 Iamb = 0.5 * gl_LightSource[0].ambient * gl_Color;"
			"vec4 Idif = (gl_LightSource[0].diffuse * gl_Color) * max(dot(N,L), 0.0);"
			"vec4 Ispec = (gl_LightSource[0].specular * (vec4(0.8, 0.8, 0.8, 0.8) + 0.2 * gl_Color)) * pow(max(dot(R, E), 0.0), 32.0);"
			"gl_FragColor = gl_FrontLightModelProduct.sceneColor + Iamb + Idif + Ispec;	}";

		//Set Rendering Shader
		CompileShaderCode(lpVertexShaderCode, lpFragShaderCode, m_uShaderEffectProgram);
		this->setShaderEffectProgram(m_uShaderEffectProgram);
		
		//Reading kernels for opencl
		DAnsiStr strFP = ExtractFilePath(GetExePath());
		strFP = ExtractOneLevelUp(strFP);
		strFP += DAnsiStr("PS_Shaders/Polygonizer.cl");
		
		LogInfo("1.Setup compute device. Prefer AMD GPU.");
		//Create a GPU Compute Device
		m_lpGPU = new ComputeDevice(ComputeDevice::dtGPU, true, "AMD");
		m_lpGPU->printInfo();
		

		//Create the OCL Scan Primitive
		//PS::HPC::Scan* lpScanner = new PS::HPC::Scan(lpGPU);
		//lpScanner->scanExclusiveLarge()
		LogInfo("2.Compile OpenCL program.");
		ComputeProgram* lpProgram = m_lpGPU->addProgramFromFile(strFP.cptr());
		assert(lpProgram != NULL);

		//Build Kernel
		LogInfo("3.Setup Kernel Functions.");
		//m_lpKernelComputeConfig = lpProgram->addKernel("ComputeConfigIndexVertexCount");
		m_lpKernelComputeConfig = lpProgram->addKernel("ComputeConfig");
		m_lpKernelComputeMesh = lpProgram->addKernel("ComputeMesh");

		m_lpKernelComputeAllFields = lpProgram->addKernel("ComputeAllFields");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeAllFields);

		m_lpKernelComputeEdgeTable = lpProgram->addKernel("ComputeEdgeTable");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeEdgeTable);

		m_lpKernelComputeVertexAttribs = lpProgram->addKernel("ComputeVertexAttribs");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeVertexAttribs);

		m_lpKernelComputeCellConfigs = lpProgram->addKernel("ComputeCellConfigs");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeCellConfigs);

		m_lpKernelComputeElements = lpProgram->addKernel("ComputeElements");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeElements);


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
		m_bModelLoaded = false;
		return 1;
	}



	/*!
	 * Runs ComputeConfig and ComputeMesh kernels in tandem.
	 */
	int GPUPoly::runTandem(float cellsize)
	{
		//Clear previous mesh
		GLMeshBuffer::cleanup();

		//BBOX
		svec3f lower = svec3f(m_arrHeader[0], m_arrHeader[1], m_arrHeader[2]);
		svec3f upper = svec3f(m_arrHeader[4], m_arrHeader[5], m_arrHeader[6]);
		svec3f temp = vscale3f(1.0f/ cellsize, vsub3f(upper, lower));
		
		//Cell Params
		m_cellParam.ctNeededCells[0] = (U32)ceil(temp.x) + 1;
		m_cellParam.ctNeededCells[1] = (U32)ceil(temp.y) + 1;
		m_cellParam.ctNeededCells[2] = (U32)ceil(temp.z) + 1;
		m_cellParam.ctTotalCells = m_cellParam.ctNeededCells[0] * m_cellParam.ctNeededCells[1] * m_cellParam.ctNeededCells[2];
		m_cellParam.cellsize = cellsize;

		//Check Matrices
		for(U32 i = 0; i < m_mtxNode.count; i++)
		{
			printf("PRIM MTX: %d\n", i);
			for(U32 j=0;j < PRIM_MATRIX_STRIDE; j++)
			{
				printf("%.2f, ", m_mtxNode.matrix[i * PRIM_MATRIX_STRIDE + j]);
				if((j+1) % 4 == 0)
					printf("\n");
			}
			//printf("\n");
		}

		//CellConfig and VertexCount
		cl_mem inoutMemCellConfig = m_lpGPU->createMemBuffer(sizeof(U8) * m_cellParam.ctTotalCells, ComputeDevice::memReadWrite);
		cl_mem inoutMemVertexCount = m_lpGPU->createMemBuffer(sizeof(U8) * m_cellParam.ctTotalCells, ComputeDevice::memReadWrite);
			
		//CellParams
		cl_mem inMemCellParams = m_lpGPU->createMemBuffer(sizeof(CellParam), ComputeDevice::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(inMemCellParams, sizeof(CellParam), &m_cellParam))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;
		
		//Kernel1
		/*
		__kernel void ComputeConfig(__global float4* arrInHeader4,
							__global float4* arrInOps4,										 
							__global float4* arrInPrims4,											 
	 					    __global float4* arrInMtxNodes4,
							__read_only image2d_t texInVertexCountTable,
							__constant struct CellParam* inCellParams,
							__global uchar* arrOutCellConfig,
							__global uchar* arrOutVertexCount)								
		*/		
		//Set the arguments to the compute kernel	
		//Setup Kernel Call, input params first then the output structures
		m_lpKernelComputeConfig->setArg(0, sizeof(cl_mem), &m_inMemHeader);
		m_lpKernelComputeConfig->setArg(1, sizeof(cl_mem), &m_inMemOps);
		m_lpKernelComputeConfig->setArg(2, sizeof(cl_mem), &m_inMemPrims);
		m_lpKernelComputeConfig->setArg(3, sizeof(cl_mem), &m_inMemMtx);
		m_lpKernelComputeConfig->setArg(4, sizeof(cl_mem), &m_inMemVertexCountTable);
		m_lpKernelComputeConfig->setArg(5, sizeof(cl_mem), &inMemCellParams);
		m_lpKernelComputeConfig->setArg(6, sizeof(cl_mem), &inoutMemCellConfig);
		m_lpKernelComputeConfig->setArg(7, sizeof(cl_mem), &inoutMemVertexCount);

	
		size_t local = m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeConfig);

		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
		size_t szNeeded[3];
		for(int i=0; i<3; i++)
			szNeeded[i] = m_cellParam.ctNeededCells[i];

		//Run Config Kernel to classify the cells
		cl_int err = clEnqueueNDRangeKernel(m_lpGPU->getCommandQ(), m_lpKernelComputeConfig->getKernel(),
											3, NULL, szNeeded, NULL, 0, NULL, NULL);
		if (err) {
			LogErrorArg1("Error: Failed to execute ComputeConfig kernel! (%s)", ComputeDevice::oclErrorString(err));
			return EXIT_FAILURE;
		}

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();

		///////////////////////////////////////////////////////////////////////////////
		// Read back the results from the device to verify the output
		U8* arrVertexCount = new U8[m_cellParam.ctTotalCells];
		U8* arrConfigIndex = new U8[m_cellParam.ctTotalCells];
		U32* arrVertexBufferOffset = new U32[m_cellParam.ctTotalCells];
		U32* arrTetMeshOffset = new U32[m_cellParam.ctTotalCells];

		U32 ctSumVertices = 0;
		U32 ctCrossedOnly = 0;
		U32 ctCrossedOrInside = 0;
		if(m_lpGPU->enqueueReadBuffer(inoutMemVertexCount, sizeof(U8) * m_cellParam.ctTotalCells, arrVertexCount) &&
		   m_lpGPU->enqueueReadBuffer(inoutMemCellConfig, sizeof(U8) * m_cellParam.ctTotalCells, arrConfigIndex))
		{						
			//Compute Vertex Buffer offsets for writing the output to
			for(U32 i=0; i < m_cellParam.ctTotalCells; i++)
			{
				arrVertexBufferOffset[i] = ctSumVertices;
				arrTetMeshOffset[i] = ctCrossedOrInside;
				ctSumVertices += arrVertexCount[i];	
			
				if((arrConfigIndex[i] != 0)&&(arrConfigIndex[i] != 255))	
					ctCrossedOnly ++;

				if(arrConfigIndex[i] != 0)
					ctCrossedOrInside ++;
			}
		}

		//if we return here
		if(ctCrossedOnly == 0)
		{
			LogWarning("No cells crossed by the boundary surface.");
		}

		//Memory Buffers for the TetMesh
		//6 Tetrahedra per each inside cube
		//TODO: Variable tetrahedra per crossed cube
		U32 ctTets = ctCrossedOrInside * 6;
		//(x,y,z) * number of cubes * 8 cube corners
		cl_mem outMemTetMeshVertices = m_lpGPU->createMemBuffer(sizeof(float) * 3 * ctCrossedOrInside * 8, ComputeDevice::memWriteOnly);

		//(a,b,c,d) * number of cubes * 6 Tetrahedra per cube
		cl_mem outMemTetMeshIndices = m_lpGPU->createMemBuffer(sizeof(U32) * 4 * ctCrossedOrInside * 6, ComputeDevice::memWriteOnly);


		//Vertex buffer offset
		cl_mem inMemVertexBufferOffset;
		inMemVertexBufferOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, ComputeDevice::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(inMemVertexBufferOffset, sizeof(U32) * m_cellParam.ctTotalCells, arrVertexBufferOffset))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Tetmesh buffer offset
		cl_mem inMemTetMeshBufferOffset;
		inMemTetMeshBufferOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, ComputeDevice::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(inMemTetMeshBufferOffset, sizeof(U32) * m_cellParam.ctTotalCells, arrTetMeshOffset))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//////////////////////////////////////////////////////////////////////////
		//Kernel2:
		m_stepVertex = 4;
		m_stepColor = 4;
		GLsizeiptr szVertexBuffer = ctSumVertices * m_stepVertex * sizeof(float);
		GLsizeiptr szNormalBuffer = ctSumVertices * 3 * sizeof(float);

		//Vertex
		glGenBuffers(1, &m_vboVertex);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshVertex = m_lpGPU->createMemBufferFromGL(m_vboVertex, ComputeDevice::memWriteOnly);

		//Color
		glGenBuffers(1, &m_vboColor);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshColor = m_lpGPU->createMemBufferFromGL(m_vboColor, ComputeDevice::memWriteOnly);

		//Normal
		glGenBuffers(1, &m_vboNormal);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboNormal);
		glBufferData(GL_ARRAY_BUFFER, szNormalBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshNormal = m_lpGPU->createMemBufferFromGL(m_vboNormal, ComputeDevice::memWriteOnly);

		//Inputs
		m_lpKernelComputeMesh->setArg(0, sizeof(cl_mem), &m_inMemHeader);
		m_lpKernelComputeMesh->setArg(1, sizeof(cl_mem), &m_inMemOps);
		m_lpKernelComputeMesh->setArg(2, sizeof(cl_mem), &m_inMemPrims);
		m_lpKernelComputeMesh->setArg(3, sizeof(cl_mem), &m_inMemMtx);
		m_lpKernelComputeMesh->setArg(4, sizeof(cl_mem), &m_inMemTriangleTable);
		m_lpKernelComputeMesh->setArg(5, sizeof(cl_mem), &inMemCellParams);
		m_lpKernelComputeMesh->setArg(6, sizeof(cl_mem), &inoutMemCellConfig);
		m_lpKernelComputeMesh->setArg(7, sizeof(cl_mem), &inMemVertexBufferOffset);

		//Tetmesh
		m_lpKernelComputeMesh->setArg(8, sizeof(cl_mem), &inMemTetMeshBufferOffset);
		m_lpKernelComputeMesh->setArg(9, sizeof(cl_mem), &outMemTetMeshVertices);
		m_lpKernelComputeMesh->setArg(10, sizeof(cl_mem), &outMemTetMeshIndices);


		//Output mesh
		m_lpKernelComputeMesh->setArg(11, sizeof(cl_mem), &outMemMeshVertex);
		m_lpKernelComputeMesh->setArg(12, sizeof(cl_mem), &outMemMeshColor);
		m_lpKernelComputeMesh->setArg(13, sizeof(cl_mem), &outMemMeshNormal);
		

		//Acquire Mesh for Writing
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshNormal, 0, 0, 0);
		


		err = clEnqueueNDRangeKernel(m_lpGPU->getCommandQ(), m_lpKernelComputeMesh->getKernel(),
									 3, NULL, szNeeded, NULL, 0, NULL, NULL);
		if (err) {
			LogErrorArg1("Error: Failed to execute ComputeMesh kernel! (%s)", ComputeDevice::oclErrorString(err));
			return EXIT_FAILURE;
		}

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();

		//Release Mesh for Writing
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshNormal, 0, 0, 0);
		m_isValidVertex = m_isValidColor = m_isValidNormal = true;
		m_ctVertices = ctSumVertices;
		m_ctFaceElements = ctSumVertices;
		//m_ctFaceElements = ctSumVertices / 3;

		//Export VEGA
		/*
		{
			U32 ctTetMeshVertices = ctCrossedOrInside * 8;
			U32 ctTetMeshElements = ctCrossedOrInside * 6;
			float* arrTetMeshVertices = new float[3 * ctTetMeshVertices];
			U32* arrTetMeshIndices = new U32[4 * ctTetMeshElements];
			m_lpGPU->enqueueReadBuffer(outMemTetMeshVertices, sizeof(float) * 3 * ctTetMeshVertices, arrTetMeshVertices);
			m_lpGPU->enqueueReadBuffer(outMemTetMeshIndices, sizeof(U32) * 4 * ctTetMeshElements, arrTetMeshIndices);


			ofstream ofs;
			ofs.open("tetmesh.veg", ios::out);
			ofs << "# Vega Mesh File, Generated by FemBrain." << endl;
			ofs << "# " << ctTetMeshVertices << " vertices, " << ctTetMeshElements << " elements" << endl;
			ofs << endl;
			ofs << "*VERTICES" << endl;
			ofs << ctTetMeshVertices << " 3 0 0" << endl;
			for(U32 i=0; i < ctTetMeshVertices; i++)
			{
				ofs << i+1 << " " << arrTetMeshVertices[i*3 + 0] << " " << arrTetMeshVertices[i*3 + 1] << " " << arrTetMeshVertices[i*3 + 2] << endl;
			}

			ofs << endl;
			ofs << "*ELEMENTS" << endl;
			ofs << "TET" << endl;
			ofs << ctTetMeshElements << " 4 0"<< endl;
			for(U32 i=0; i < ctTetMeshElements; i++)
			{
				ofs << i+1 << " " << arrTetMeshIndices[i*4 + 0] + 1 << " " << arrTetMeshIndices[i*4 + 1] + 1 << " " << arrTetMeshIndices[i*4 + 2] + 1 << " " << arrTetMeshIndices[i*4 + 3] + 1 << endl;
			}

			ofs << endl;
			ofs << "*MATERIAL BODY" << endl;
			ofs << "ENU, 1000, 10000000, 0.45" << endl;
			ofs << endl;

			ofs << "*REGION" << endl;
			ofs << "allElements, BODY" << endl;
			ofs.close();
		}

		//Export offsets
		{
			ofstream ofs;
			ofs.open("mesh_output_offsets.txt", ios::out);
			for(U32 i=0; i < m_ctCells; i++)
			{
				int ctVertices = arrVertexCount[i];
				int idxConfig = arrConfigIndex[i];
				ofs << "Vertex Offset " << i << " = " << arrVertexBufferOffset[i];
				ofs << "\t Vertex Count = " << ctVertices;
				ofs << "\t Config Index = " << idxConfig << endl;
			}
			ofs.close();
		}

		//Export output VBO
		{
			float* arrMeshVertices = new float[4 * ctSumVertices];
			if(m_lpGPU->enqueueReadBuffer(outMemMeshVertex, sizeof(float) * m_stepVertex * ctSumVertices, arrMeshVertices))
			{
				ofstream ofs;
				ofs.open("mesh_output_vbo.txt", ios::out);
				for(U32 i=0; i < ctSumVertices; i++)
				{
					ofs << "\t Vertex " << i << " [";
					for(U32 j=0; j<4; j++)
						ofs << "\t" << arrMeshVertices[i*4+j];
					 ofs << "]" << endl;
				}
				ofs.close();
			}
			SAFE_DELETE(arrMeshVertices);
		}
		*/

		// Shutdown and cleanup
		clReleaseMemObject(inoutMemCellConfig);
		clReleaseMemObject(inoutMemVertexCount);

		clReleaseMemObject(inMemCellParams);

		SAFE_DELETE(arrConfigIndex);
		SAFE_DELETE(arrVertexCount);
		SAFE_DELETE(arrVertexBufferOffset);
		SAFE_DELETE(arrTetMeshOffset);
		return 1;
	}

	//Runs a multipass High Performance Polygonizer
	int GPUPoly::runMultiPass(float cellsize)
	{
		//Clear previous mesh
		GLMeshBuffer::cleanup();

		//1. Compute All Fields
		computeAllFields(cellsize);

		//2. Compute Edge Table
		computeEdgeTable();
		{
			//Read-back EdgeTable Count
			U32* lpEdgeTableOffsets = new U32[m_gridParam.ctTotalPoints];
			m_lpGPU->enqueueReadBuffer(m_inoutMemHighEdgesCount, sizeof(U32) * m_gridParam.ctTotalPoints, lpEdgeTableOffsets);
			PrintArray(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);

			//3. SumScan to compute crossed edges
			m_ctVertices = m_lpOclSumScan->compute(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);
			PrintArray(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);

			//Write Edge Offsets to device memory
			m_inMemHighEdgesOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_gridParam.ctTotalPoints, ComputeDevice::memReadOnly);
			m_lpGPU->enqueueWriteBuffer(m_inMemHighEdgesOffset, sizeof(U32) * m_gridParam.ctTotalPoints, lpEdgeTableOffsets);
			SAFE_DELETE(lpEdgeTableOffsets);
		}

		//4. Compute VertexAttribs
		computeVertexAttribs(m_ctVertices);

		//5. Compute Cell Configs
		computeCellConfigs();
		{
			//Read-back Elements Count
			U32* lpCellElementsOffset = new U32[m_cellParam.ctTotalCells];
			m_lpGPU->enqueueReadBuffer(m_inoutMemCellElementsCount, sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);
			PrintArray(lpCellElementsOffset, m_cellParam.ctTotalCells);

			//6. SumScan to read all elements
			m_ctFaceElements = m_lpOclSumScan->compute(lpCellElementsOffset, m_cellParam.ctTotalCells);
			PrintArray(lpCellElementsOffset, m_cellParam.ctTotalCells);

			//Write Element Offsets
			m_inMemCellElementsOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, ComputeDevice::memReadOnly);
			m_lpGPU->enqueueWriteBuffer(m_inMemCellElementsOffset, sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);
			SAFE_DELETE(lpCellElementsOffset);
		}

		//7. Compute Elements
		computeElements(m_ctFaceElements);


		//8. Draw Mesh
		m_isValidIndex = true;
		m_faceMode = ftTriangles;


		/*
		m_isValidIndex = false;
		m_ctFaceElements = m_ctVertices;
		m_faceMode = ftPoints;
		*/


		return 1;
	}

	int GPUPoly::computeElements(U32 ctElements)
	{
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_cellParam.ctNeededCells[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelComputeEdgeTable->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		//Elements
		glGenBuffers(1, &m_iboFaces);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboFaces);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(U32) * ctElements, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshElements = m_lpGPU->createMemBufferFromGL(m_iboFaces, ComputeDevice::memWriteOnly);

		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshElements, 0, 0, 0);
		/*
		__kernel void ComputeElements(__global U32* arrInHighEdgesOffset,
				   	   	   	   	   	  __global U8* arrInHighEdgesFlags,
				   	   	   	   	   	  __global U8* arrInCellConfig,
									  __global U32* arrInCellElementCount,
									  __global U32* arrInCellElementOffset,
									  __read_only image2d_t texInTriangleTable,
									  __constant struct GridParam* inGridParam,
									  __constant struct CellParam* inCellParam,
									  __global U32* arrOutElements)
		*/
		m_lpKernelComputeElements->setArg(0, sizeof(cl_mem), &m_inMemHighEdgesOffset);
		m_lpKernelComputeElements->setArg(1, sizeof(cl_mem), &m_inoutMemHighEdgesFlags);
		m_lpKernelComputeElements->setArg(2, sizeof(cl_mem), &m_inoutMemCellConfig);
		m_lpKernelComputeElements->setArg(3, sizeof(cl_mem), &m_inoutMemCellElementsCount);
		m_lpKernelComputeElements->setArg(4, sizeof(cl_mem), &m_inMemCellElementsOffset);
		m_lpKernelComputeElements->setArg(5, sizeof(cl_mem), &m_inMemTriangleTable);
		m_lpKernelComputeElements->setArg(6, sizeof(cl_mem), &m_inMemGridParam);
		m_lpKernelComputeElements->setArg(7, sizeof(cl_mem), &m_inMemCellParam);
		m_lpKernelComputeElements->setArg(8, sizeof(cl_mem), &outMemMeshElements);

		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeElements, 3, arrGlobalIndex, arrLocalIndex);
		m_lpGPU->finishAllCommands();

		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshElements, 0, 0, 0);

		//Check Indices
		U32* arrIndices = new U32[m_ctFaceElements];
		if(m_lpGPU->enqueueReadBuffer(outMemMeshElements, sizeof(U32) * m_ctFaceElements, arrIndices))
		{
			PrintArray(arrIndices, m_ctFaceElements);
			U32 ctErrors = 0;
			for(U32 i=0; i<m_ctFaceElements; i++)
			{
				if(arrIndices[i] >= m_ctVertices)
					ctErrors++;
			}
			printf("Error: %d\n", ctErrors);
		}


		return 1;
	}

	int GPUPoly::computeCellConfigs()
	{
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_cellParam.ctNeededCells[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelComputeEdgeTable->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		m_inoutMemCellElementsCount = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, ComputeDevice::memReadWrite);
		m_inoutMemCellConfig = m_lpGPU->createMemBuffer(sizeof(U8) * m_cellParam.ctTotalCells, ComputeDevice::memReadWrite);

		/*
		__kernel void ComputeCellConfigs(__global float4* arrInFields,
										 __read_only image2d_t texInVertexCountTable,
										 __constant struct GridParam* inGridParam,
										 __constant struct CellParam* inCellParam,
										 __global U32* arrOutCellElementsCount,
										 __global U8* arrOutCellConfig)
		*/
		m_lpKernelComputeCellConfigs->setArg(0, sizeof(cl_mem), &m_inoutMemAllFields);
		m_lpKernelComputeCellConfigs->setArg(1, sizeof(cl_mem), &m_inMemVertexCountTable);
		m_lpKernelComputeCellConfigs->setArg(2, sizeof(cl_mem), &m_inMemGridParam);
		m_lpKernelComputeCellConfigs->setArg(3, sizeof(cl_mem), &m_inMemCellParam);
		m_lpKernelComputeCellConfigs->setArg(4, sizeof(cl_mem), &m_inoutMemCellElementsCount);
		m_lpKernelComputeCellConfigs->setArg(5, sizeof(cl_mem), &m_inoutMemCellConfig);

		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeCellConfigs, 3, arrGlobalIndex, arrLocalIndex);
		m_lpGPU->finishAllCommands();

		return 1;
	}

	int GPUPoly::computeVertexAttribs(U32 ctVertices)
	{
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_gridParam.ctGridPoints[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelComputeEdgeTable->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		//Vertex Attrib Buffers
		m_stepVertex = 4;
		m_stepColor = 4;
		GLsizeiptr szVertexBuffer = ctVertices * m_stepVertex * sizeof(float);
		GLsizeiptr szNormalBuffer = ctVertices * 3 * sizeof(float);

		//Vertex
		glGenBuffers(1, &m_vboVertex);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshVertex = m_lpGPU->createMemBufferFromGL(m_vboVertex, ComputeDevice::memWriteOnly);

		//Color
		glGenBuffers(1, &m_vboColor);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshColor = m_lpGPU->createMemBufferFromGL(m_vboColor, ComputeDevice::memWriteOnly);

		//Normal
		glGenBuffers(1, &m_vboNormal);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboNormal);
		glBufferData(GL_ARRAY_BUFFER, szNormalBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshNormal = m_lpGPU->createMemBufferFromGL(m_vboNormal, ComputeDevice::memWriteOnly);

		//Acquire Mesh for Writing
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshNormal, 0, 0, 0);

		/*
		__kernel void ComputeVertexAttribs(__global float4* arrInHeader4,
										   __global float4* arrInOps4,
										   __global float4* arrInPrims4,
										   __global float4* arrInMtxNodes4,
										   __global float4* arrInFields,
										   __global U32* arrInHighEdgesCount,
										   __global U32* arrInHighEdgesOffset,
										   __global U8* arrInHighEdgesFlags,
										   __constant struct GridParam* inGridParam,
										   __global float4* arrOutMeshVertex,
										   __global float4* arrOutMeshColor,
										   __global float* arrOutMeshNormal)
		*/
		m_lpKernelComputeVertexAttribs->setArg(0, sizeof(cl_mem), &m_inMemHeader);
		m_lpKernelComputeVertexAttribs->setArg(1, sizeof(cl_mem), &m_inMemOps);
		m_lpKernelComputeVertexAttribs->setArg(2, sizeof(cl_mem), &m_inMemPrims);
		m_lpKernelComputeVertexAttribs->setArg(3, sizeof(cl_mem), &m_inMemMtx);
		m_lpKernelComputeVertexAttribs->setArg(4, sizeof(cl_mem), &m_inoutMemAllFields);
		m_lpKernelComputeVertexAttribs->setArg(5, sizeof(cl_mem), &m_inoutMemHighEdgesCount);
		m_lpKernelComputeVertexAttribs->setArg(6, sizeof(cl_mem), &m_inMemHighEdgesOffset);
		m_lpKernelComputeVertexAttribs->setArg(7, sizeof(cl_mem), &m_inoutMemHighEdgesFlags);
		m_lpKernelComputeVertexAttribs->setArg(8, sizeof(cl_mem), &m_inMemGridParam);
		m_lpKernelComputeVertexAttribs->setArg(9, sizeof(cl_mem), &outMemMeshVertex);
		m_lpKernelComputeVertexAttribs->setArg(10, sizeof(cl_mem), &outMemMeshColor);
		m_lpKernelComputeVertexAttribs->setArg(11, sizeof(cl_mem), &outMemMeshNormal);

		//Enqueue Kernel
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeVertexAttribs, 3, arrGlobalIndex, arrLocalIndex);
		m_lpGPU->finishAllCommands();


		//Release Mesh for Writing
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshNormal, 0, 0, 0);

		//Vertices
		m_isValidVertex = m_isValidColor = m_isValidNormal = true;
		m_ctVertices = ctVertices;

		//Read Vertices
		/*
		float* arrVertices = new float[m_ctVertices*4];
		if(m_lpGPU->enqueueReadBuffer(outMemMeshVertex, sizeof(float) * 4 * m_ctVertices, arrVertices))
			PrintArrayF(arrVertices, 4 * m_ctVertices);
		SAFE_DELETE(arrVertices);
		*/

		return 1;
	}

	int GPUPoly::computeEdgeTable()
	{
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_gridParam.ctGridPoints[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelComputeEdgeTable->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		//Buffers for edge table
		m_inoutMemHighEdgesCount = m_lpGPU->createMemBuffer(sizeof(U32) * m_gridParam.ctTotalPoints, ComputeDevice::memReadWrite);
		m_inoutMemHighEdgesFlags = m_lpGPU->createMemBuffer(sizeof(U8) * m_gridParam.ctTotalPoints, ComputeDevice::memReadWrite);

		/*
		 * __kernel void ComputeEdgeTable(__global float4* arrInFields,
							   __constant struct GridParam* inGridParam,
							   __global U32* arrOutHighEdgesCount,
							   __global U8* arrOutHighEdgesFlags)
		 */
		m_lpKernelComputeEdgeTable->setArg(0, sizeof(cl_mem), &m_inoutMemAllFields);
		m_lpKernelComputeEdgeTable->setArg(1, sizeof(cl_mem), &m_inMemGridParam);
		m_lpKernelComputeEdgeTable->setArg(2, sizeof(cl_mem), &m_inoutMemHighEdgesCount);
		m_lpKernelComputeEdgeTable->setArg(3, sizeof(cl_mem), &m_inoutMemHighEdgesFlags);


		cl_int err = clEnqueueNDRangeKernel(m_lpGPU->getCommandQ(), m_lpKernelComputeEdgeTable->getKernel(),
									 	    3, NULL, arrGlobalIndex, arrLocalIndex, 0, NULL, NULL);
		if (err) {
			LogErrorArg1("Error: Failed to execute ComputeAllFields kernel! (%s)", ComputeDevice::oclErrorString(err));
			return EXIT_FAILURE;
		}

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();


		return 1;
	}

	int GPUPoly::computeAllFields(float cellsize)
	{
		//BBOX
		svec3f lower = svec3f(m_arrHeader[0], m_arrHeader[1], m_arrHeader[2]);
		svec3f upper = svec3f(m_arrHeader[4], m_arrHeader[5], m_arrHeader[6]);
		svec3f temp = vscale3f(1.0f/ cellsize, vsub3f(upper, lower));

		//Cell Params
		m_cellParam.ctNeededCells[0] = (U32)ceil(temp.x) + 1;
		m_cellParam.ctNeededCells[1] = (U32)ceil(temp.y) + 1;
		m_cellParam.ctNeededCells[2] = (U32)ceil(temp.z) + 1;
		m_cellParam.ctTotalCells = m_cellParam.ctNeededCells[0] * m_cellParam.ctNeededCells[1] * m_cellParam.ctNeededCells[2];
		m_cellParam.cellsize = cellsize;

		//Grid Params
		m_gridParam.ctGridPoints[0] = m_cellParam.ctNeededCells[0] + 1;
		m_gridParam.ctGridPoints[1] = m_cellParam.ctNeededCells[1] + 1;
		m_gridParam.ctGridPoints[2] = m_cellParam.ctNeededCells[2] + 1;
		m_gridParam.ctTotalPoints = m_gridParam.ctGridPoints[0] * m_gridParam.ctGridPoints[1] * m_gridParam.ctGridPoints[2];
		m_gridParam.cellsize = cellsize;


		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_gridParam.ctGridPoints[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelComputeAllFields->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		//CellParam
		m_inMemCellParam = m_lpGPU->createMemBuffer(sizeof(CellParam), ComputeDevice::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemCellParam, sizeof(CellParam), &m_cellParam))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//GridParam
		m_inMemGridParam = m_lpGPU->createMemBuffer(sizeof(GridParam), ComputeDevice::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemGridParam, sizeof(GridParam), &m_gridParam))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Create Memory for all field points
		m_inoutMemAllFields = m_lpGPU->createMemBuffer(sizeof(float) * 4 * m_gridParam.ctTotalPoints, ComputeDevice::memReadWrite);

		/*
		__kernel void ComputeAllFields(__global float4* arrInHeader4,
									   __global float4* arrInOps4,
									   __global float4* arrInPrims4,
									   __global float4* arrInMtxNodes4,
									   __constant struct GridParam* inGridParam,
									   __global float4* arrOutFields)
									   */
		m_lpKernelComputeAllFields->setArg(0, sizeof(cl_mem), &m_inMemHeader);
		m_lpKernelComputeAllFields->setArg(1, sizeof(cl_mem), &m_inMemOps);
		m_lpKernelComputeAllFields->setArg(2, sizeof(cl_mem), &m_inMemPrims);
		m_lpKernelComputeAllFields->setArg(3, sizeof(cl_mem), &m_inMemMtx);
		m_lpKernelComputeAllFields->setArg(4, sizeof(cl_mem), &m_inMemGridParam);
		m_lpKernelComputeAllFields->setArg(5, sizeof(cl_mem), &m_inoutMemAllFields);

		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
		cl_int err = clEnqueueNDRangeKernel(m_lpGPU->getCommandQ(), m_lpKernelComputeAllFields->getKernel(),
									 	    3, NULL, arrGlobalIndex, arrLocalIndex, 0, NULL, NULL);
		if (err) {
			LogErrorArg1("Error: Failed to execute ComputeAllFields kernel! (%s)", ComputeDevice::oclErrorString(err));
			return EXIT_FAILURE;
		}

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();

		return 1;
	}



}
}



