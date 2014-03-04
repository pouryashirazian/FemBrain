/*
 * PS_OclPolygonizer.cpp
 *
 *  Created on: 2012-02-20
 *      Author: pourya
 */
#include <iostream>
#include "OclPolygonizer.h"
#include "base/DebugUtils.h"
#include "base/FileDirectory.h"
#include "base/Logger.h"
#include "base/Profiler.h"
#include "ReadSceneModel.h"
#include "_CellConfigTable.h"
#include "_CellConfigTableCompact.h"
#include "graphics/OclSumScan.h"
#include "graphics/OclScan.h"
#include "graphics/OclHelperFuncs.h"
#include "graphics/CLManager.h"



#define __NO_STD_VECTOR // Use cl::vector and cl::string and
#define __NO_STD_STRING // not STL versions, more on this later


using namespace std;
using namespace PS;
using namespace PS::GL;
using namespace PS::SG;
using namespace PS::FILESTRINGUTILS;
using namespace PS::CL;

#define SUCCESS 1
#define ERR_GPUPOLY_KERNEL_NOT_BUILT -1
#define ERR_GPUPOLY_BUFFER_NOT_WRITTEN -2
#define ERR_GPUPOLY_BUFFER_NOT_READ -3
#define ERR_GPUPOLY_TRITABLE_NOT_READ -4
#define ERR_GPUPOLY_VERTEXTABLE_NOT_READ -5
#define ERR_GPUPOLY_FIELDSTABLE		-6
#define ERR_NO_CROSSED_CELLS 		-7
#define ERR_INVALID_INPUT_PARAM 	-8


#define MAX_VERTICES_COUNT_PER_CELL		15
#define MAX_TRIANGLES_COUNT_PER_CELL	5



extern unsigned char g_triTableCompact[256][16];

namespace PS {
namespace SKETCH {

	GPUPoly::GPUPoly() : SG::SGMesh()
	{
		init();		
	}

	GPUPoly::GPUPoly(const LinearBlobTree& blob):SG::SGMesh() {
		init();
		this->setBlob(blob);
	}

	GPUPoly::~GPUPoly()
	{
		GLMeshBuffer::cleanup();

		//Cleanup rest pos vertices
        if(m_inoutMemRestPos)
            clReleaseMemObject(m_inoutMemRestPos);

		//Clear MC Tables
        if(m_inMemTriangleTable)
            clReleaseMemObject(m_inMemTriangleTable);
        if(m_inMemVertexCountTable)
            clReleaseMemObject(m_inMemVertexCountTable);

		//Clear Buffer
		clearBlobBuffer();
		SAFE_DELETE(m_lpOclSumScan);
	}

	void GPUPoly::clearBlobBuffer() {
		//Clear Mem Objects
		if(!m_bModelLoaded)
			return;

		clReleaseMemObject(m_inMemHeader);
		clReleaseMemObject(m_inMemOps);
		clReleaseMemObject(m_inMemPrims);
		clReleaseMemObject(m_inMemMtx);
		m_bModelLoaded = false;
	}

	GLMeshBuffer* GPUPoly::prepareMeshBufferForDrawingNormals(float len) {
		if(!m_isValidVertex || !m_isValidNormal)
			return NULL;

		U32 ctVertices;
		vector<float> vertices;
		vector<float> normals;
		readBackNormals(ctVertices, vertices, normals);

		vector<float> allvertices;
		//vector<U32> elements;

		allvertices.resize(ctVertices* 3 * 2);
		//elements.resize(ctVertices * 2);
		for(U32 i=0; i < ctVertices; i++) {
			vec3f ptStart = vec3f(&vertices[i*3]);
			vec3f ptEnd = ptStart + vec3f(&normals[i*3]) * len;
			allvertices[i*6] = ptStart.x;
			allvertices[i*6 + 1] = ptStart.y;
			allvertices[i*6 + 2] = ptStart.z;
			allvertices[i*6 + 3] = ptEnd.x;
			allvertices[i*6 + 4] = ptEnd.y;
			allvertices[i*6 + 5] = ptEnd.z;
		}

		//Create scene node
		GLMeshBuffer* lpDrawNormal = new GLMeshBuffer();
		lpDrawNormal->setupPerVertexColor(vec4f(0,0,1,1), ctVertices*2, 4);
		lpDrawNormal->setupVertexAttribs(allvertices, 3, mbtPosition);
		lpDrawNormal->setFaceMode(ftLines);

		return lpDrawNormal;
	}

	void GPUPoly::drawNormals() {
		if(!m_isValidVertex || !m_isValidNormal)
			return;

		U32 ctVertices;
		vector<float> vertices;
		vector<float> normals;

		readBackNormals(ctVertices, vertices, normals);

		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glColor3f(0, 0, 1);
		glBegin(GL_LINES);
		for(U32 i=0; i < ctVertices; i++) {
			vec3f ptStart = vec3f(&vertices[i*3]);
			vec3f ptEnd = ptStart + vec3f(&normals[i*3]) * 0.3f;
			glVertex3fv(ptStart.ptr());
			glVertex3fv(ptEnd.ptr());
		}

		glEnd();
		glPopAttrib();

	}

	void GPUPoly::DrawBox(const vec3f& lo, const vec3f& hi, const vec3f& color, float lineWidth)
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
		//Set Rendering Shader
        if(TheShaderManager::Instance().has("phong")) {
            m_spEffect = SmartPtrSGEffect(new SGEffect(TheShaderManager::Instance().get("phong")));
        }
		
		//Reading kernels for opencl
		AnsiStr strCodePath = ExtractFilePath(GetExePath());
		strCodePath = ExtractOneLevelUp(strCodePath);
		
		AnsiStr strPolyFP = strCodePath + AnsiStr("data/shaders/Polygonizer.cl");
		AnsiStr strTetFP = strCodePath + AnsiStr("data/shaders/Tetrahedralizer.cl");

		LogInfo("1.Setup compute device. Prefer AMD GPU.");
		//Create a GPU Compute Device
		m_lpGPU = TheCLManager::Instance().device();
		

		//Create the OCL Scan Primitive
		//PS::HPC::Scan* lpScanner = new PS::HPC::Scan(lpGPU);
		//lpScanner->scanExclusiveLarge()
		LogInfo("2.Compile OpenCL polygonizer program.");
		int prgID = m_lpGPU->addProgramFromFile(strPolyFP);
		assert(prgID >= 0);


		//Build Kernel
		LogInfo("3.Setup Kernel Functions.");
		//m_lpKernelComputeConfig = lpProgram->addKernel("ComputeConfigIndexVertexCount");
		//m_lpKernelComputeConfig = lpProgram->addKernel("ComputeConfig");
		//m_lpKernelComputeMesh = lpProgram->addKernel("ComputeMesh");

		//Polygonizer
		m_lpKernelComputeAllFields = m_lpGPU->addKernel(prgID, "ComputeAllFields");
		m_lpKernelComputeEdgeTable = m_lpGPU->addKernel(prgID, "ComputeEdgeTable");
		m_lpKernelComputeVertexAttribs = m_lpGPU->addKernel(prgID, "ComputeVertexAttribs");
		m_lpKernelComputeCellConfigs = m_lpGPU->addKernel(prgID, "ComputeCellConfigs");
		m_lpKernelComputeElements = m_lpGPU->addKernel(prgID, "ComputeElements");


		//Apply Deformations to Polygonized Mesh
		m_lpKernelApplyDeformations = m_lpGPU->addKernel(prgID, "ApplyVertexDeformations");
		m_lpKernelComputeFieldArray = m_lpGPU->addKernel(prgID, "ComputeFieldArray");
        m_lpKernelComputeFieldImage = m_lpGPU->addKernel(prgID, "ComputeFieldImage");
		m_lpKernelComputeOffSurfacePointsAndFields = m_lpGPU->addKernel(prgID, "ComputeOffSurfacePointsAndFields");



		//Tetrahedralizer
		/*
		LogInfo("3.Compile OpenCL Tetrahedralizer program");
		ComputeProgram* lpProgramTet = m_lpGPU->tryLoadBinaryThenCompile(strTetFP.cptr());
		assert(lpProgramTet != NULL);

		m_lpKernelTetMeshCountCells = lpProgramTet->addKernel("TetMeshCells");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelTetMeshCountCells);

		m_lpKernelTetMeshVertices = lpProgramTet->addKernel("TetMeshVertices");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelTetMeshVertices);

		m_lpKernelTetMeshElements = lpProgramTet->addKernel("TetMeshElements");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelTetMeshElements);
		*/

		//Init some vars
		m_ctTetMeshVertices = m_ctTetMeshElements = 0;
        
        m_inoutMemRestPos = NULL;

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
			LogErrorArg1("Failed to create VertexCountTable! Ocl Error: %s", PS::CL::oclErrorString(errNum));
			return ERR_GPUPOLY_VERTEXTABLE_NOT_READ;
		}

		//Triangle Count Table
		m_inMemTriangleTable = clCreateImage2D(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
											   &imageFormat, 16, 256, 0, (void*) g_triTableCompact, &errNum);
		if(errNum != CL_SUCCESS)
		{
			LogErrorArg1("Failed to create TriableTable! Ocl Error: %s", PS::CL::oclErrorString(errNum));
			return ERR_GPUPOLY_TRITABLE_NOT_READ;
		}
#endif

		//Fill CellParam
		//const int corner1[12]    = {LBN,LTN,LBN,LBF,RBN,RTN,RBN,RBF,LBN,LBF,LTN,LTF};
		//const int corner2[12]    = {LBF,LTF,LTN,LTF,RBF,RTF,RTN,RTF,RBN,RBF,RTN,RTF};
		//const int edgeaxis[12]   = {AAZ,AAZ,AAY,AAY,AAZ,AAZ,AAY,AAY,AAX,AAX,AAX,AAX};
		for(int i=0; i<12; i++)
		{
			m_cellParam.corner1[i] = (U8)corner1[i];
			m_cellParam.corner2[i] = (U8)corner2[i];
			m_cellParam.edgeaxis[i] = (U8)edgeaxis[i];
		}

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
		vec3f lower = vec3f(m_blob.arrHeader[0], m_blob.arrHeader[1], m_blob.arrHeader[2]);
		vec3f upper = vec3f(m_blob.arrHeader[4], m_blob.arrHeader[5], m_blob.arrHeader[6]);
		vec3f temp = vec3f::mul(1.0f/ cellsize, vec3f::sub(upper, lower));
		
		//Cell Params
		m_cellParam.ctNeededCells[0] = (U32)ceil(temp.x) + 1;
		m_cellParam.ctNeededCells[1] = (U32)ceil(temp.y) + 1;
		m_cellParam.ctNeededCells[2] = (U32)ceil(temp.z) + 1;
		m_cellParam.ctTotalCells = m_cellParam.ctNeededCells[0] * m_cellParam.ctNeededCells[1] * m_cellParam.ctNeededCells[2];
		m_cellParam.cellsize = cellsize;

		//Check Matrices
		for(U32 i = 0; i < m_blob.countMtxNodes(); i++)
		{
			printf("PRIM MTX: %d\n", i);
			for(U32 j=0;j < PRIM_MATRIX_STRIDE; j++)
			{
				printf("%.2f, ", m_blob.arrMtxNodes[i * PRIM_MATRIX_STRIDE + j]);
				if((j+1) % 4 == 0)
					printf("\n");
			}
			//printf("\n");
		}

		//CellConfig and VertexCount
		cl_mem inoutMemCellConfig = m_lpGPU->createMemBuffer(sizeof(U8) * m_cellParam.ctTotalCells, PS::CL::memReadWrite);
		cl_mem inoutMemVertexCount = m_lpGPU->createMemBuffer(sizeof(U8) * m_cellParam.ctTotalCells, PS::CL::memReadWrite);
			
		//CellParams
		cl_mem inMemCellParams = m_lpGPU->createMemBuffer(sizeof(CellParam), PS::CL::memReadOnly);
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
			LogErrorArg1("Error: Failed to execute ComputeConfig kernel! (%s)", oclErrorString(err));
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
		//U32 ctTets = ctCrossedOrInside * 6;
		//(x,y,z) * number of cubes * 8 cube corners
		cl_mem outMemTetMeshVertices = m_lpGPU->createMemBuffer(sizeof(float) * 3 * ctCrossedOrInside * 8, PS::CL::memWriteOnly);

		//(a,b,c,d) * number of cubes * 6 Tetrahedra per cube
		cl_mem outMemTetMeshIndices = m_lpGPU->createMemBuffer(sizeof(U32) * 4 * ctCrossedOrInside * 6, PS::CL::memWriteOnly);


		//Vertex buffer offset
		cl_mem inMemVertexBufferOffset;
		inMemVertexBufferOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, PS::CL::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(inMemVertexBufferOffset, sizeof(U32) * m_cellParam.ctTotalCells, arrVertexBufferOffset))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Tetmesh buffer offset
		cl_mem inMemTetMeshBufferOffset;
		inMemTetMeshBufferOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, PS::CL::memReadOnly);
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
		cl_mem outMemMeshVertex = m_lpGPU->createMemBufferFromGL(m_vboVertex, PS::CL::memWriteOnly);

		//Color
		glGenBuffers(1, &m_vboColor);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshColor = m_lpGPU->createMemBufferFromGL(m_vboColor, PS::CL::memWriteOnly);

		//Normal
		glGenBuffers(1, &m_vboNormal);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboNormal);
		glBufferData(GL_ARRAY_BUFFER, szNormalBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshNormal = m_lpGPU->createMemBufferFromGL(m_vboNormal, PS::CL::memWriteOnly);

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
			LogErrorArg1("Error: Failed to execute ComputeMesh kernel! (%s)", oclErrorString(err));
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
	int GPUPoly::runPolygonizer(float cellsize)
	{
		if(DefinitelyLessThan(cellsize, 0.01, EPSILON))
			return -1;

		AnsiStr strArg = printToAStr("Total poly with cellsize: %.3f", cellsize);
		ProfileStartArg(strArg.cptr());

		//Clear previous mesh
		GLMeshBuffer::cleanup();

		//1. Compute All Fields
		computeAllFields(cellsize);

		//2. Compute Edge Table
		computeEdgeTable();
		{
			ProfileAutoArg("KERNEL: Pre-Fix SumScan on EdgeBuffer");
			//Read-back EdgeTable Count
			U32* lpEdgeTableOffsets = new U32[m_gridParam.ctTotalPoints];
			m_lpGPU->enqueueReadBuffer(m_inoutMemHighEdgesCount, sizeof(U32) * m_gridParam.ctTotalPoints, lpEdgeTableOffsets);
			m_lpGPU->finishAllCommands();
			//PrintArray(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);

			//3. SumScan to compute crossed edges
			m_ctVertices = m_lpOclSumScan->compute(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);
			//PrintArray(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);

			//Write Edge Offsets to device memory
			m_inMemHighEdgesOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_gridParam.ctTotalPoints, PS::CL::memReadOnly);
			m_lpGPU->enqueueWriteBuffer(m_inMemHighEdgesOffset, sizeof(U32) * m_gridParam.ctTotalPoints, lpEdgeTableOffsets);
			SAFE_DELETE(lpEdgeTableOffsets);
		}

		//If the search for iso-surface didn't return any points!
		if(m_ctVertices == 0) {
			LogError("No edges intersected with the surface!");
			return -1;
		}

		//4. Compute VertexAttribs
		computeVertexAttribs(m_ctVertices);

		//5. Compute Cell Configs
		computeCellConfigs();
		{
			ProfileAutoArg("KERNEL: Pre-Fix SumScan on FaceBuffer");
			//Read-back Elements Count
			U32* lpCellElementsOffset = new U32[m_cellParam.ctTotalCells];
			m_lpGPU->enqueueReadBuffer(m_inoutMemCellElementsCount, sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);
			m_lpGPU->finishAllCommands();
			//PrintArray(lpCellElementsOffset, m_cellParam.ctTotalCells);

			//6. SumScan to read all elements
			m_ctFaceElements = m_lpOclSumScan->compute(lpCellElementsOffset, m_cellParam.ctTotalCells);
			//PrintArray(lpCellElementsOffset, m_cellParam.ctTotalCells);

			//Write Element Offsets
			m_inMemCellElementsOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, PS::CL::memReadOnly);
			m_lpGPU->enqueueWriteBuffer(m_inMemCellElementsOffset, sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);
			SAFE_DELETE(lpCellElementsOffset);
		}

		//7. Compute Elements
		computeElements(m_ctFaceElements);


		//Draw Mesh
		m_isValidIndex = true;
		m_faceMode = ftTriangles;

		//Release all temp vars
		clReleaseMemObject(m_inMemCellParam);
		clReleaseMemObject(m_inMemGridParam);

		//clReleaseMemObject(m_inoutMemAllFields);
		clReleaseMemObject(m_inoutMemHighEdgesCount);
		clReleaseMemObject(m_inoutMemHighEdgesFlags);
		clReleaseMemObject(m_inMemHighEdgesOffset);

		clReleaseMemObject(m_inoutMemCellConfig);
		clReleaseMemObject(m_inoutMemCellElementsCount);
		clReleaseMemObject(m_inMemCellElementsOffset);

		ProfileEnd();

		return 1;
	}

	/*!
	 * Run GPU Tetrahedralizer
	 */
	int GPUPoly::runTetrahedralizer() {

		//8. Find all cells and vertices includes in the tetmesh
		computeTetMeshCellsInsideOrCrossed();

		//SumScan to Compute Total Tetrahedra elements produced and elements offset
		{
			//Read-back Elements Count
			U32* lpCellElementsOffset = new U32[m_cellParam.ctTotalCells];
			m_lpGPU->enqueueReadBuffer(m_inoutMemCellElementsCount,
					sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);

			//9. SumScan to read all elements, 6 Tetrahedra per each cell
			m_ctTetMeshElements = m_lpOclSumScan->compute(lpCellElementsOffset,
					m_cellParam.ctTotalCells);
			m_ctTetMeshElements *= 6;

			//Write Element Offsets
			//m_inMemCellElementsOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, ComputeDevice::memReadOnly);
			m_lpGPU->enqueueWriteBuffer(m_inMemCellElementsOffset,
					sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);
			SAFE_DELETE(lpCellElementsOffset);
		}

		//SumScan to Compute Total Vertices count and vertex list offsets
		{
			//Read-back EdgeTable Count
			U32* lpVerticesIncluded = new U32[m_gridParam.ctTotalPoints];
			m_lpGPU->enqueueReadBuffer(m_inoutMemHighEdgesCount,
					sizeof(U32) * m_gridParam.ctTotalPoints, lpVerticesIncluded);

			//10. SumScan to compute included vertices
			m_ctTetMeshVertices = m_lpOclSumScan->compute(lpVerticesIncluded,
					m_gridParam.ctTotalPoints);

			//Write Edge Offsets to device memory
			//m_inMemHighEdgesOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_gridParam.ctTotalPoints, ComputeDevice::memReadOnly);
			m_lpGPU->enqueueWriteBuffer(m_inMemHighEdgesOffset,
					sizeof(U32) * m_gridParam.ctTotalPoints, lpVerticesIncluded);
			SAFE_DELETE(lpVerticesIncluded);
		}

		//11. Compute Tetmesh vertices
		computeTetMeshVertices(m_ctTetMeshVertices);

		//12.Compute Tetrahedra for all inside or crossed cells (6 tets per each cell)
		computeTetMeshElements(m_ctTetMeshElements);

		//13. Store tet mesh as vega format
		string strVegaFP = m_strModelFilePath + string(".veg");
		storeTetMeshInVegaFormat(strVegaFP.c_str());

		//Release Temp vars for tetmesh
		clReleaseMemObject(m_outMemTetMeshVertices);
		clReleaseMemObject(m_outMemTetMeshElements);

		return 1;
	}

	bool GPUPoly::readbackMeshV3T3(U32& ctVertices, vector<float>& vertices,
								  U32& ctTriangles, vector<U32>& elements) const {
		if(!m_isValidIndex)
			return false;

		ctVertices = m_ctVertices;
		ctTriangles = m_ctFaceElements / 3;

		//Vertices
		vector<float> homogenousVertices;
		homogenousVertices.resize(m_ctVertices * 4);
		vertices.resize(m_ctVertices * 3);
		elements.resize(m_ctFaceElements);

		//Vertices
		U32 szVertexBuffer = sizeof(float) * 4 * m_ctVertices;
		U32 szIndexBuffer = sizeof(U32) * m_ctFaceElements;


		//Vertices
		cl_mem outMemMeshVertices = m_lpGPU->createMemBufferFromGL(m_vboVertex, PS::CL::memReadOnly);
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshVertices);
		m_lpGPU->enqueueReadBuffer(outMemMeshVertices, szVertexBuffer, &homogenousVertices[0]);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshVertices);
		m_lpGPU->finishAllCommands();

		//Copy from homogenous coordinate system
		for(U32 i=0; i<m_ctVertices; i++) {
			vertices[i*3] = homogenousVertices[i*4];
			vertices[i*3 + 1] = homogenousVertices[i*4 + 1];
			vertices[i*3 + 2] = homogenousVertices[i*4 + 2];
		}


		//Elements
		cl_mem outMemMeshElements = m_lpGPU->createMemBufferFromGL(m_iboFaces, PS::CL::memReadOnly);
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshElements);
		m_lpGPU->enqueueReadBuffer(outMemMeshElements, szIndexBuffer, &elements[0]);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshElements);
		m_lpGPU->finishAllCommands();
		//PrintArray(&elements[0], 512);

		clReleaseMemObject(outMemMeshVertices);
		clReleaseMemObject(outMemMeshElements);

		return true;
	}

	bool GPUPoly::readBackNormals(U32& ctVertices, vector<float>& verticesXYZ, vector<float>& normals) const {
		if(!m_isValidIndex)
			return false;

		ctVertices = m_ctVertices;

		//Allocate buffers
		vector<float> homogenousVertices;
		homogenousVertices.resize(m_ctVertices * 4);
		verticesXYZ.resize(m_ctVertices * 3);
		normals.resize(m_ctVertices * 3);

		//Sizes
		U32 szVertexBuffer = sizeof(float) * 4 * m_ctVertices;
		U32 szNormalBuffer = sizeof(float) * 3 * m_ctVertices;

		//Vertices
		cl_mem outMemMeshVertices = m_lpGPU->createMemBufferFromGL(m_vboVertex, PS::CL::memReadOnly);
		cl_mem outMemMeshNormals = m_lpGPU->createMemBufferFromGL(m_vboNormal, PS::CL::memReadOnly);

		//Vertices
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshVertices);
		m_lpGPU->enqueueReadBuffer(outMemMeshVertices, szVertexBuffer, &homogenousVertices[0]);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshVertices);

		//Normals
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshNormals);
		m_lpGPU->enqueueReadBuffer(outMemMeshNormals, szNormalBuffer, &normals[0]);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshNormals);

		m_lpGPU->finishAllCommands();

		//Copy from homogenous coordinate system
		for(U32 i=0; i<m_ctVertices; i++) {
			verticesXYZ[i*3] = homogenousVertices[i*4];
			verticesXYZ[i*3 + 1] = homogenousVertices[i*4 + 1];
			verticesXYZ[i*3 + 2] = homogenousVertices[i*4 + 2];
		}


		//Release memory buffers
		clReleaseMemObject(outMemMeshVertices);
		clReleaseMemObject(outMemMeshNormals);

		return true;
	}

	bool GPUPoly::readBackVoxelGridSamples(vec4u& dim, vector<float>& arrXYZF) const {
		if(!m_isValidIndex)
			return false;

//		int idX = get_global_id(0);
//		int idY = get_global_id(1);
//		int idZ = get_global_id(2);
//		if((idX >= inGridParam->ctGridPoints[0])||(idY >= inGridParam->ctGridPoints[1])||(idZ >= inGridParam->ctGridPoints[2]))
//			return;
//		U32 idxVertex = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + idX;
//		if(idxVertex >= inGridParam->ctTotalPoints)
//			return;
		//Copy Grid Dimension
		dim.x = m_gridParam.ctGridPoints[0];
		dim.y = m_gridParam.ctGridPoints[1];
		dim.z = m_gridParam.ctGridPoints[2];
		dim.w = m_gridParam.ctTotalPoints;

		arrXYZF.resize(m_gridParam.ctTotalPoints * 4);
		U32 szBuffer = m_gridParam.ctTotalPoints * 4 * sizeof(float);
		m_lpGPU->enqueueReadBuffer(m_inoutMemAllFields, szBuffer, &arrXYZF[0]);
		m_lpGPU->finishAllCommands();

		return true;
	}


	int GPUPoly::computeFieldArray(U32 ctVertices, U32 step, vector<float>& vertices) {
		if(step != 4)
			return ERR_INVALID_INPUT_PARAM;
		//Computes field value for an array of vertices
//		__kernel void ComputeFieldArray(U32 ctVertices,
//									   __global float4* arrInHeader4,
//									   __global float4* arrInOps4,
//									   __global float4* arrInPrims4,
//									   __global float4* arrInMtxNodes4,
//									   __global float4* arrInOutVertexFields)
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		arrGlobalIndex[0] = ctVertices;
		arrGlobalIndex[1] = 0;
		arrGlobalIndex[2] = 0;

		ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelComputeFieldArray->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);

		//Create Memory for all field points
		U32 szVertexBuffer = sizeof(float) * step * ctVertices;
		cl_mem inoutMemFieldArray = m_lpGPU->createMemBuffer(szVertexBuffer, PS::CL::memReadWrite);
		if(!m_lpGPU->enqueueWriteBuffer(inoutMemFieldArray, szVertexBuffer, &vertices[0]))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		m_lpKernelComputeFieldArray->setArg(0, sizeof(U32), (void *)&ctVertices);
		m_lpKernelComputeFieldArray->setArg(1, sizeof(cl_mem), &m_inMemHeader);
		m_lpKernelComputeFieldArray->setArg(2, sizeof(cl_mem), &m_inMemOps);
		m_lpKernelComputeFieldArray->setArg(3, sizeof(cl_mem), &m_inMemPrims);
		m_lpKernelComputeFieldArray->setArg(4, sizeof(cl_mem), &m_inMemMtx);
		m_lpKernelComputeFieldArray->setArg(5, sizeof(cl_mem), &inoutMemFieldArray);

		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeFieldArray, 1, arrGlobalIndex, arrLocalIndex);
		m_lpGPU->enqueueReadBuffer(inoutMemFieldArray, szVertexBuffer, &vertices[0]);

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();



		return 1;
	}
    
    
    int GPUPoly::computeFieldImage(const vec3f& lo, const vec3f& hi, GLTexture* lpOutTex) {
        if(lpOutTex == NULL)
            return -1;
        /*
        __kernel void ComputeFieldImage(float4 lo, float4 hi,
                                        int dimx, int dimy,
                                        __global float4* arrInHeader4,
                                        __global float4* arrInOps4,
                                        __global float4* arrInPrims4,
                                        __global float4* arrInMtxNodes4,							
                                        __write_only image2d_t fields) {
         */
        
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		arrGlobalIndex[0] = lpOutTex->dim().x;
		arrGlobalIndex[1] = lpOutTex->dim().y;
		arrGlobalIndex[2] = 0;
        
        
		ComputeKernel::ComputeLocalIndexSpace(2, m_lpKernelComputeFieldImage->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(2, arrLocalIndex, arrGlobalIndex);
        
		//Create Memory for fields image
        cl_mem outMemFields = m_lpGPU->createImageFromGL(lpOutTex->handle(), PS::CL::memWriteOnly);
                
        int dimx = lpOutTex->dim().x;
        int dimy = lpOutTex->dim().y;
        
        //Set Args
        m_lpKernelComputeFieldImage->setArg(0, sizeof(float) * 4, (void*)lo.cptr());
        m_lpKernelComputeFieldImage->setArg(1, sizeof(float) * 4, (void*)hi.cptr());
        m_lpKernelComputeFieldImage->setArg(2, sizeof(int), (void*)&dimx);
        m_lpKernelComputeFieldImage->setArg(3, sizeof(int), (void*)&dimy);

		m_lpKernelComputeFieldImage->setArg(4, sizeof(cl_mem), &m_inMemHeader);
		m_lpKernelComputeFieldImage->setArg(5, sizeof(cl_mem), &m_inMemOps);
		m_lpKernelComputeFieldImage->setArg(6, sizeof(cl_mem), &m_inMemPrims);
		m_lpKernelComputeFieldImage->setArg(7, sizeof(cl_mem), &m_inMemMtx);
		m_lpKernelComputeFieldImage->setArg(8, sizeof(cl_mem), &outMemFields);
        
		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
        m_lpGPU->enqueueAcquireGLObject(1, &outMemFields);
        
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeFieldImage, 1, arrGlobalIndex, arrLocalIndex);

        m_lpGPU->enqueueReleaseGLObject(1, &outMemFields);
        
		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();

        return 1;
    }
    

	int GPUPoly::computeOffSurfacePointsAndFields(U32 interval, float len, U32& ctOutVertices, vector<float>& outOffSurfacePoints) {
//		__kernel void ComputeOffSurfacePointsAndFields(float len,
//													  U32 ctVertices,
//													  __global float4* arrInMeshVertex,
//													  __global float3* arrInMeshNormal,
//													  __global float4* arrInHeader4,
//													  __global float4* arrInOps4,
//													  __global float4* arrInPrims4,
//													  __global float4* arrInMtxNodes4,
//													  __global float4* arrOutOffSurfaceXYZF)
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		arrGlobalIndex[0] = m_ctVertices;
		arrGlobalIndex[1] = 0;
		arrGlobalIndex[2] = 0;


		ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelComputeOffSurfacePointsAndFields->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);

		//Create Memory for all field points
		ctOutVertices = m_ctVertices*2;
		outOffSurfacePoints.resize(m_stepVertex * ctOutVertices);
		U32 szOffSurfaceBuffer = sizeof(float) * m_stepVertex * ctOutVertices;
		cl_mem inoutMemOffSurface = m_lpGPU->createMemBuffer(szOffSurfaceBuffer, PS::CL::memReadWrite);


		//Vertices
		cl_mem outMemMeshVertices = m_lpGPU->createMemBufferFromGL(m_vboVertex, PS::CL::memReadOnly);
		cl_mem outMemMeshNormals = m_lpGPU->createMemBufferFromGL(m_vboNormal, PS::CL::memReadOnly);

		//Vertices
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshVertices);
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshNormals);


		m_lpKernelComputeOffSurfacePointsAndFields->setArg(0, sizeof(float), (void *)&len);
		m_lpKernelComputeOffSurfacePointsAndFields->setArg(1, sizeof(U32), (void *)&m_ctVertices);
		m_lpKernelComputeOffSurfacePointsAndFields->setArg(2, sizeof(cl_mem), &outMemMeshVertices);
		m_lpKernelComputeOffSurfacePointsAndFields->setArg(3, sizeof(cl_mem), &outMemMeshNormals);

		m_lpKernelComputeOffSurfacePointsAndFields->setArg(4, sizeof(cl_mem), &m_inMemHeader);
		m_lpKernelComputeOffSurfacePointsAndFields->setArg(5, sizeof(cl_mem), &m_inMemOps);
		m_lpKernelComputeOffSurfacePointsAndFields->setArg(6, sizeof(cl_mem), &m_inMemPrims);
		m_lpKernelComputeOffSurfacePointsAndFields->setArg(7, sizeof(cl_mem), &m_inMemMtx);
		m_lpKernelComputeOffSurfacePointsAndFields->setArg(8, sizeof(cl_mem), &inoutMemOffSurface);

		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeOffSurfacePointsAndFields, 1, arrGlobalIndex, arrLocalIndex);

		//Release Mesh Buffer
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshVertices);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshNormals);

		//ReadBack Offsurface Points
		m_lpGPU->enqueueReadBuffer(inoutMemOffSurface, szOffSurfaceBuffer, &outOffSurfacePoints[0]);

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();

		return 1;
	}


	int GPUPoly::computeElements(U32 ctElements)
	{
		ProfileAutoArg("KERNEL: ComputeFaces");
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
		cl_mem outMemMeshElements = m_lpGPU->createMemBufferFromGL(m_iboFaces, PS::CL::memWriteOnly);

		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshElements);
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

		//Q commands
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeElements, 3, arrGlobalIndex, arrLocalIndex);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshElements);

		//Execute All
		m_lpGPU->finishAllCommands();

		//Check Indices
		/*
		U32* arrIndices = new U32[m_ctFaceElements];
		m_lpGPU->enqueueReadBuffer(outMemMeshElements, sizeof(U32) * m_ctFaceElements, arrIndices);
		SAFE_DELETE(arrIndices);
		*/
		/*
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
		SAFE_DELETE(arrIndices);
		*/

		return 1;
	}

	int GPUPoly::computeCellConfigs()
	{
		ProfileAutoArg("KERNEL: ComputeCellConfigs");
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_cellParam.ctNeededCells[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelComputeCellConfigs->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		m_inoutMemCellElementsCount = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, PS::CL::memReadWrite);
		m_inoutMemCellConfig = m_lpGPU->createMemBuffer(sizeof(U8) * m_cellParam.ctTotalCells, PS::CL::memReadWrite);

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
		ProfileAutoArg("KERNEL: Vertex Attribs");

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

		//Create memory buffer for vertex rest positions
		m_inoutMemRestPos = m_lpGPU->createMemBuffer(szVertexBuffer, PS::CL::memReadWrite);

		//Vertex
		glGenBuffers(1, &m_vboVertex);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshVertex = m_lpGPU->createMemBufferFromGL(m_vboVertex, PS::CL::memWriteOnly);

		//Color
		glGenBuffers(1, &m_vboColor);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshColor = m_lpGPU->createMemBufferFromGL(m_vboColor, PS::CL::memWriteOnly);

		//Normal
		glGenBuffers(1, &m_vboNormal);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboNormal);
		glBufferData(GL_ARRAY_BUFFER, szNormalBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshNormal = m_lpGPU->createMemBufferFromGL(m_vboNormal, PS::CL::memWriteOnly);

		//Acquire Mesh for Writing
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshVertex);
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshColor);
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshNormal);

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

		//Copy Mesh Vertices to Rest Pos Mesh
		m_lpGPU->enqueueCopyBuffer(outMemMeshVertex, m_inoutMemRestPos, 0, 0, (U32)szVertexBuffer);

		//Release All objects
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshVertex);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshColor);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshNormal);

		//Execute all commands in command q
		m_lpGPU->finishAllCommands();

		//Vertices
		m_isValidVertex = m_isValidColor = m_isValidNormal = true;
		m_ctVertices = ctVertices;


		//Read Vertices
		/*
		float* arrVertices = new float[m_ctVertices*4];
		m_lpGPU->enqueueReadBuffer(outMemMeshVertex, sizeof(float) * 4 * m_ctVertices, arrVertices);
			//PrintArrayF(arrVertices, 4 * m_ctVertices);
		SAFE_DELETE(arrVertices);
		*/

		//Release OCL Mem
		clReleaseMemObject(outMemMeshVertex);
		clReleaseMemObject(outMemMeshColor);
		clReleaseMemObject(outMemMeshNormal);

		return 1;
	}

	int GPUPoly::computeEdgeTable()
	{
		ProfileAutoArg("KERNEL: ComputeEdgeTable");
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_gridParam.ctGridPoints[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelComputeEdgeTable->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		//Buffers for edge table
		m_inoutMemHighEdgesCount = m_lpGPU->createMemBuffer(sizeof(U32) * m_gridParam.ctTotalPoints, PS::CL::memReadWrite);
		m_inoutMemHighEdgesFlags = m_lpGPU->createMemBuffer(sizeof(U8) * m_gridParam.ctTotalPoints, PS::CL::memReadWrite);

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

		//ComputeEdgeTable
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeEdgeTable, 3, arrGlobalIndex, arrLocalIndex);

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();


		return 1;
	}

	int GPUPoly::computeAllFields(float cellsize)
	{
		ProfileAutoArg("KERNEL: ComputeAllFields");

		//BBOX
		vec3f lower = vec3f(&m_blob.arrHeader[0]);
		vec3f upper = vec3f(&m_blob.arrHeader[4]);
		vec3f temp = vec3f::mul(1.0f/ cellsize, vec3f::sub(upper, lower));

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
		m_inMemCellParam = m_lpGPU->createMemBuffer(sizeof(CellParam), PS::CL::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemCellParam, sizeof(CellParam), &m_cellParam))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//GridParam
		m_inMemGridParam = m_lpGPU->createMemBuffer(sizeof(GridParam), PS::CL::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemGridParam, sizeof(GridParam), &m_gridParam))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Create Memory for all field points
		m_inoutMemAllFields = m_lpGPU->createMemBuffer(sizeof(float) * 4 * m_gridParam.ctTotalPoints, PS::CL::memReadWrite);

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
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeAllFields, 3, arrGlobalIndex, arrLocalIndex);

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();

		return 1;
	}


	int GPUPoly::computeTetMeshCellsInsideOrCrossed() {
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_cellParam.ctNeededCells[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelTetMeshCountCells->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		//Zero out the array before writing
		U32* lpIncludedVertices = new U32[m_gridParam.ctTotalPoints];
		memset(lpIncludedVertices, 0, m_gridParam.ctTotalPoints * sizeof(U32));
		m_lpGPU->enqueueWriteBuffer(m_inoutMemHighEdgesCount, m_gridParam.ctTotalPoints * sizeof(U32), lpIncludedVertices);
		SAFE_DELETE_ARRAY(lpIncludedVertices);


		/*
			__kernel void TetMeshCells(__global U8* arrInCellConfig,
										__constant struct CellParam* inCellParam,
										__constant struct GridParam* inGridParam,
										__global U32* arrOutIncludedCells,
										__global U32* arrOutIncludedVertices) {
		*/
		m_lpKernelTetMeshCountCells->setArg(0, sizeof(cl_mem), &m_inoutMemCellConfig);
		m_lpKernelTetMeshCountCells->setArg(1, sizeof(cl_mem), &m_inMemCellParam);
		m_lpKernelTetMeshCountCells->setArg(2, sizeof(cl_mem), &m_inMemGridParam);
		m_lpKernelTetMeshCountCells->setArg(3, sizeof(cl_mem), &m_inoutMemCellElementsCount);
		m_lpKernelTetMeshCountCells->setArg(4, sizeof(cl_mem), &m_inoutMemHighEdgesCount);

		m_lpGPU->enqueueNDRangeKernel(m_lpKernelTetMeshCountCells, 3, arrGlobalIndex, arrLocalIndex);
		m_lpGPU->finishAllCommands();

		return 1;
	}

	int GPUPoly::computeTetMeshVertices(U32 ctVertices) {
		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_gridParam.ctGridPoints[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelTetMeshVertices->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		//Vertex Attrib Buffers
		GLsizeiptr szVertexBuffer = ctVertices * 3 * sizeof(float);
		m_outMemTetMeshVertices = m_lpGPU->createMemBuffer(szVertexBuffer, PS::CL::memReadWrite);
		/*
			__kernel void TetMeshVertices(__global float4* arrInFields,
										  __global U32* arrInVerticesIncluded,
										  __global U32* arrInVerticesOffset,
										  __constant struct GridParam* inGridParam,
										  __global float* arrOutTetMeshVertices)
		*/
		m_lpKernelTetMeshVertices->setArg(0, sizeof(cl_mem), &m_inoutMemAllFields);
		m_lpKernelTetMeshVertices->setArg(1, sizeof(cl_mem), &m_inoutMemHighEdgesCount);
		m_lpKernelTetMeshVertices->setArg(2, sizeof(cl_mem), &m_inMemHighEdgesOffset);
		m_lpKernelTetMeshVertices->setArg(3, sizeof(cl_mem), &m_inMemGridParam);
		m_lpKernelTetMeshVertices->setArg(4, sizeof(cl_mem), &m_outMemTetMeshVertices);

		//Enqueue Kernel
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelTetMeshVertices, 3, arrGlobalIndex, arrLocalIndex);
		m_lpGPU->finishAllCommands();


		//Read Vertices
		/*
		float* arrVertices = new float[m_ctVertices*4];
		m_lpGPU->enqueueReadBuffer(outMemMeshVertex, sizeof(float) * 4 * m_ctVertices, arrVertices);
			//PrintArrayF(arrVertices, 4 * m_ctVertices);
		SAFE_DELETE(arrVertices);
		*/

		return 1;
	}


	int GPUPoly::computeTetMeshElements(U32 ctElements) {

		size_t arrLocalIndex[3];
		size_t arrGlobalIndex[3];
		for(int i=0; i<3; i++)
			arrGlobalIndex[i] = m_cellParam.ctNeededCells[i];

		ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelTetMeshElements->getKernelWorkGroupSize(), arrLocalIndex);
		ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

		//Each Tetrahedra has 4 indices
		GLsizeiptr szElementsBuffer = ctElements * 4 * sizeof(U32);
		m_outMemTetMeshElements = m_lpGPU->createMemBuffer(szElementsBuffer, PS::CL::memReadWrite);


		/*
			__kernel void TetMeshElements(__global U32* arrInVerticesOffset,
										  __global U32* arrInCellTetOffset,
										  __global U32* arrInCellsIncluded,
										  __constant struct GridParam* inGridParam,
										  __constant struct CellParam* inCellParam,
										  __global U32* arrOutTetMeshIndices)
		*/
		m_lpKernelTetMeshElements->setArg(0, sizeof(cl_mem), &m_inMemHighEdgesOffset);
		m_lpKernelTetMeshElements->setArg(1, sizeof(cl_mem), &m_inMemCellElementsOffset);
		m_lpKernelTetMeshElements->setArg(2, sizeof(cl_mem), &m_inoutMemCellElementsCount);
		m_lpKernelTetMeshElements->setArg(3, sizeof(cl_mem), &m_inMemGridParam);
		m_lpKernelTetMeshElements->setArg(4, sizeof(cl_mem), &m_inMemCellParam);
		m_lpKernelTetMeshElements->setArg(5, sizeof(cl_mem), &m_outMemTetMeshElements);

		m_lpGPU->enqueueNDRangeKernel(m_lpKernelTetMeshElements, 3, arrGlobalIndex, arrLocalIndex);
		m_lpGPU->finishAllCommands();

		return 1;
	}


	bool GPUPoly::applyFemDisplacements(U32 dof, double* displacements) {

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
		cl_mem inMemDisplacements = m_lpGPU->createMemBuffer(szVertexBuffer, PS::CL::memReadOnly);
		m_lpGPU->enqueueWriteBuffer(inMemDisplacements, szVertexBuffer, &homogenousDisplacements[0]);

		//Vertices
		cl_mem outMemMeshVertices = m_lpGPU->createMemBufferFromGL(m_vboVertex, PS::CL::memWriteOnly);
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

	vec3i GPUPoly::voxelGridDim() const {
		return vec3i((int)m_gridParam.ctGridPoints[0], (int)m_gridParam.ctGridPoints[1], (int)m_gridParam.ctGridPoints[2]);
	}

	bool GPUPoly::setBlob(const LinearBlobTree& blob) {
		if(blob.countPrimitives() == 0) {
			LogError("There is no primitives in the blobtree!");
			return false;
		}

		if(blob.countMtxNodes() == 0) {
			LogError("There is no mtx nodes in the blobtree!");
			return false;
		}

		//Clear previous buffers
		clearBlobBuffer();

		//Copy Blob
		m_blob.copyFrom(blob);
		m_aabb = blob.aabb();
        m_bModelLoaded = true;


		//Model memories
		U32 ctOpsToCopy = MATHMAX(m_blob.countOperators(), 1);
		U32 ctPrimsToCopy = MATHMAX(m_blob.countPrimitives(), 1);

		m_inMemHeader = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_HEADER, PS::CL::memReadOnly);
		m_inMemOps    = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, PS::CL::memReadOnly);
		m_inMemPrims  = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, PS::CL::memReadOnly);
		m_inMemMtx	  = m_lpGPU->createMemBuffer(sizeof(float) * PRIM_MATRIX_STRIDE * m_blob.countMtxNodes(), PS::CL::memReadOnly);

		//Header
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemHeader, sizeof(float) * DATASIZE_HEADER, m_blob.arrHeader.cptr()))
			return false;

		//Ops
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemOps, sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, m_blob.arrOps.cptr()))
			return false;

		//Prims
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemPrims, sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, m_blob.arrPrims.cptr()))
			return false;

		//Matrix
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemMtx, sizeof(float) * PRIM_MATRIX_STRIDE * m_blob.countMtxNodes(), m_blob.arrMtxNodes.cptr()))
			return false;

		return true;
	}

	//Export VEGA
	bool GPUPoly::storeTetMeshInVegaFormat(const char* chrFilePath)
	{
		if((m_ctTetMeshVertices == 0)||(m_ctTetMeshElements == 0))
			return false;

		float* arrTetMeshVertices = new float[3 * m_ctTetMeshVertices];
		U32* arrTetMeshIndices = new U32[4 * m_ctTetMeshElements];

		m_lpGPU->enqueueReadBuffer(m_outMemTetMeshVertices, sizeof(float) * 3 * m_ctTetMeshVertices, arrTetMeshVertices);
		m_lpGPU->enqueueReadBuffer(m_outMemTetMeshElements, sizeof(U32) * 4 * m_ctTetMeshElements, arrTetMeshIndices);


		ofstream ofs;
		ofs.open(chrFilePath, ios::out);
		ofs << "# Vega Mesh File, Generated by FemBrain." << endl;
		ofs << "# " << m_ctTetMeshVertices << " vertices, " << m_ctTetMeshElements << " elements" << endl;
		ofs << endl;
		ofs << "*VERTICES" << endl;
		ofs << m_ctTetMeshVertices << " 3 0 0" << endl;
		for(U32 i=0; i < m_ctTetMeshVertices; i++)
		{
			ofs << i+1 << " " << arrTetMeshVertices[i*3 + 0] << " " << arrTetMeshVertices[i*3 + 1] << " " << arrTetMeshVertices[i*3 + 2] << endl;
		}

		ofs << endl;
		ofs << "*ELEMENTS" << endl;
		ofs << "TET" << endl;
		ofs << m_ctTetMeshElements << " 4 0"<< endl;
		for(U32 i=0; i < m_ctTetMeshElements; i++)
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

		return true;
	}

}
}



