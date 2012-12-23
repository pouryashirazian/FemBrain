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
#define ERR_NO_CROSSED_CELLS -6

#define MAX_VERTICES_COUNT_PER_CELL		15
#define MAX_TRIANGLES_COUNT_PER_CELL	5


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
		clReleaseMemObject(m_inMemTriangleTable);
		clReleaseMemObject(m_inMemVertexCountTable);
		SAFE_DELETE(m_lpGPU);
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
			m_param.corner1[i] = (U8)corner1[i];
			m_param.corner2[i] = (U8)corner2[i];
			m_param.edgeaxis[i] = (U8)edgeaxis[i];
		}		
		return true;
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
			return ERR_GPUPOLY_VERTEXTABLE_NOT_READ;

		//Triangle Count Table
		m_inMemTriangleTable = clCreateImage2D(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
											   &imageFormat, 16, 256, 0, (void*) g_triTableCompact, &errNum);
		if(errNum != CL_SUCCESS)
			return ERR_GPUPOLY_TRITABLE_NOT_READ;
#endif

	}

	//////////////////////////////////////////////////////////////////////
	int GPUPoly::run(float cellsize)
	{
		//Algorithm Overview
		//1.First we compute the fields in the grid of 8x8x8
		//2.Then all the cells within each grid are evaluated and the number of output vertices are determined.
		//3.Empty cells are discarded
		//4.Triangles are computed to the output mesh
		m_arrHeader[OFFSET_CELLSIZE] = cellsize;
		GLMeshBuffer::cleanup();


		//BBOX
		svec3f lower = svec3f(m_arrHeader[0], m_arrHeader[1], m_arrHeader[2]);
		svec3f upper = svec3f(m_arrHeader[4], m_arrHeader[5], m_arrHeader[6]);
		svec3f temp = vscale3f(1.0f/ cellsize, vsub3f(upper, lower));

		
		//Cell Params
		m_param.ctNeededCells[0] = (U32)ceil(temp.x) + 1;
		m_param.ctNeededCells[1] = (U32)ceil(temp.y) + 1;
		m_param.ctNeededCells[2] = (U32)ceil(temp.z) + 1;
		m_param.ctTotalCells = m_param.ctNeededCells[0] * m_param.ctNeededCells[1] * m_param.ctNeededCells[2];
		m_ctCells = m_param.ctTotalCells;

		//Output Buffers for the mesh from GL		
		/*
		glGenBuffers(1, &m_meshBuffer.iboFaces);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_meshBuffer.iboFaces);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_MPU_TRIANGLE_COUNT * 3 * sizeof(U32), 0, GL_DYNAMIC_DRAW);
		cl_mem inMemMeshFaces = clCreateFromGLBuffer(m_lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_meshBuffer.iboFaces, NULL);
		*/
		m_stepVertex = 4;
		m_stepColor = 4;
		U32 szVertexBuffer = m_ctCells * MAX_VERTICES_COUNT_PER_CELL * 4 * sizeof(float);

		//Vertex
		glGenBuffers(1, &m_vboVertex);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshVertex = clCreateFromGLBuffer(m_lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_vboVertex, NULL);

		//Color
		glGenBuffers(1, &m_vboColor);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshColor = clCreateFromGLBuffer(m_lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_vboColor, NULL);

		//Normal
		glGenBuffers(1, &m_vboNormal);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboNormal);
		glBufferData(GL_ARRAY_BUFFER, szVertexBuffer, 0, GL_DYNAMIC_DRAW);
		cl_mem outMemMeshNormal = clCreateFromGLBuffer(m_lpGPU->getContext(), CL_MEM_WRITE_ONLY, m_vboNormal, NULL);

		
		//Load Marching Cubes tables
		cl_image_format imageFormat;
		imageFormat.image_channel_order = CL_R;
		imageFormat.image_channel_data_type = CL_UNSIGNED_INT8;

		//Vertex Count Table
#ifdef CL_API_SUFFIX__VERSION_1_2
		cl_int errNum;
		cl_image_desc desc;
		desc.image_type = CL_MEM_OBJECT_IMAGE2D;
		desc.image_width = 256;
		desc.image_height = 1;
		desc.image_row_pitch = 0;
		desc.num_mip_levels = 0;
		desc.num_samples = 0;
		desc.buffer = NULL;


		cl_mem inMemVertexCountTable = clCreateImage(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
					  	  	  	  	  	     &imageFormat, &desc, (void*) g_numVerticesTableCompact, &errNum );
		if(errNum != CL_SUCCESS)
			return ERR_GPUPOLY_TRITABLE_NOT_READ;

		desc.image_width = 16;
		desc.image_height = 256;
		cl_mem inMemTriTable = clCreateImage(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
													&imageFormat, &desc, (void*) g_triTableCompact, &errNum);
#else
		cl_int errNum;		
		cl_mem inMemVertexCountTable = clCreateImage2D(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
													   &imageFormat, 256, 1, 0, (void*) g_numVerticesTableCompact, &errNum);
		if(errNum != CL_SUCCESS)
			return ERR_GPUPOLY_VERTEXTABLE_NOT_READ;

		//Triangle Count Table
		cl_mem inMemTriTable = clCreateImage2D(m_lpGPU->getContext(), CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
											   &imageFormat, 16, 256, 0, (void*) g_triTableCompact, &errNum);
		if(errNum != CL_SUCCESS)					
			return ERR_GPUPOLY_TRITABLE_NOT_READ;
#endif
		//Input Pos
		cl_mem outMemCellConfig;
		cl_mem outMemVertexCount;
		cl_mem inMemHeader;
		cl_mem inMemOps;
		cl_mem inMemPrims;		
		cl_mem inMemMtx;
		cl_mem inMemCellParams;
		U32 ctOpsToCopy = MATHMAX(m_ctOps, 1);
		U32 ctPrimsToCopy = MATHMAX(m_ctPrims, 1);
		
		// Create the device memory vectors
		outMemCellConfig = m_lpGPU->createMemBuffer(sizeof(U8) * m_ctCells, ComputeDevice::memWriteOnly);
		outMemVertexCount = m_lpGPU->createMemBuffer(sizeof(U8) * m_ctCells, ComputeDevice::memWriteOnly);
			
		inMemHeader = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_HEADER, ComputeDevice::memReadOnly);
		inMemOps  	= m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, ComputeDevice::memReadOnly);
		inMemPrims  = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, ComputeDevice::memReadOnly);		
		inMemMtx	= m_lpGPU->createMemBuffer(sizeof(float) * PRIM_MATRIX_STRIDE, ComputeDevice::memReadOnly);
		inMemCellParams = m_lpGPU->createMemBuffer(sizeof(CellParam), ComputeDevice::memReadOnly);

		// Transfer the input vector into device memory.
		//Header
		if(!m_lpGPU->enqueueWriteBuffer(inMemHeader, sizeof(float) * DATASIZE_HEADER, m_arrHeader))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Ops
		if(!m_lpGPU->enqueueWriteBuffer(inMemOps, sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, m_arrOps))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Prims
		if(!m_lpGPU->enqueueWriteBuffer(inMemPrims, sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, m_arrPrims))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Matrix
		if(!m_lpGPU->enqueueWriteBuffer(inMemMtx, sizeof(float) * PRIM_MATRIX_STRIDE * m_mtxNode.count, m_mtxNode.matrix))	
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Needed
		if(!m_lpGPU->enqueueWriteBuffer(inMemCellParams, sizeof(CellParam), &m_param))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;
		
		//Set the arguments to the compute kernel	
		//Setup Kernel Call, input params first then the output structures
		m_lpKernelCellConfig->setArg(0, sizeof(cl_mem), &inMemHeader);
		m_lpKernelCellConfig->setArg(1, sizeof(cl_mem), &inMemOps);
		m_lpKernelCellConfig->setArg(2, sizeof(cl_mem), &inMemPrims);
		m_lpKernelCellConfig->setArg(3, sizeof(cl_mem), &inMemMtx);
		m_lpKernelCellConfig->setArg(4, sizeof(cl_mem), &inMemCellParams);
		m_lpKernelCellConfig->setArg(5, sizeof(cl_mem), &inMemVertexCountTable);
		m_lpKernelCellConfig->setArg(6, sizeof(cl_mem), &inMemTriTable);

		m_lpKernelCellConfig->setArg(7, sizeof(cl_mem), &outMemCellConfig);
		m_lpKernelCellConfig->setArg(8, sizeof(cl_mem), &outMemVertexCount);
		m_lpKernelCellConfig->setArg(9, sizeof(cl_mem), &outMemMeshVertex);
		m_lpKernelCellConfig->setArg(10, sizeof(cl_mem), &outMemMeshColor);
		m_lpKernelCellConfig->setArg(11, sizeof(cl_mem), &outMemMeshNormal);
		


		//Acquire Mesh for Writing
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);
		clEnqueueAcquireGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshNormal, 0, 0, 0);
		
		size_t local;
		// Get the maximum work group size for executing the kernel on the device
		cl_int err = clGetKernelWorkGroupInfo(m_lpKernelCellConfig->getKernel(),
				m_lpGPU->getDevice(),
				CL_KERNEL_WORK_GROUP_SIZE,
				sizeof(local), &local, NULL);
		if (err != CL_SUCCESS) 
		{
			cerr << "Error: Failed to retrieve kernel work group info! "
					<<  err << endl;
			exit(1);
		}

		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
		size_t szNeeded[3];
		for(int i=0; i<3; i++)
			szNeeded[i] = m_param.ctNeededCells[i];

		err = clEnqueueNDRangeKernel(m_lpGPU->getCommandQ(), m_lpKernelCellConfig->getKernel(),
				3, NULL, szNeeded, NULL,
				0, NULL, NULL);
		if (err) {
			char buffer[1024];
			sprintf(buffer, "Error: Failed to execute kernel! (%s)", ComputeDevice::oclErrorString(err));
			cerr << buffer << endl;
			return EXIT_FAILURE;
		}

		// Wait for all commands to complete
		m_lpGPU->finishAllCommands();

		//Release Mesh for Writing
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshNormal, 0, 0, 0);
		clEnqueueReleaseGLObjects(m_lpGPU->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);
		m_isValidVertex = m_isValidNormal = m_isValidColor = true;

		// Read back the results from the device to verify the output
		U8* arrVertexCount = new U8[m_ctCells];
		U8* arrConfigIndex = new U8[m_ctCells];		
		U32* arrVertexBufferOffset = new U32[m_ctCells];

		U32 ctSumVertices = 0;
		U32 ctFilled = 0;
		if(m_lpGPU->enqueueReadBuffer(outMemVertexCount, sizeof(U8) * m_ctCells, arrVertexCount) &&
		   m_lpGPU->enqueueReadBuffer(outMemCellConfig, sizeof(U8) * m_ctCells, arrConfigIndex))		  
		{						
			//Compute Vertex Buffer offsets for writing the output to
			for(U32 i=0; i < m_ctCells; i++)
			{
				arrVertexBufferOffset[i] = ctSumVertices;
				ctSumVertices += arrVertexCount[i];	
			
				if((arrConfigIndex[i] != 0)&&(arrConfigIndex[i] != 255))
				{
					//arrConfigIndex[i] = arrConfigIndex[i];
					ctFilled ++;
				}
			}
		}

		// Shutdown and cleanup
		clReleaseMemObject(outMemCellConfig);
		clReleaseMemObject(outMemVertexCount);

		clReleaseMemObject(inMemTriTable);
		clReleaseMemObject(inMemVertexCountTable);

		clReleaseMemObject(inMemHeader);
		clReleaseMemObject(inMemOps);
		clReleaseMemObject(inMemPrims);		
		clReleaseMemObject(inMemMtx);
		clReleaseMemObject(inMemCellParams);

		SAFE_DELETE(arrConfigIndex);
		SAFE_DELETE(arrVertexCount);
		//SAFE_DELETE(arrFields);
		return 1;
	}


	/*!
	 * Runs ComputeConfig and ComputeMesh kernels in tandem.
	 */
	int GPUPoly::runTandem(float cellsize)
	{
		//Clear previous mesh
		GLMeshBuffer::cleanup();

		//Set CellSize
		m_arrHeader[OFFSET_CELLSIZE] = cellsize;
		
		//BBOX
		svec3f lower = svec3f(m_arrHeader[0], m_arrHeader[1], m_arrHeader[2]);
		svec3f upper = svec3f(m_arrHeader[4], m_arrHeader[5], m_arrHeader[6]);
		svec3f temp = vscale3f(1.0f/ cellsize, vsub3f(upper, lower));
		
		//Cell Params
		m_param.ctNeededCells[0] = (U32)ceil(temp.x) + 1;
		m_param.ctNeededCells[1] = (U32)ceil(temp.y) + 1;
		m_param.ctNeededCells[2] = (U32)ceil(temp.z) + 1;
		m_param.ctTotalCells = m_param.ctNeededCells[0] * m_param.ctNeededCells[1] * m_param.ctNeededCells[2];
		m_ctCells = m_param.ctTotalCells;

		//Check Matrices
		for(int i = 0; i < m_mtxNode.count; i++)
		{
			printf("PRIM MTX: %d\n", i);
			for(int j=0;j < PRIM_MATRIX_STRIDE; j++)
			{
				printf("%.2f, ", m_mtxNode.matrix[i * PRIM_MATRIX_STRIDE + j]);
				if((j+1) % 4 == 0)
					printf("\n");
			}
			//printf("\n");
		}

		//Input Pos
		cl_mem inoutMemCellConfig;
		cl_mem inoutMemVertexCount;
		cl_mem inMemHeader;
		cl_mem inMemOps;
		cl_mem inMemPrims;		
		cl_mem inMemMtx;
		cl_mem inMemCellParams;
		U32 ctOpsToCopy = MATHMAX(m_ctOps, 1);
		U32 ctPrimsToCopy = MATHMAX(m_ctPrims, 1);
		
		// Create the device memory vectors
		inoutMemCellConfig = m_lpGPU->createMemBuffer(sizeof(U8) * m_ctCells, ComputeDevice::memReadWrite);
		inoutMemVertexCount = m_lpGPU->createMemBuffer(sizeof(U8) * m_ctCells, ComputeDevice::memReadWrite);
			
		inMemHeader = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_HEADER, ComputeDevice::memReadOnly);
		inMemOps  	= m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, ComputeDevice::memReadOnly);
		inMemPrims  = m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, ComputeDevice::memReadOnly);		
		inMemMtx	= m_lpGPU->createMemBuffer(sizeof(float) * PRIM_MATRIX_STRIDE * m_mtxNode.count, ComputeDevice::memReadOnly);
		inMemCellParams = m_lpGPU->createMemBuffer(sizeof(CellParam), ComputeDevice::memReadOnly);

		// Transfer the input vector into device memory.
		//Header
		if(!m_lpGPU->enqueueWriteBuffer(inMemHeader, sizeof(float) * DATASIZE_HEADER, m_arrHeader))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Ops
		if(!m_lpGPU->enqueueWriteBuffer(inMemOps, sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, m_arrOps))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Prims
		if(!m_lpGPU->enqueueWriteBuffer(inMemPrims, sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, m_arrPrims))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Matrix
		if(!m_lpGPU->enqueueWriteBuffer(inMemMtx, sizeof(float) * PRIM_MATRIX_STRIDE * m_mtxNode.count, m_mtxNode.matrix))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Needed
		if(!m_lpGPU->enqueueWriteBuffer(inMemCellParams, sizeof(CellParam), &m_param))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;
		
		//Kernel1
		/*
		__kernel void ComputeConfig(__global float4* arrInHeader4,
							__global float4* arrInOps4,										 
							__global float4* arrInPrims4,											 
	 					    __global float4* arrInMtxNodes4,
							__constant struct CellParam* inCellParams,
							__read_only image2d_t texInVertexCountTable,
							__global uchar* arrOutCellConfig,
							__global uchar* arrOutVertexCount)								
		*/		
		//Set the arguments to the compute kernel	
		//Setup Kernel Call, input params first then the output structures
		m_lpKernelComputeConfig->setArg(0, sizeof(cl_mem), &inMemHeader);
		m_lpKernelComputeConfig->setArg(1, sizeof(cl_mem), &inMemOps);
		m_lpKernelComputeConfig->setArg(2, sizeof(cl_mem), &inMemPrims);
		m_lpKernelComputeConfig->setArg(3, sizeof(cl_mem), &inMemMtx);
		m_lpKernelComputeConfig->setArg(4, sizeof(cl_mem), &inMemCellParams);
		m_lpKernelComputeConfig->setArg(5, sizeof(cl_mem), &m_inMemVertexCountTable);
		m_lpKernelComputeConfig->setArg(6, sizeof(cl_mem), &inoutMemCellConfig);
		m_lpKernelComputeConfig->setArg(7, sizeof(cl_mem), &inoutMemVertexCount);

	
		size_t local = m_lpGPU->getKernelWorkgroupSize(m_lpKernelComputeConfig);

		// Execute the kernel over the vector using the
		// maximum number of work group items for this device
		size_t szNeeded[3];
		for(int i=0; i<3; i++)
			szNeeded[i] = m_param.ctNeededCells[i];

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
		U8* arrVertexCount = new U8[m_ctCells];
		U8* arrConfigIndex = new U8[m_ctCells];		
		U32* arrVertexBufferOffset = new U32[m_ctCells];
		U32* arrTetMeshOffset = new U32[m_ctCells];

		U32 ctSumVertices = 0;
		U32 ctCrossedOnly = 0;
		U32 ctCrossedOrInside = 0;
		if(m_lpGPU->enqueueReadBuffer(inoutMemVertexCount, sizeof(U8) * m_ctCells, arrVertexCount) &&
		   m_lpGPU->enqueueReadBuffer(inoutMemCellConfig, sizeof(U8) * m_ctCells, arrConfigIndex))		  
		{						
			//Compute Vertex Buffer offsets for writing the output to
			for(U32 i=0; i < m_ctCells; i++)
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
		inMemVertexBufferOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_ctCells, ComputeDevice::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(inMemVertexBufferOffset, sizeof(U32) * m_ctCells, arrVertexBufferOffset))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//Tetmesh buffer offset
		cl_mem inMemTetMeshBufferOffset;
		inMemTetMeshBufferOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_ctCells, ComputeDevice::memReadOnly);
		if(!m_lpGPU->enqueueWriteBuffer(inMemTetMeshBufferOffset, sizeof(U32) * m_ctCells, arrTetMeshOffset))
			return ERR_GPUPOLY_BUFFER_NOT_WRITTEN;

		//////////////////////////////////////////////////////////////////////////
		//Kernel2:
		m_stepVertex = 4;
		m_stepColor = 4;
		U32 szVertexBuffer = ctSumVertices * m_stepVertex * sizeof(float);
		U32 szNormalBuffer = ctSumVertices * 3 * sizeof(float);

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

		m_lpKernelComputeMesh->setArg(0, sizeof(cl_mem), &inMemHeader);
		m_lpKernelComputeMesh->setArg(1, sizeof(cl_mem), &inMemOps);
		m_lpKernelComputeMesh->setArg(2, sizeof(cl_mem), &inMemPrims);
		m_lpKernelComputeMesh->setArg(3, sizeof(cl_mem), &inMemMtx);
		m_lpKernelComputeMesh->setArg(4, sizeof(cl_mem), &inMemCellParams);
		m_lpKernelComputeMesh->setArg(5, sizeof(cl_mem), &m_inMemTriangleTable);
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


		clReleaseMemObject(inMemHeader);
		clReleaseMemObject(inMemOps);
		clReleaseMemObject(inMemPrims);		
		clReleaseMemObject(inMemMtx);
		clReleaseMemObject(inMemCellParams);

		SAFE_DELETE(arrConfigIndex);
		SAFE_DELETE(arrVertexCount);
		SAFE_DELETE(arrVertexBufferOffset);
		SAFE_DELETE(arrTetMeshOffset);
		return 1;
	}


	int Run_SphereDistKernel()
	{
		DAnsiStr strFP = ExtractOneLevelUp(ExtractFilePath(GetExePath()));
		strFP += DAnsiStr("/PS_OpenCLKernels/SphereDist.cl");
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

