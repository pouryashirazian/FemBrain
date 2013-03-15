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

		//Cleanup rest pos vertices
		clReleaseMemObject(m_inoutMemRestPos);

		//Clear MC Tables
		clReleaseMemObject(m_inMemTriangleTable);
		clReleaseMemObject(m_inMemVertexCountTable);

		//Clear Mem Objects
		if(m_bModelLoaded)
		{
			clReleaseMemObject(m_inMemHeader);
			clReleaseMemObject(m_inMemOps);
			clReleaseMemObject(m_inMemPrims);
			clReleaseMemObject(m_inMemMtx);
		}

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

		//Model File Path
		m_strModelFilePath = string(lpFilePath);

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

		//Root Op
		m_arrHeader[11] = NULL_BLOB;

		//Count
		m_ctPrims = tempPrims.count;
		m_ctOps = tempOps.count;

		//Prims
		for(U32 i=0; i < tempPrims.count; i++)
		{
			//Update Parent/Link and Link Properties
			m_arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE] = static_cast<float>(tempPrims.type[i]);
			m_arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_IDX_MATRIX] = static_cast<float>(tempPrims.idxMatrix[i]);
			m_arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT] = 0;
			m_arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_SIBLING] = 0;

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
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_LC] 	 = static_cast<float>(tempOps.opLeftChild[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_RC]   = static_cast<float>(tempOps.opRightChild[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = NULL_BLOB;

				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_RES_X] = static_cast<float>(tempOps.resX[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_RES_Y] = static_cast<float>(tempOps.resY[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_RES_Z] = static_cast<float>(tempOps.resZ[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_FLAGS] = static_cast<float>(tempOps.opFlags[i]);

				//AABB
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO] = static_cast<float>(tempOps.bboxLoX[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 1] = static_cast<float>(tempOps.bboxLoY[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 2] = static_cast<float>(tempOps.bboxLoZ[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 3] = 1.0f;
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI] = static_cast<float>(tempOps.bboxHiX[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 1] = static_cast<float>(tempOps.bboxHiY[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 2] = static_cast<float>(tempOps.bboxHiZ[i]);
				m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 3] = 1.0f;
			}
		}
		else {
			m_arrOps[OFFSET_OP_NEXT] = NULL_BLOB;
		}

		//Set Traversal Route
		if(!setTraversalRoute())
		{
			LogError("Failed to find a tree traversal route in this BlobTree!");
			return false;
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
			return false;

		//Ops
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemOps, sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, m_arrOps))
			return false;

		//Prims
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemPrims, sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, m_arrPrims))
			return false;

		//Matrix
		if(!m_lpGPU->enqueueWriteBuffer(m_inMemMtx, sizeof(float) * PRIM_MATRIX_STRIDE * m_mtxNode.count, m_mtxNode.matrix))
			return false;
		m_bModelLoaded = true;

		return true;
	}

	bool GPUPoly::setTraversalRoute() {
		printBlobTree("blobtree.txt");

		//Set OP next and Link Properties
		SIMPLESTACK<MAX_TREE_NODES> stkOps;
		SIMPLESTACK<MAX_TREE_NODES> stkLastBreak;

		stkOps.push(0);
		stkLastBreak.push(NULL_BLOB);

		//Process All Ops
		while(!stkOps.empty())
		{
			U16 idxOp = stkOps.top();
			U32 opFlags = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_FLAGS]);
			bool isBreak = (bool)((opFlags & ofBreak) >> 5);
			bool isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
			bool isRCOp  = (bool) (opFlags & ofRightChildIsOp);

			U32 idxLC = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_LC]);
			U32 idxRC = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RC]);

			//Process a break when we got two primitive kids
			if(isLCOp == false && isRCOp == false && stkLastBreak.empty() == false)
			{
				//Remove key op from stack
				stkOps.pop();
				U16 idxBreak = stkLastBreak.top();
				stkLastBreak.pop();

				//Set Root Op
				if(idxBreak == NULL_BLOB)
					m_arrHeader[OFFSET_HEADER_START] = idxOp;
				else
					m_arrOps[idxBreak * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = idxOp;

				if(isBreak)
					stkLastBreak.push(idxOp);

				//Pop while not at a break node
				while(!isBreak && !stkOps.empty())
				{
					idxOp = stkOps.top();
					stkOps.pop();

					opFlags = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_FLAGS]);
					isBreak = (bool)((opFlags & ofBreak) >> 5);
					//bool isRop   = (bool)((opFlags & ofIsRightOp) >> 4);
					//bool isUnary = (bool)((opFlags & ofIsUnaryOp) >> 3);
					//bool isRange = (bool)((opFlags & ofChildIndexIsRange) >> 2);
					isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
					isRCOp  = (bool) (opFlags & ofRightChildIsOp);

					//LC
					idxLC = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_LC]);

					//RC
					idxRC = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RC]);

					if(isBreak)
						stkLastBreak.push(idxOp);
					if(isLCOp)
						m_arrOps[idxLC * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = idxOp;
					if(isRCOp)
						m_arrOps[idxRC * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = idxOp;
				}
			}
			else
			{
				if(isLCOp)
					stkOps.push(idxLC);

				if(isRCOp)
					stkOps.push(idxRC);
			}
		}

		//Test Route
		//Check next links and connections
		if(m_arrOps[OFFSET_OP_NEXT] != NULL_BLOB)
		{
			LogError("Root operator next pointer not set.");
			return false;
		}

		int ctErrors = 0;
		for(U32 i=1; i< m_ctOps; i++)
		{
			U16 next = m_arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_NEXT];
			if(next < 0 || next >= m_ctOps)
			{
				ctErrors ++;
				LogErrorArg1("Next pointer not set properly at operator #%d", i);
			}
		}

		return (ctErrors == 0);
	}

	void GPUPoly::printBlobTree(const char* chrFilePath) const
	{
		ofstream ofs;
		ofs.open(chrFilePath, ios::out);

		//Compute Tree Depth
		U32 maxDepth = 0;

		//Compute MAXDEPTH
		if(m_ctOps > 0)
		{
			GENERIC_PAIR_STACK<U32, MAX_TREE_NODES> stkDepth;
			stkDepth.push(0, 0);
			while(!stkDepth.empty())
			{
				U32 idxOp = stkDepth.topFirst();
				U32 depth = stkDepth.topSecond();
				maxDepth = MATHMAX(maxDepth, depth);
				stkDepth.pop();

				U32 opFlags = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_FLAGS]);
				bool isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
				bool isRCOp  = (bool) (opFlags & ofRightChildIsOp);

				U16 idxLC   = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_LC]);
				U16 idxRC   = static_cast<U32>(m_arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RC]);

				if(isLCOp)
					stkDepth.push(idxLC, depth + 1);
				if(isRCOp)
					stkDepth.push(idxRC, depth + 1);
			}

			//Last Op has prims
			maxDepth ++;
		}


		if(m_ctOps > 0)
		{
			list<NODE> lstNodes;
			lstNodes.push_back(NODE(0, 0, 1));
			U32 currentDepth = -1;
			while(!lstNodes.empty())
			{
				NODE n = lstNodes.front();
				lstNodes.pop_front();

				if(n.depth != currentDepth)
				{
					ofs << endl;
					currentDepth = n.depth;
					for(int i=0; i<(maxDepth - currentDepth); i++)
						ofs << "\t\t\t";
				}

				if(n.isOp)
				{
					U32 opType  = static_cast<U32>(m_arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_TYPE]);
					U32 opFlags = static_cast<U32>(m_arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_FLAGS]);
					bool isUnary = (bool)((opFlags & ofIsUnaryOp) >> 3);
					bool isRange = (bool)((opFlags & ofIsUnaryOp) >> 2);
					bool isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
					bool isRCOp  = (bool) (opFlags & ofRightChildIsOp);

					U16 idxLC   = static_cast<U32>(m_arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_LC]);
					U16 idxRC   = static_cast<U32>(m_arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_RC]);

					//Visit Node
					DAnsiStr strName = ModelReader::GetScriptOpName(opType);
					ofs << "OP#" << n.index << ":" << strName.substr(0, 5) << "\t";

					if(isRange)
					{
						for(int i = idxLC; i <= idxRC; i++)
							lstNodes.push_back(NODE(i, n.depth + 1, isLCOp));
					}
					else
					{
						lstNodes.push_back(NODE(idxLC, n.depth + 1, isLCOp));
						if(!isUnary)
							lstNodes.push_back(NODE(idxRC, n.depth + 1, isRCOp));
					}
				}
				else
				{
					U16 primType = static_cast<U16>(m_arrPrims[n.index * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE]);
					DAnsiStr strName = ModelReader::GetScriptPrimName(primType);
					ofs << "PR#" << n.index << ":" << strName.substr(0, 5) << "\t";
				}
			}
		}
		else
		{
			U16 primType = static_cast<U16>(m_arrPrims[OFFSET_PRIM_TYPE]);
			DAnsiStr strName = ModelReader::GetScriptPrimName(primType);
			ofs << "PR#0:" << strName.substr(0, 5) << "\t";
		}
		/*
		//Iterate over depths
		for(int i=0; i<=maxDepth; i++)
		{
			//Tabs
			for(int j=0; j<(maxDepth - i); j++)
				ofs << "\t\t\t";

			for(int j=0; j<arrNodes.size(); j++)
			{
				NODE n = arrNodes[j];
				if(n.depth == i)
				{
					if(n.isOp) {
						U32 opFlags = static_cast<U32>(m_arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS]);
						bool isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
						bool isRCOp  = (bool) (opFlags & ofRightChildIsOp);
						U16 opType = static_cast<U16>(m_arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_TYPE]);


						DAnsiStr strName = ModelReader::GetScriptOpName(opType);
						ofs << "OP#" << n.index << ":" << strName.substr(0, 3) << "\t";
					}
					else {
						U16 primType = static_cast<U16>(m_arrPrims[n.index * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE]);
						DAnsiStr strName = ModelReader::GetScriptPrimName(primType);
						ofs << "PR#" << n.index << ":" << strName.substr(0, 3) << "\t";
					}
				}
			}

			ofs << endl;
		}
		*/

		ofs.close();
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
		m_lpGPU = new ComputeDevice(ComputeDevice::dtGPU, true, false, "AMD");
		m_lpGPU->printInfo();
		

		//Create the OCL Scan Primitive
		//PS::HPC::Scan* lpScanner = new PS::HPC::Scan(lpGPU);
		//lpScanner->scanExclusiveLarge()
		LogInfo("2.Compile OpenCL program.");
		ComputeProgram* lpProgram = m_lpGPU->tryLoadBinaryThenCompile(strFP.cptr());
		assert(lpProgram != NULL);


		//Build Kernel
		LogInfo("3.Setup Kernel Functions.");
		//m_lpKernelComputeConfig = lpProgram->addKernel("ComputeConfigIndexVertexCount");
		//m_lpKernelComputeConfig = lpProgram->addKernel("ComputeConfig");
		//m_lpKernelComputeMesh = lpProgram->addKernel("ComputeMesh");


		//Polygonizer
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

		//Tetrahedralizer
		m_lpKernelTetMeshCountCells = lpProgram->addKernel("TetMeshCells");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelTetMeshCountCells);

		m_lpKernelTetMeshVertices = lpProgram->addKernel("TetMeshVertices");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelTetMeshVertices);

		m_lpKernelTetMeshElements = lpProgram->addKernel("TetMeshElements");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelTetMeshElements);

		m_lpKernelApplyDeformations = lpProgram->addKernel("ApplyVertexDeformations");
		m_lpGPU->getKernelWorkgroupSize(m_lpKernelApplyDeformations);

		//Init some vars
		m_ctTetMeshVertices = m_ctTetMeshElements = 0;

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
	int GPUPoly::runMultiPass(float cellsize, bool outputTetMesh)
	{
		if(DefinitelyLessThan(cellsize, 0.01, EPSILON))
			return -1;

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
			//PrintArray(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);

			//3. SumScan to compute crossed edges
			m_ctVertices = m_lpOclSumScan->compute(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);
			//PrintArray(lpEdgeTableOffsets, m_gridParam.ctTotalPoints);

			//Write Edge Offsets to device memory
			m_inMemHighEdgesOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_gridParam.ctTotalPoints, ComputeDevice::memReadOnly);
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
			//Read-back Elements Count
			U32* lpCellElementsOffset = new U32[m_cellParam.ctTotalCells];
			m_lpGPU->enqueueReadBuffer(m_inoutMemCellElementsCount, sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);
			//PrintArray(lpCellElementsOffset, m_cellParam.ctTotalCells);

			//6. SumScan to read all elements
			m_ctFaceElements = m_lpOclSumScan->compute(lpCellElementsOffset, m_cellParam.ctTotalCells);
			//PrintArray(lpCellElementsOffset, m_cellParam.ctTotalCells);

			//Write Element Offsets
			m_inMemCellElementsOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, ComputeDevice::memReadOnly);
			m_lpGPU->enqueueWriteBuffer(m_inMemCellElementsOffset, sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);
			SAFE_DELETE(lpCellElementsOffset);
		}

		//7. Compute Elements
		computeElements(m_ctFaceElements);

		/////////////////////////////////////////////////////////////////////
		// Tetrahedralization
		if(outputTetMesh) {
			//8. Find all cells and vertices includes in the tetmesh
			computeTetMeshCellsInsideOrCrossed();

			//SumScan to Compute Total Tetrahedra elements produced and elements offset
			{
				//Read-back Elements Count
				U32* lpCellElementsOffset = new U32[m_cellParam.ctTotalCells];
				m_lpGPU->enqueueReadBuffer(m_inoutMemCellElementsCount, sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);

				//9. SumScan to read all elements, 6 Tetrahedra per each cell
				m_ctTetMeshElements = m_lpOclSumScan->compute(lpCellElementsOffset, m_cellParam.ctTotalCells);
				m_ctTetMeshElements *= 6;

				//Write Element Offsets
				//m_inMemCellElementsOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_cellParam.ctTotalCells, ComputeDevice::memReadOnly);
				m_lpGPU->enqueueWriteBuffer(m_inMemCellElementsOffset, sizeof(U32) * m_cellParam.ctTotalCells, lpCellElementsOffset);
				SAFE_DELETE(lpCellElementsOffset);
			}

			//SumScan to Compute Total Vertices count and vertex list offsets
			{
				//Read-back EdgeTable Count
				U32* lpVerticesIncluded = new U32[m_gridParam.ctTotalPoints];
				m_lpGPU->enqueueReadBuffer(m_inoutMemHighEdgesCount, sizeof(U32) * m_gridParam.ctTotalPoints, lpVerticesIncluded);

				//10. SumScan to compute included vertices
				m_ctTetMeshVertices = m_lpOclSumScan->compute(lpVerticesIncluded, m_gridParam.ctTotalPoints);

				//Write Edge Offsets to device memory
				//m_inMemHighEdgesOffset = m_lpGPU->createMemBuffer(sizeof(U32) * m_gridParam.ctTotalPoints, ComputeDevice::memReadOnly);
				m_lpGPU->enqueueWriteBuffer(m_inMemHighEdgesOffset, sizeof(U32) * m_gridParam.ctTotalPoints, lpVerticesIncluded);
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
		}

		//Draw Mesh
		m_isValidIndex = true;
		m_faceMode = ftTriangles;

		/*
		m_isValidIndex = false;
		m_ctFaceElements = m_ctVertices;
		m_faceMode = ftPoints;
		*/

		//Release all temp vars
		clReleaseMemObject(m_inMemCellParam);
		clReleaseMemObject(m_inMemGridParam);

		clReleaseMemObject(m_inoutMemAllFields);
		clReleaseMemObject(m_inoutMemHighEdgesCount);
		clReleaseMemObject(m_inoutMemHighEdgesFlags);
		clReleaseMemObject(m_inMemHighEdgesOffset);

		clReleaseMemObject(m_inoutMemCellConfig);
		clReleaseMemObject(m_inoutMemCellElementsCount);
		clReleaseMemObject(m_inMemCellElementsOffset);


		return 1;
	}

	bool GPUPoly::readBackMesh(U32& ctVertices, vector<float>& vertices,
								  U32& ctFaceElements, vector<U32>& elements) {
		if(!m_isValidIndex)
			return false;

		ctVertices = m_ctVertices;
		ctFaceElements = m_ctFaceElements;

		//Vertices
		vector<float> homogenousVertices;
		homogenousVertices.resize(m_ctVertices * 4);
		vertices.resize(m_ctVertices * 3);
		elements.resize(m_ctFaceElements);

		//Vertices
		U32 szVertexBuffer = sizeof(float) * 4 * m_ctVertices;
		U32 szIndexBuffer = sizeof(U32) * m_ctFaceElements;


		//Vertices
		cl_mem outMemMeshVertices = m_lpGPU->createMemBufferFromGL(m_vboVertex, ComputeDevice::memReadOnly);
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
		cl_mem outMemMeshElements = m_lpGPU->createMemBufferFromGL(m_iboFaces, ComputeDevice::memReadOnly);
		m_lpGPU->enqueueAcquireGLObject(1, &outMemMeshElements);
		m_lpGPU->enqueueReadBuffer(outMemMeshElements, szIndexBuffer, &elements[0]);
		m_lpGPU->enqueueReleaseGLObject(1, &outMemMeshElements);
		m_lpGPU->finishAllCommands();
		//PrintArray(&elements[0], 512);

		clReleaseMemObject(outMemMeshVertices);
		clReleaseMemObject(outMemMeshElements);

		return true;
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

		//Create memory buffer for vertex rest positions
		m_inoutMemRestPos = m_lpGPU->createMemBuffer(szVertexBuffer, ComputeDevice::memReadWrite);

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
		m_lpGPU->enqueueCopyBuffer(outMemMeshVertex, m_inoutMemRestPos, 0, 0, szVertexBuffer);

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

		//ComputeEdgeTable
		m_lpGPU->enqueueNDRangeKernel(m_lpKernelComputeEdgeTable, 3, arrGlobalIndex, arrLocalIndex);

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
		m_outMemTetMeshVertices = m_lpGPU->createMemBuffer(szVertexBuffer, ComputeDevice::memReadWrite);
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
		m_outMemTetMeshElements = m_lpGPU->createMemBuffer(szElementsBuffer, ComputeDevice::memReadWrite);


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



