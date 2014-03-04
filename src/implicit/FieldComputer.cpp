//
//  FieldComputer.cpp
//  hifem
//
//  Created by pshiraz on 1/15/14.
//  Copyright (c) 2014 pshiraz. All rights reserved.
//

#include "FieldComputer.h"
#include "graphics/CLManager.h"
#include "base/FileDirectory.h"
#include "base/Logger.h"
#include "base/Profiler.h"

using namespace PS;
using namespace PS::FILESTRINGUTILS;

namespace PS {
    namespace SKETCH {

        FieldComputer::FieldComputer() {
            init();
        }
        
        FieldComputer::FieldComputer(const LinearBlobTree& blob) {
            init();
            setBlob(blob);
        }
        
        FieldComputer::~FieldComputer() {
            cleanup();
        }
        
        void FieldComputer::init() {
    		//Set Rendering Shader
            if(TheShaderManager::Instance().has("bluepoints")) {
                m_spEffect = SmartPtrSGEffect(new SGEffect(TheShaderManager::Instance().get("bluepoints")));
            }

        	AnsiStr strCodePath = ExtractOneLevelUp(ExtractFilePath(GetExePath()));
            AnsiStr strPolyFP = strCodePath + AnsiStr("data/shaders/Polygonizer.cl");

    		//Create a GPU Compute Device
    		m_lpGPU = TheCLManager::Instance().device();
       		int prgID = m_lpGPU->addProgramFromFile(strPolyFP);
    		assert(prgID >= 0);

    		m_lpKernelFieldVoxelGridStackBased = m_lpGPU->addKernel(prgID, "ComputeAllFieldsStackBased");
    		m_lpKernelFieldVoxelGridStackLess = m_lpGPU->addKernel(prgID, "ComputeAllFields");

    		m_lpKernelFieldArrayStackBased = m_lpGPU->addKernel(prgID, "ComputeFieldArrayStackBased");
    		m_lpKernelFieldArrayStackLess = m_lpGPU->addKernel(prgID, "ComputeFieldArray");

     		m_blobLoaded = false;
     		m_vboVertex = INVALID_GLBUFFER;
        }
      
        void FieldComputer::cleanup() {
        	clearBlobBuffers();
        }

        void FieldComputer::clearBlobBuffers() {
            m_inMemHeader.release();
            m_inMemMtx.release();
            m_inMemOps.release();
            m_inMemPrims.release();
        }

        bool FieldComputer::setBlob(const LinearBlobTree& blob) {
    		if(blob.countPrimitives() == 0) {
    			LogError("There is no primitives in the blobtree!");
    			return false;
    		}

    		if(blob.countMtxNodes() == 0) {
    			LogError("There is no mtx nodes in the blobtree!");
    			return false;
    		}

    		//Clear previous buffers
    		clearBlobBuffers();

    		//Copy Blob
    		m_blob.copyFrom(blob);
    		m_aabb = blob.aabb();
            m_blobLoaded = true;


    		//Model memories
    		U32 ctOpsToCopy = MATHMAX(m_blob.countOperators(), 1);
    		U32 ctPrimsToCopy = MATHMAX(m_blob.countPrimitives(), 1);

    		//Memory Buffers
    		m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_HEADER, PS::CL::memReadOnly, m_inMemHeader);
    		m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_OPERATOR * ctOpsToCopy, PS::CL::memReadOnly, m_inMemOps);
    		m_lpGPU->createMemBuffer(sizeof(float) * DATASIZE_PRIMITIVE * ctPrimsToCopy, PS::CL::memReadOnly, m_inMemPrims);
    		m_lpGPU->createMemBuffer(sizeof(float) * PRIM_MATRIX_STRIDE * m_blob.countMtxNodes(), PS::CL::memReadOnly, m_inMemMtx);

    		//Header
    		if(!m_lpGPU->enqueueWriteBuffer(m_inMemHeader.handle(), m_inMemHeader.size(), m_blob.arrHeader.cptr()))
    			return false;

    		//Ops
    		if(!m_lpGPU->enqueueWriteBuffer(m_inMemOps.handle(), m_inMemOps.size(), m_blob.arrOps.cptr()))
    			return false;

    		//Prims
    		if(!m_lpGPU->enqueueWriteBuffer(m_inMemPrims.handle(), m_inMemPrims.size(), m_blob.arrPrims.cptr()))
    			return false;

    		//Matrix
    		if(!m_lpGPU->enqueueWriteBuffer(m_inMemMtx.handle(), m_inMemMtx.size(), m_blob.arrMtxNodes.cptr()))
    			return false;

    		return true;

        }

        void FieldComputer::draw() {
            if(m_spEffect)
                m_spEffect->bind();

        	glPushAttrib(GL_ALL_ATTRIB_BITS);

        	if(m_isValidVertex)
        	{
        		glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
        		glVertexPointer(m_stepVertex, GL_FLOAT, 0, 0);
        		glEnableClientState(GL_VERTEX_ARRAY);
        	}

        	glDrawArrays(m_faceMode, 0, m_ctVertices);

        	if(m_isValidVertex)
        		glDisableClientState(GL_VERTEX_ARRAY);

        	glPopAttrib();

            if(m_spEffect)
                m_spEffect->unbind();
        }

        int FieldComputer::fieldsForVoxelGrid(float cellsize, bool stackless) {
        	AnsiStr strArg = printToAStr("Compute Fields for voxel grid using %s method", stackless ? "stackless" : "stack-based");
        	ProfileAutoArg(strArg.cptr());

        	//Release Grid Fields
            if(glIsBuffer(m_vboVertex))
            	glDeleteBuffers(1, &m_vboVertex);


			//BBOX
			vec3f lower = vec3f(&m_blob.arrHeader[0]);
			vec3f upper = vec3f(&m_blob.arrHeader[4]);
			vec3f temp = vec3f::mul(1.0f/ cellsize, vec3f::sub(upper, lower));

			VoxelGrid grid;
			grid.axialCount[0] = (U32)ceil(temp.x) + 2;
			grid.axialCount[1] = (U32)ceil(temp.y) + 2;
			grid.axialCount[2] = (U32)ceil(temp.z) + 2;
			grid.total = grid.axialCount[0] * grid.axialCount[1] * grid.axialCount[2];
			grid.cellsize = cellsize;

			size_t arrLocalIndex[3];
			size_t arrGlobalIndex[3];
			for(int i=0; i<3; i++)
				arrGlobalIndex[i] = grid.axialCount[i];

			//GridParam
			Buffer inMemGridParam;
			m_lpGPU->createMemBuffer(sizeof(VoxelGrid), PS::CL::memReadOnly, inMemGridParam);
			if(!m_lpGPU->enqueueWriteBuffer(inMemGridParam.handle(), inMemGridParam.size(), &grid))
				return -1;

			//Create Memory for all field points
			cl_mem inoutMemVertices;
			glGenBuffers(1, &m_vboVertex);
			glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
			glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * grid.total, 0, GL_DYNAMIC_DRAW);
			inoutMemVertices = m_lpGPU->createMemBufferFromGL(m_vboVertex, PS::CL::memReadWrite);
			m_lpGPU->enqueueAcquireGLObject(1, &inoutMemVertices);

			//m_lpGPU->createMemBuffer(sizeof(float) * 4 * grid.total, PS::CL::memReadWrite, m_inoutMemGrid);

			if(stackless) {
				ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelFieldVoxelGridStackLess->getKernelWorkGroupSize(), arrLocalIndex);
				ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

				m_lpKernelFieldVoxelGridStackLess->setArg(0, sizeof(cl_mem), m_inMemHeader.cptr());
				m_lpKernelFieldVoxelGridStackLess->setArg(1, sizeof(cl_mem), m_inMemOps.cptr());
				m_lpKernelFieldVoxelGridStackLess->setArg(2, sizeof(cl_mem), m_inMemPrims.cptr());
				m_lpKernelFieldVoxelGridStackLess->setArg(3, sizeof(cl_mem), m_inMemMtx.cptr());
				m_lpKernelFieldVoxelGridStackLess->setArg(4, sizeof(cl_mem), inMemGridParam.cptr());
				m_lpKernelFieldVoxelGridStackLess->setArg(5, sizeof(cl_mem), &inoutMemVertices);

				// Execute the kernel over the vector using the
				// maximum number of work group items for this device
				m_lpGPU->enqueueNDRangeKernel(m_lpKernelFieldVoxelGridStackLess, 3, arrGlobalIndex, arrLocalIndex);

			}
			else {

				ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelFieldVoxelGridStackBased->getKernelWorkGroupSize(), arrLocalIndex);
				ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

				m_lpKernelFieldVoxelGridStackBased->setArg(0, sizeof(cl_mem), m_inMemHeader.cptr());
				m_lpKernelFieldVoxelGridStackBased->setArg(1, sizeof(cl_mem), m_inMemOps.cptr());
				m_lpKernelFieldVoxelGridStackBased->setArg(2, sizeof(cl_mem), m_inMemPrims.cptr());
				m_lpKernelFieldVoxelGridStackBased->setArg(3, sizeof(cl_mem), m_inMemMtx.cptr());
				m_lpKernelFieldVoxelGridStackBased->setArg(4, sizeof(cl_mem), inMemGridParam.cptr());
				m_lpKernelFieldVoxelGridStackBased->setArg(5, sizeof(cl_mem), &inoutMemVertices);

				// Execute the kernel over the vector using the
				// maximum number of work group items for this device
				m_lpGPU->enqueueNDRangeKernel(m_lpKernelFieldVoxelGridStackBased, 3, arrGlobalIndex, arrLocalIndex);
			}

			//ReleaseGL
			m_lpGPU->enqueueReleaseGLObject(1, &inoutMemVertices);

			// Wait for all commands to complete
			m_lpGPU->finishAllCommands();

			//Release
			m_ctVertices = m_ctFaceElements = grid.total;
			m_faceMode = ftPoints;
			m_stepVertex = 4;
			m_isValidVertex = true;

			return (int)grid.total;
        }

        float FieldComputer::field(const vec3f& p) {
        	U32 szWorkGroup = m_lpKernelFieldArrayStackBased->getKernelWorkGroupSize();
        	vector<float> arrXYZF;
        	arrXYZF.resize(szWorkGroup * 4);
        	for(U32 i=0; i<szWorkGroup; i++)
        		p.store(&arrXYZF[i*4]);
        	field(szWorkGroup, 4, arrXYZF);
        	return arrXYZF[3];
        }

        float FieldComputer::field_stackless(const vec3f &p) {
        	U32 szWorkGroup = m_lpKernelFieldArrayStackLess->getKernelWorkGroupSize();
        	vector<float> arrXYZF;
        	arrXYZF.resize(szWorkGroup * 4);
        	for(U32 i=0; i<szWorkGroup; i++)
        		p.store(&arrXYZF[i*4]);
        	field(szWorkGroup, 4, arrXYZF);
        	return arrXYZF[3];
        }

        int FieldComputer::field(U32 ctVertices, U32 step, vector<float>& vertices) {
    		if(step != 4)
    			return -1;
    		if(step * ctVertices != vertices.size())
    			return -1;
        	/*
        	__kernel void ComputeFieldArrayStackBased(U32 ctVertices,
        								   __global float4* arrInHeader4,
        								   __global float4* arrInOps4,
        								   __global float4* arrInPrims4,
        								   __global float4* arrInMtxNodes4,
        								   __global float4* arrInOutVertexFields)
			*/

    		size_t arrLocalIndex[3];
    		size_t arrGlobalIndex[3];
    		arrGlobalIndex[0] = ctVertices;
    		arrGlobalIndex[1] = 0;
    		arrGlobalIndex[2] = 0;

    		ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelFieldArrayStackBased->getKernelWorkGroupSize(), arrLocalIndex);
    		ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);

    		//Create Memory for all field points
    		U32 szVertexBuffer = sizeof(float) * step * ctVertices;
    		Buffer inoutMemFieldArray;
    		m_lpGPU->createMemBuffer(szVertexBuffer, PS::CL::memReadWrite, inoutMemFieldArray);
    		if(!m_lpGPU->enqueueWriteBuffer(inoutMemFieldArray.handle(), inoutMemFieldArray.size(), &vertices[0]))
    			return -1;

    		m_lpKernelFieldArrayStackBased->setArg(0, sizeof(U32), (void *)&ctVertices);
    		m_lpKernelFieldArrayStackBased->setArg(1, sizeof(cl_mem), m_inMemHeader.cptr());
    		m_lpKernelFieldArrayStackBased->setArg(2, sizeof(cl_mem), m_inMemOps.cptr());
    		m_lpKernelFieldArrayStackBased->setArg(3, sizeof(cl_mem), m_inMemPrims.cptr());
    		m_lpKernelFieldArrayStackBased->setArg(4, sizeof(cl_mem), m_inMemMtx.cptr());
    		m_lpKernelFieldArrayStackBased->setArg(5, sizeof(cl_mem), inoutMemFieldArray.cptr());

    		// Execute the kernel over the vector using the
    		// maximum number of work group items for this device
    		m_lpGPU->enqueueNDRangeKernel(m_lpKernelFieldArrayStackBased, 1, arrGlobalIndex, arrLocalIndex);
    		m_lpGPU->enqueueReadBuffer(inoutMemFieldArray.handle(), szVertexBuffer, &vertices[0]);

    		// Wait for all commands to complete
    		m_lpGPU->finishAllCommands();

    		return ctVertices;
        }

        int FieldComputer::field_stackless(U32 ctVertices, U32 step, vector<float>& vertices) {
    		if(step != 4)
    			return -1;
    		if(step * ctVertices != vertices.size())
    			return -1;
        	/*
        	__kernel void ComputeFieldArrayStackLess(U32 ctVertices,
        								   __global float4* arrInHeader4,
        								   __global float4* arrInOps4,
        								   __global float4* arrInPrims4,
        								   __global float4* arrInMtxNodes4,
        								   __global float4* arrInOutVertexFields)
			*/

    		size_t arrLocalIndex[3];
    		size_t arrGlobalIndex[3];
    		arrGlobalIndex[0] = ctVertices;
    		arrGlobalIndex[1] = 0;
    		arrGlobalIndex[2] = 0;

    		ComputeKernel::ComputeLocalIndexSpace(1, m_lpKernelFieldArrayStackLess->getKernelWorkGroupSize(), arrLocalIndex);
    		ComputeKernel::ComputeGlobalIndexSpace(1, arrLocalIndex, arrGlobalIndex);

    		//Create Memory for all field points
    		U32 szVertexBuffer = sizeof(float) * step * ctVertices;
    		Buffer inoutMemFieldArray;
    		m_lpGPU->createMemBuffer(szVertexBuffer, PS::CL::memReadWrite, inoutMemFieldArray);
    		if(!m_lpGPU->enqueueWriteBuffer(inoutMemFieldArray.handle(), inoutMemFieldArray.size(), &vertices[0]))
    			return -1;

    		m_lpKernelFieldArrayStackLess->setArg(0, sizeof(U32), (void *)&ctVertices);
    		m_lpKernelFieldArrayStackLess->setArg(1, sizeof(cl_mem), m_inMemHeader.cptr());
    		m_lpKernelFieldArrayStackLess->setArg(2, sizeof(cl_mem), m_inMemOps.cptr());
    		m_lpKernelFieldArrayStackLess->setArg(3, sizeof(cl_mem), m_inMemPrims.cptr());
    		m_lpKernelFieldArrayStackLess->setArg(4, sizeof(cl_mem), m_inMemMtx.cptr());
    		m_lpKernelFieldArrayStackLess->setArg(5, sizeof(cl_mem), inoutMemFieldArray.cptr());

    		// Execute the kernel over the vector using the
    		// maximum number of work group items for this device
    		m_lpGPU->enqueueNDRangeKernel(m_lpKernelFieldArrayStackLess, 1, arrGlobalIndex, arrLocalIndex);
    		m_lpGPU->enqueueReadBuffer(inoutMemFieldArray.handle(), szVertexBuffer, &vertices[0]);

    		// Wait for all commands to complete
    		m_lpGPU->finishAllCommands();

    		return ctVertices;
        }


    }
}
