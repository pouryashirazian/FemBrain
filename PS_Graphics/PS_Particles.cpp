#include "PS_Particles.h"
#include <GL/glew.h>

#include "../PS_Base/PS_FileDirectory.h"

using namespace PS::FILESTRINGUTILS;

namespace PS{
	namespace HPC{

		//Cleanup Mesh Buffer Objects
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

		//Particles Setup and Animation
		Particles::Particles(const U32 maxParticles):m_maxParticles(maxParticles)
		{
			setup();
		}

		Particles::~Particles()
		{
			SAFE_DELETE(m_lpDevice);
			m_vPos.resize(0);
			m_vVel.resize(0);
			m_vColor.resize(0);
		}

		//Setup Kernels for running
		bool Particles::setup()
		{
			m_vPos.resize(3 * m_maxParticles);
			m_vVel.resize(3 * m_maxParticles);
			m_vColor.resize(3 * m_maxParticles);
			for(U32 i=0; i < m_maxParticles; i++)
			{							
				float angle = DEGTORAD(RandRangeT<float>(0.0f, 360.0f));				
				svec3f v;
				if(i % 3 == 0)
					v = svec3f(0.0f, cos(angle), sin(angle));
				else if(i % 3 == 1)
					v = svec3f(cos(angle), 0.0f, sin(angle));
				else if(i % 3 == 2)
					v = svec3f(0.0f, cos(angle), sin(angle));
				vnormalize3f(v);

				U32 i3 = i*3;
				m_vVel[i3] = v.x; 
				m_vVel[i3 + 1] = v.y; 
				m_vVel[i3 + 2] = v.z;

				m_vPos[i3] = RandRangeT<float>(-1.0f, 1.0f);
				m_vPos[i3 + 1] = RandRangeT<float>(-1.0f, 1.0f);
				m_vPos[i3 + 2] = RandRangeT<float>(-1.0f, 1.0f);

				m_vColor[i3] = 1.0f;
				m_vColor[i3 + 1] = 0.0f;
				m_vColor[i3 + 2] = 0.0f;
			}

			//Buffers
			glGenBuffers(1, &m_meshBO.vboVertex);
			glBindBuffer(GL_ARRAY_BUFFER, m_meshBO.vboVertex);
			glBufferData(GL_ARRAY_BUFFER, m_maxParticles * 3 * sizeof(float), &m_vPos[0], GL_DYNAMIC_DRAW);

			glGenBuffers(1, &m_meshBO.vboColor);
			glBindBuffer(GL_ARRAY_BUFFER, m_meshBO.vboColor);
			glBufferData(GL_ARRAY_BUFFER, m_maxParticles * 3 * sizeof(float), &m_vColor[0], GL_DYNAMIC_DRAW);

			/*
			glGenBuffers(1, &m_meshBO.vboNormal);
			glBindBuffer(GL_ARRAY_BUFFER, m_meshBO.vboNormal);
			glBufferData(GL_ARRAY_BUFFER, ctVertices*3 * sizeof(float), arrNormals, GL_DYNAMIC_DRAW);


			glGenBuffers(1, &m_meshBO.iboFaces);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_meshBO.iboFaces);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, ctTriangles *3 * sizeof(U32), arrFaces, GL_DYNAMIC_DRAW);
			*/
			m_meshBO.bIsValid = true;

			return true;
			/*
			m_meshBO.bIsValid = false;
			m_lpDevice = new ComputeDevice(PS::HPC::ComputeDevice::dtGPU, true, "AMD");
			if(!m_lpDevice)
				return false;

			DAnsiStr strPath = ExtractOneLevelUp(ExtractFilePath(GetExePath()));
			strPath += DAnsiStr("PS_OpenCLKernels\\ParticleSystem.cl");

			ComputeProgram* lpProgram = m_lpDevice->addProgramFromFile(strPath.cptr());
			if(lpProgram == NULL)
			{
				SAFE_DELETE(m_lpDevice);
				return false;
			}


			//Add Kernel from Program
			m_lpKernelMove = lpProgram->addKernel("particleMove");

			//Input mem buffers
			cl_mem inMemPos;
			cl_mem inMemVelocity;
			cl_mem inMemColor;
			U32 szVec4 = sizeof(svec4f);
			inMemPos = m_lpDevice->createMemBuffer(sizeof(svec4f) * m_maxParticles, ComputeDevice::memReadOnly);
			inMemVelocity = m_lpDevice->createMemBuffer(sizeof(svec4f) * m_maxParticles, ComputeDevice::memReadOnly);
			inMemColor = m_lpDevice->createMemBuffer(sizeof(svec4f) * m_maxParticles, ComputeDevice::memReadOnly);		
		
			//Out Buffers
			glGenBuffers(1, &m_meshBO.vboVertex);
			glBindBuffer(GL_ARRAY_BUFFER, m_meshBO.vboVertex);
			glBufferData(GL_ARRAY_BUFFER, m_maxParticles * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
			cl_mem outMemMeshVertex = clCreateFromGLBuffer(m_lpDevice->getContext(), CL_MEM_WRITE_ONLY, m_meshBO.vboVertex, NULL);

			glGenBuffers(1, &m_meshBO.vboColor);
			glBindBuffer(GL_ARRAY_BUFFER, m_meshBO.vboColor);
			glBufferData(GL_ARRAY_BUFFER, m_maxParticles * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
			cl_mem outMemMeshColor = clCreateFromGLBuffer(m_lpDevice->getContext(), CL_MEM_WRITE_ONLY, m_meshBO.vboColor, NULL);

			//Fill all values
			m_vPos.resize(m_maxParticles);
			m_vVel.resize(m_maxParticles);
			m_vColor.resize(m_maxParticles);
			for(U32 i=0; i < m_maxParticles; i++)
			{							
				float angle = DEGTORAD(RandRangeT<float>(0.0f, 360.0f));				
				svec3f v;
				if(i % 3 == 0)
					v = svec3f(0.0f, cos(angle), sin(angle));
				else if(i % 3 == 1)
					v = svec3f(cos(angle), 0.0f, sin(angle));
				else if(i % 3 == 2)
					v = svec3f(0.0f, cos(angle), sin(angle));
				vnormalize3f(v);
				m_vVel[i] = svec4f(v.x, v.y, v.z, 0.0f);
				m_vPos[i] = svec4f(0, 0, 0, 0);
				m_vColor[i] = svec4f(1, 0, 0, 1);
			}

			// Transfer the input vector into device memory.			
			//Position
			if(!m_lpDevice->enqueueWriteBuffer(inMemPos, sizeof(svec4f) * m_maxParticles, &m_vPos[0].x))
				return false;

			//Velocity
			if(!m_lpDevice->enqueueWriteBuffer(inMemVelocity, sizeof(svec4f) * m_maxParticles, &m_vVel[0].x))
				return false;

			//Color
			if(!m_lpDevice->enqueueWriteBuffer(inMemColor, sizeof(svec4f) * m_maxParticles, &m_vColor[0].x))
				return false;

			// Set the arguments to the compute kernel
			m_lpKernelMove->setArg(0, sizeof(cl_mem), &outMemMeshVertex);
			m_lpKernelMove->setArg(1, sizeof(cl_mem), &outMemMeshColor);
			m_lpKernelMove->setArg(2, sizeof(cl_mem), &inMemPos);
			m_lpKernelMove->setArg(3, sizeof(cl_mem), &inMemVelocity);
			m_lpKernelMove->setArg(4, sizeof(cl_mem), &inMemColor);
			m_lpKernelMove->setArg(5, sizeof(U32), &m_maxParticles);
			m_lpKernelMove->setArg(6, sizeof(float), &deltaT);

			//Acquire Mesh for Writing
			clEnqueueAcquireGLObjects(m_lpDevice->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
			clEnqueueAcquireGLObjects(m_lpDevice->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);

			size_t local;
			size_t global;
			// Get the maximum work group size for executing the kernel on the device
			cl_int err = clGetKernelWorkGroupInfo(m_lpKernelMove->getKernel(),
				m_lpDevice->getDevice(),
				CL_KERNEL_WORK_GROUP_SIZE,
				sizeof(local), &local, NULL);
			if (err != CL_SUCCESS) {
				cerr << "Error: Failed to retrieve kernel work group info! "
					<<  err << endl;
				exit(1);
			}

			// Execute the kernel over the vector using the
			// maximum number of work group items for this device
			global = m_maxParticles;
			err = clEnqueueNDRangeKernel(m_lpDevice->getCommandQ(), m_lpKernelMove->getKernel(),
				1, NULL, &global, &local,
				0, NULL, NULL);
			if (err) {
				cerr << "Error: Failed to execute kernel!" << endl;
				return false;
			}

			// Wait for all commands to complete
			m_lpDevice->finishAllCommands();

			//Release Mesh for Writing
			clEnqueueReleaseGLObjects(m_lpDevice->getCommandQ(), 1, &outMemMeshVertex, 0, 0, 0);
			clEnqueueReleaseGLObjects(m_lpDevice->getCommandQ(), 1, &outMemMeshColor, 0, 0, 0);

			m_meshBO.bIsValid = true;

			return true;
			*/
		}


		void Particles::render()
		{	
			if(!m_meshBO.bIsValid)	return;

			glBindBuffer(GL_ARRAY_BUFFER, m_meshBO.vboColor);
			glColorPointer(3, GL_FLOAT, 0, 0);
			glEnableClientState(GL_COLOR_ARRAY);

			glBindBuffer(GL_ARRAY_BUFFER, m_meshBO.vboVertex);
			glVertexPointer(3, GL_FLOAT, 0, 0);
			glEnableClientState(GL_VERTEX_ARRAY);

			//Draw Elements
			glDrawArrays(GL_POINTS, 0, m_maxParticles);

			glDisableClientState(GL_COLOR_ARRAY);
			glDisableClientState(GL_VERTEX_ARRAY);
		}

		void Particles::animate()
		{

		}
	}

}
