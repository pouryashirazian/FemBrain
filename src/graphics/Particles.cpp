#include "Particles.h"
#include "base/Vec.h"
#include "base/Matrix.h"
#include "base/FileDirectory.h"

using namespace PS::FILESTRINGUTILS;

namespace PS {
	namespace SG {

		//Particles Setup and Animation
		Particles::Particles(const U32 maxParticles):m_maxParticles(maxParticles)
		{
			setup();
		}

		Particles::~Particles()
		{
			cleanup();
		}

		void Particles::cleanup() {
			SAFE_DELETE(m_lpDevice);
			m_vPos.resize(0);
			m_vVel.resize(0);
			m_vColor.resize(0);
		}

		//Setup Kernels for running
		bool Particles::setup()
		{
			//Fill and initialize
			m_vPos.resize(4 * m_maxParticles);
			m_vVel.resize(4 * m_maxParticles);
			m_vColor.resize(4 * m_maxParticles);

			//Loop over particles
			for(U32 i=0; i < m_maxParticles; i++)
			{							
				float angle = DEGTORAD(RandRangeT<float>(0.0f, 360.0f));				
				vec3f v;
				if(i % 3 == 0)
					v = vec3f(0.0f, cos(angle), sin(angle));
				else if(i % 3 == 1)
					v = vec3f(cos(angle), 0.0f, sin(angle));
				else if(i % 3 == 2)
					v = vec3f(0.0f, cos(angle), sin(angle));
				v.normalize();

				U32 i4 = i*4;
				m_vPos[i4] = 0.0f;
				m_vPos[i4 + 1] = 0.0f;
				m_vPos[i4 + 2] = 0.0f;
				m_vPos[i4 + 3] = 1.0f;

				m_vVel[i4] = v.x;
				m_vVel[i4 + 1] = v.y;
				m_vVel[i4 + 2] = v.z;
				m_vVel[i4 + 3] = 1.0f;

				m_vColor[i4] = 1.0f;
				m_vColor[i4 + 1] = 0.0f;
				m_vColor[i4 + 2] = 0.0f;
				m_vColor[i4 + 3] = 0.4f;
			}

			//OpenCL
			AnsiStr strCodePath = ExtractOneLevelUp(ExtractFilePath(GetExePath()));
			AnsiStr strParticleFP = strCodePath + AnsiStr("PS_Shaders/ParticleSystem.cl");

			//Create compute device
			m_lpDevice = TheCLManager::Instance().device();
			int prg = m_lpDevice->addProgramFromFile(strParticleFP, true);
			assert(prg >= 0);

			//Kernel: particleMove
			m_lpKernelMove = m_lpDevice->addKernel(prg, "particleMove");
			assert(m_lpKernelMove != NULL);

			//MEM Buffer
			m_inoutMemVelocity = m_lpDevice->createMemBuffer(sizeof(float) * m_vVel.size(), PS::CL::memReadWrite);

			//Mesh Attributes
			this->setupVertexAttribs(m_vPos, 4, mbtPosition);
			this->setupVertexAttribs(m_vColor, 4, mbtColor);
			this->setFaceMode(ftPoints);

			return true;
		}


		void Particles::timestep()
		{
			if(!m_isValidVertex || !m_isValidColor)
				return;

			const float dt = 0.01f;

			/*
			vec3f pos;
			vec3f vel;
			for(U32 i=0; i<m_maxParticles; i++) {

				U32 i4 = i * 4;
				m_vPos[i4] += dt * m_vVel[i4];
				m_vPos[i4 + 1] += dt * m_vVel[i4 + 1];
				m_vPos[i4 + 2] += dt * m_vVel[i4 + 2];

				pos = vec3f(&m_vPos[i4]);
				vel = vec3f(&m_vVel[i4]);

				if(pos.x > 1.0f)
					vel = vel * (-1.0f);
				else if(pos.x < -1.0f)
					vel = vel * (-1.0f);

				if(pos.y > 1.0f)
					vel = vel * (-1.0f);
				else if(pos.y < -1.0f)
					vel = vel * (-1.0f);

				if(pos.z > 1.0f)
					vel = vel * (-1.0f);
				else if(pos.z < -1.0f)
					vel = vel * (-1.0f);

				vel.store(&m_vVel[i4]);
			}

			glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
			glBufferData(GL_ARRAY_BUFFER, m_vPos.size() * sizeof(float), &m_vPos[0], GL_DYNAMIC_DRAW);

			glBindBuffer(GL_ARRAY_BUFFER, m_vboColor);
			glBufferData(GL_ARRAY_BUFFER, m_vColor.size() * sizeof(float), &m_vColor[0], GL_DYNAMIC_DRAW);
	*/



			//Solution space
			size_t arrLocalIndex[3];
			size_t arrGlobalIndex[3];
			arrGlobalIndex[0] = m_maxParticles;
			arrGlobalIndex[1] = 0;
			arrGlobalIndex[2] = 0;
			ComputeKernel::ComputeLocalIndexSpace(3, m_lpKernelMove->getKernelWorkGroupSize(), arrLocalIndex);
			ComputeKernel::ComputeGlobalIndexSpace(3, arrLocalIndex, arrGlobalIndex);

			cl_mem inoutMemVertexBuffer = m_lpDevice->createMemBufferFromGL(m_gmbVertex.handle(), PS::CL::memReadWrite);
			cl_mem inoutMemColorBuffer = m_lpDevice->createMemBufferFromGL(m_gmbColor.handle(), PS::CL::memReadWrite);
			m_lpDevice->enqueueAcquireGLObject(1, &inoutMemVertexBuffer);
			m_lpDevice->enqueueAcquireGLObject(1, &inoutMemColorBuffer);


			/*
			__kernel void particleMove(__global float4* arrInOutPosition,
									   __global float4* arrInOutVelocity,
									   __global float4* arrInMeshColor,
									   const unsigned int count,
									   const float deltaT)
			*/
			m_lpKernelMove->setArg(0, sizeof(cl_mem), &inoutMemVertexBuffer);
			m_lpKernelMove->setArg(1, sizeof(cl_mem), &m_inoutMemVelocity);
			m_lpKernelMove->setArg(2, sizeof(cl_mem), &inoutMemColorBuffer);
			m_lpKernelMove->setArg(3, sizeof(U32), &m_maxParticles);
			m_lpKernelMove->setArg(4, sizeof(float), &dt);
			m_lpDevice->enqueueNDRangeKernel(m_lpKernelMove, 1, arrGlobalIndex, arrLocalIndex);

			m_lpDevice->enqueueReleaseGLObject(1, &inoutMemVertexBuffer);
			m_lpDevice->enqueueReleaseGLObject(1, &inoutMemColorBuffer);

			m_lpDevice->finishAllCommands();

			clReleaseMemObject(inoutMemVertexBuffer);
			clReleaseMemObject(inoutMemColorBuffer);

		}
	}

}
