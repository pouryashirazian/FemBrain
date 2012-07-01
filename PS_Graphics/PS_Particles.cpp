#include "PS_Particles.h"
#include <GL/glew.h>

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

		Particles::Particles(const U32 maxParticles):m_maxParticles(maxParticles)
		{
			setup();
		}

		Particles::~Particles()
		{

		}

		void Particles::setup()
		{
		}


		void Particles::render()
		{
		}

		void Particles::animate()
		{

		}
	}

}