#include "PS_GLSurface.h"
#include "PS_Logger.h"
#include <GL/glew.h>

using namespace PS::RASTER;

GLSurface::GLSurface()
{
	init(DEFAULT_SURFACE_WIDTH, DEFAULT_SURFACE_HEIGHT);
}

GLSurface::GLSurface(U32 w, U32 h)
{
	init(w, h);
}

GLSurface::~GLSurface()
{
	cleanup();
}

void GLSurface::cleanup()
{
	//Detach Buffers
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

	//Cleanup by deleting all buffers
	glDeleteFramebuffers(1, &m_glFBO);
	glDeleteTextures(1, &m_glTexture);
}

/*!
*  Initialize the Surface
*/
void GLSurface::init(U32 w, U32 h)
{
	LogInfoArg2("Init surface with dimensions [%d, %d]", w, h);

	m_width = w;
	m_height = h;

	//Create a texture handle
	glGenTextures(1, &m_glTexture);

	//Bind Texture
	glBindTexture(GL_TEXTURE_2D, m_glTexture);

	//Create the texture
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);


	//Create an openGL framebuffer object
	glGenFramebuffers(1, &m_glFBO);

	//Bind FrameBuffer to pipeline
	glBindFramebuffer(GL_FRAMEBUFFER, m_glFBO);

	//Attach 2D Texture to render 
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
						   GL_TEXTURE_2D, m_glTexture, 0);

	//Render Buffer
	//Create Render Buffer for Depth Testing
	glGenRenderbuffers(1, &m_glRBO);
	glBindRenderbuffer(GL_RENDERBUFFER, m_glRBO);

	//Specify width and height for storage of render buffer
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_width, m_height);

	//Attach Render Buffer to Frame Buffer
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_glRBO);
}


//Attach to surface
void GLSurface::attach()
{
	glBindFramebuffer(GL_FRAMEBUFFER, m_glFBO);
	glBindRenderbuffer(GL_RENDERBUFFER, m_glRBO);
}

//Detach from surface
void GLSurface::detach()
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
}

void GLSurface::test()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	glLoadIdentity();
	glOrtho(-2.0f, 2.0f, -2.0f, 2.0f, -1.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	glClearColor(1.0, 0.0f, 0.0f, 1.0f);

	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_TRIANGLES);
		glVertex3f(-1.0f, 0.0f, 0.0f);
		glVertex3f(1.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 1.0f, 0.0f);
	glEnd();

	glFlush();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

//Save the framebuffer output as a ppm
void GLSurface::saveAsPPM(const char* lpFilePath)
{
	CPixelMap* lpMap = new CPixelMap(m_width, m_height);
	
	//Saving to ppm
	glReadPixels(0, 0, m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)lpMap->buffer());
	lpMap->save(lpFilePath);

	SAFE_DELETE(lpMap);
}

void GLSurface::drawAsQuad()
{
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, m_glTexture);
	glBegin(GL_QUADS);
		glVertex2f(0.0f, 0.0f);
		glTexCoord2f(0.0f, 0.0f);

		glVertex2f(1.0f, 0.0f);
		glTexCoord2f(1.0f, 0.0f);

		glVertex2f(1.0f, 1.0f);
		glTexCoord2f(1.0f, 1.0f);

		glVertex2f(0.0f, 1.0f);
		glTexCoord2f(0.0f, 1.0f);

	glEnd();
}