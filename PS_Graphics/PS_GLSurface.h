/*!
* When programming for GPGPU there are times when you need to pass the result one rendering step to
* the next step. GLSurface is an interface for rendering to a texture
*/
#include "../PS_Base/PS_MathBase.h"
#include "PS_PixelMap.h"
//#include <GL/glew.h>

#define DEFAULT_SURFACE_WIDTH 512
#define DEFAULT_SURFACE_HEIGHT 512
class GLSurface
{
public:
	GLSurface();
	GLSurface(U32 w, U32 h);
	~GLSurface();

	void cleanup();

	//Attach surface to the render buffer target of the pipeline
	void attach();

	//Detach surface from the render buffer target
	void detach();

	//
	void test();

	void saveAsPPM(const char* lpFilePath);

	/*!
	*  Draws the output texture as a quad
	*/
	void drawAsQuad();

private:
	void init(U32 w, U32 h);
	void initTextures(U32 w, U32 h);

private:
	U32 m_glTex[2];
	U32 m_glFBO[2];
	U32 m_glRBO;

	U32 m_width;
	U32 m_height;
};

