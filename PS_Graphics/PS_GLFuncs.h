/*
 * PS_GLFuncs.h
 *
 *  Created on: 2011-12-12
 *      Author: pourya
 */

#ifndef PS_GLFUNCS_H_
#define PS_GLFUNCS_H_

#include <string>
#include "PS_MathBase.h"


using namespace std;

//Global Funcs for OpenGL

/*!
 * A Generic OpenGL initialization.
 */
void InitGL();

/*!
 * Write 2D text using GLUT bitmap fonts.
 */
void DrawString(const char *str, int x, int y, float color[4], void *font);

/*!
 * Write text in 3D
 */
void DrawString3D(const char *str, float pos[3], float color[4], void *font);

/*!
 * Checks the status of OpenGL FrameBuffer.
 * @return true if there was no error otherwise false.
 */
bool CheckFramebufferStatus();

/*!
 * Prints information of the framebuffer.
 */
void PrintFramebufferInfo();

/*!
 * Reads a shader code from file and stores as as a string.
 * @return true if the shader code has been read successfully.
 * @param lpChrFilePath Path to the shader code.
 * @param lpSourceCode The string containing the source code
 */
bool ReadShaderCode(const char* lpChrFilePath,
					char** lpSourceCode);

/*!
 * Compiles shader code and build a program object.
 * @vShaderCode string containing vertex shader code
 * @vFragmentCode string containing fragment shader code
 * @uiOutProgramObj The id assigned to the program object after building the executable.
 * @return true if the compilation done successfully.
 */
bool CompileShaderCode(const char* vShaderCode,
					   const char* vFragmentCode,
					   U32& uiOutProgramObj);


#endif /* PS_GLFUNCS_H_ */
