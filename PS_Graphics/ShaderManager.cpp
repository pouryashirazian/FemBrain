#include "ShaderManager.h"
#include "PS_GLFuncs.h"
#include "PS_Base/PS_Logger.h"
#include "PS_Base/PS_FileDirectory.h"
#include <GL/glew.h>

using namespace PS::FILESTRINGUTILS;

void ReleaseShader(U32 shader) {
    if(glIsProgram(shader))
        glDeleteProgram(shader);
}

ShaderManager::ShaderManager() {

}

ShaderManager::~ShaderManager() {
    ShaderManagerParent::cleanup();
}


U32 ShaderManager::add(const char* vertexShaderCode, const char* fragmentShaderCode, const char* name) {
    U32 shader = -1;
    if(CompileShaderCode(vertexShaderCode, fragmentShaderCode, shader))
        ShaderManagerParent::add(shader, name);

    return shader;
}

U32 ShaderManager::addFromFile(const char* chrVertexShaderPath,
								 const char* chrFragShaderPath,
								 const char* name) {
	char* lpVertShaderCode = NULL;
	char* lpFragShaderCode = NULL;
	if(!ReadShaderCode(chrVertexShaderPath, &lpVertShaderCode)) {
		LogErrorArg1("Unable to read vertex shader code at: %s", chrVertexShaderPath);
		return false;
	}

	if(!ReadShaderCode(chrFragShaderPath, &lpFragShaderCode)) {
		LogErrorArg1("Unable to read fragment shader code at: %s", chrFragShaderPath);
		return false;
	}


	DAnsiStr strTitle = (name != NULL) ? DAnsiStr(name) : ExtractFileTitleOnly(chrVertexShaderPath);
	return this->add(lpVertShaderCode, lpFragShaderCode, strTitle.cptr());
}

