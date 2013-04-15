#include "ShaderManager.h"
#include "PS_GLFuncs.h"
#include "PS_Base/PS_Logger.h"
#include <GL/glew.h>

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
