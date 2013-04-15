#pragma once
#ifndef CGLSHADER_H
#define CGLSHADER_H

#include "../PS_Base/PS_String.h"
#include <vector>

#define GL_SUCCESS		   1
#define ERR_SHADER_FILE_NOT_FOUND -1
#define ERR_SHADER_COMPILE_ERROR  -2
#define ERR_SHADER_LINK_ERROR	  -3

namespace PS{

namespace SHADER{

//An OpenGL shader program
class GLShaderProgram
{
public:
    GLShaderProgram();
    GLShaderProgram(const char* pszVertexShader, const char* pszFragmentShader);
    virtual ~GLShaderProgram();

    enum ShaderType {stVertex = 0, stFragment = 1, stGeometry = 2};
    enum VARUSAGE {vuAttribute, vuUniform};
    enum VARPRECISION {vpLow, vpMedium, vpHigh, vpUndefined};
    struct VARPROP
    {
        DAnsiStr strName;
        DAnsiStr strType;
        VARUSAGE usage;
        VARPRECISION precision;
        int idxLocation;
    };

    bool init();

    //After analysis is done user can access the variables found in the shader code
    bool	 getAttributeVar(const char* chrAttribName, VARPROP& outAttribProp);
    bool	 getUniformVar(const char* chrUniformName, VARPROP& outUniformProp);

    //After analysis is done we fetch the locations of uniforms
    int		 getUniformLocation(const char* chrUniformName);
    bool 	 getAllUniformLocationsFromContext();


    //After analysis is done user can get the location where attribs set Automatically
    int 	 getAttribLocation(const char* chrAttribName);
    bool 	 setAttribLocation(const char* chrAttribName, int idxLoc);

    //Count Attributes and Uniforms
    int 	 countAttributeVars() const {return (int)m_vAttribs.size();}
    int 	 countUniformVars() const {return (int)m_vUniforms.size();}

    DAnsiStr getVertexShaderFilePath() const {return m_strVertexShaderFile;}
    void	 setVertexShaderFilePath(const DAnsiStr& strFN) {m_strVertexShaderFile = strFN;}

    DAnsiStr getFragmentShaderFilePath() const {return m_strFragmentShaderFile;}
    void	 setFragmentShaderFilePath(const DAnsiStr& strFN) {m_strFragmentShaderFile = strFN;}

    U32 getShaderProgram() const { return m_uiProgramObject;}
    U32 getVertexShader() const { return m_uiVertexShader;}
    U32 getFragmentShader() const { return m_uiFragShader;}

    //Status
    bool isRunning() const {return m_bIsRunning;}
    bool isAnalyzed() const {return m_bAnalyzed;}
    bool isCompiled() const {return m_bCompiled;}
    bool isReadyToRun();
    bool isBinaryShaderSupported() const {return m_bBinaryShaderSupported;}

    /*
  * Opens the text file containing the shader code and reads the content for both
  * Vertex Shader and Fragment Shader programs.
  * @param strVertexShaderFP File path to the vertex shader code
  * @param strFragmentShaderFP File path to the fragment shader program
  * @return 1 if shader compiles successfully
  */
    int	 compileFromFile(const DAnsiStr& strVertexShaderFP, const DAnsiStr& strFragmentShaderFP);


    /*
  * Compiles shader code and performs an analysis on the number variables defined.
  * Reports compile and link errors. This function attaches all attribute variables to
  * their locations based on their position in the list (i.e. First Found first position)
  * All Found attributes are added to attributes list and all uniforms are being put in the
  * uniforms list.
  * @param vShaderCode string containing Vertex Shader code.
  * @param vFragmentCode string containing Fragment Shader code.
  * @return 1 if successfully compile and link the shader program
  */
    int  compileCode(const char* vShaderCode, const char* vFragmentCode);

    //Load and Save Binary Shaders
    static bool IsGLExtensionSupported(const char *extension);


    bool loadBinaryProgram(const char* Filename, U32 &ProgramObjectID);
    bool saveBinaryProgram(const char* Filename, U32 &ProgramObjectID);


    /*
  * Find all uniforms and attributes in the vertex and fragment
  * shaders.
  */
    bool analyze();

    //Program run
    bool run();

    //Program stop
    void stop();

private:
    bool removeAllCppComments(DAnsiStr& strCode);
    bool readShaderCode(const DAnsiStr& strFilePath, DAnsiStr& strCode);

    DAnsiStr m_strVertexShaderFile;
    DAnsiStr m_strFragmentShaderFile;

    DAnsiStr m_strVertexShaderCode;
    DAnsiStr m_strFragmentShaderCode;
    U32   m_uiProgramObject;
    U32   m_uiVertexShader;
    U32   m_uiFragShader;


    bool   m_bIsRunning;
    bool   m_bCompiled;
    bool   m_bAnalyzed;
    bool   m_bBinaryShaderSupported;

    std::vector<VARPROP>   m_vAttribs;
    std::vector<VARPROP>   m_vUniforms;
};

}
}
#endif


