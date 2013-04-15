#include <iostream>
#include <fstream>
#include "PS_ShaderGLSL.h"
#include "GL/glew.h"
#include "../PS_Base/PS_FileDirectory.h"
#include "../PS_Base/PS_ErrorManager.h"

//#define SAVE_LOAD_BINARY_SHADER

#if !defined(WIN32)

#if defined(EGL_NOT_PRESENT) || (defined(__BADA__) && defined(_X86_)) // Bada simulator

#if defined(__PALMPDK__)
#include "SDL.h"
#define PSGetProcAddress(x) SDL_GLES_GetProcAddress(#x)
#else
#define PSGetProcAddress(x) NULL
#endif

#else

#if defined(__APPLE__) && defined (TARGET_OS_IPHONE)	//Binary shaders cannot be run on Apple Hardware. Training course will compile shaders every time.
#define PSGetProcAddress(x) ::x
#else
//#include <EGL/egl.h>
//#define PSGetProcAddress(x) eglGetProcAddress(#x)
#endif

#endif

#endif

#if defined(SAVE_LOAD_BINARY_SHADER) && !defined (TARGET_OS_IPHONE)
// Declares the binary program functions
PFNGLGETPROGRAMBINARYOESPROC				glGetProgramBinaryOES;
PFNGLPROGRAMBINARYOESPROC					glProgramBinaryOES;
#endif



using namespace PS::FILESTRINGUTILS;


namespace PS{

namespace SHADER{

GLShaderProgram::GLShaderProgram()
{
    init();
}

GLShaderProgram::GLShaderProgram(const char* pszVertexShader, const char* pszFragmentShader)
{
    init();
    compileCode(pszVertexShader, pszFragmentShader);
}

GLShaderProgram::~GLShaderProgram()
{
    m_vAttribs.resize(0);
    m_vUniforms.resize(0);

    if(glIsProgram(m_uiProgramObject))
    	glDeleteProgram(m_uiProgramObject);
    m_bCompiled = false;
    m_bIsRunning = false;
}


bool GLShaderProgram::init()
{
    m_bAnalyzed = false;
    m_bIsRunning = false;
    m_bCompiled = false;
    m_bBinaryShaderSupported = false;

    m_uiVertexShader = 0;
    m_uiFragShader = 0;
    m_uiProgramObject = 0;

    m_strVertexShaderFile = "inline.vsh";
    m_strFragmentShaderFile = "inline.fsh";

#ifdef USE_GLES
    //Checks if the program binary handling extension is supported.
    m_bBinaryShaderSupported = IsGLExtensionSupported("GL_OES_get_program_binary");

#if !defined (TARGET_OS_IPHONE)
    glGetProgramBinaryOES=0;
    glProgramBinaryOES=0;

    // Retrieves the functions needed to use the extension.
    if (m_bBinaryShaderSupported)
    {
        glGetProgramBinaryOES = (PFNGLGETPROGRAMBINARYOESPROC) PSGetProcAddress(glGetProgramBinaryOES);
        glProgramBinaryOES = (PFNGLPROGRAMBINARYOESPROC) PSGetProcAddress(glProgramBinaryOES);
    }
#endif
#else
    GLenum err = glewInit();
    if(err != GLEW_OK)
    {
        DAnsiStr strError = printToAStr("GLEW init error: %s", glewGetErrorString(err));
        ReportError(strError.ptr());
        return false;
    }
#endif


    return true;
}


//Reads the shader code from file and compiles
int GLShaderProgram::compileFromFile(const DAnsiStr& strVertexShaderFP, const DAnsiStr& strFragmentShaderFP)
{
    if((strVertexShaderFP.length() == 0)||(strFragmentShaderFP.length() == 0))
        return ERR_SHADER_FILE_NOT_FOUND;

    m_strVertexShaderFile = strVertexShaderFP;
    m_strFragmentShaderFile = strFragmentShaderFP;

    //Read both files
    bool bres = readShaderCode(m_strVertexShaderFile, m_strVertexShaderCode);
    if(!bres) return ERR_SHADER_FILE_NOT_FOUND;
    bres = readShaderCode(m_strFragmentShaderFile, m_strFragmentShaderCode);
    if(!bres) return ERR_SHADER_FILE_NOT_FOUND;

    return compileCode(m_strVertexShaderCode.ptr(), m_strFragmentShaderCode.ptr());
}

int GLShaderProgram::compileCode(const char* vShaderCode, const char* vFragmentCode)
{
    // Create the fragment shader object
    m_uiFragShader = glCreateShader(GL_FRAGMENT_SHADER);

    // Load the source code into it
    glShaderSource(m_uiFragShader, 1, (const char**)&vFragmentCode, NULL);

    // Compile the source code
    glCompileShader(m_uiFragShader);

    // Check if compilation succeeded
    GLint bShaderCompiled;
    glGetShaderiv(m_uiFragShader, GL_COMPILE_STATUS, &bShaderCompiled);
    if (!bShaderCompiled)
    {
        // An error happened, first retrieve the length of the log message
        int i32InfoLogLength, i32CharsWritten;
        glGetShaderiv(m_uiFragShader, GL_INFO_LOG_LENGTH, &i32InfoLogLength);

        // Allocate enough space for the message and retrieve it
        char* pszInfoLog = new char[i32InfoLogLength];
        glGetShaderInfoLog(m_uiFragShader, i32InfoLogLength, &i32CharsWritten, pszInfoLog);

        /*
   Displays the message in a dialog box when the application quits
   using the shell PVRShellSet function with first parameter prefExitMessage.
  */
        char* pszMsg = new char[i32InfoLogLength+256];
        strcpy(pszMsg, "Failed to compile fragment shader: ");
        strcat(pszMsg, pszInfoLog);
        fprintf(stderr, pszMsg);

        delete [] pszMsg;
        delete [] pszInfoLog;
        return false;
    }

    // Loads the vertex shader in the same way
    m_uiVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(m_uiVertexShader, 1, (const char**)&vShaderCode, NULL);
    glCompileShader(m_uiVertexShader);
    glGetShaderiv(m_uiVertexShader, GL_COMPILE_STATUS, &bShaderCompiled);
    if (!bShaderCompiled)
    {
        int i32InfoLogLength, i32CharsWritten;
        glGetShaderiv(m_uiVertexShader, GL_INFO_LOG_LENGTH, &i32InfoLogLength);
        char* pszInfoLog = new char[i32InfoLogLength];
        glGetShaderInfoLog(m_uiVertexShader, i32InfoLogLength, &i32CharsWritten, pszInfoLog);
        char* pszMsg = new char[i32InfoLogLength+256];
        strcpy(pszMsg, "Failed to compile vertex shader: ");
        strcat(pszMsg, pszInfoLog);
        fprintf(stderr, pszMsg);


        delete [] pszMsg;
        delete [] pszInfoLog;
        return false;
    }

    // Create the shader program
    m_uiProgramObject = glCreateProgram();

    // Attach the fragment and vertex shaders to it
    glAttachShader(m_uiProgramObject, m_uiFragShader);
    glAttachShader(m_uiProgramObject, m_uiVertexShader);

    //Now that the shader is compiled we analyze it and find all
    //uniform and attribute variables
    m_bAnalyzed = analyze();

    // Link the program
    glLinkProgram(m_uiProgramObject);

    // Check if linking succeeded in the same way we checked for compilation success
    GLint bLinked;
    glGetProgramiv(m_uiProgramObject, GL_LINK_STATUS, &bLinked);

    if (!bLinked)
    {
        int i32InfoLogLength, i32CharsWritten;
        glGetProgramiv(m_uiProgramObject, GL_INFO_LOG_LENGTH, &i32InfoLogLength);
        char* pszInfoLog = new char[i32InfoLogLength];
        glGetProgramInfoLog(m_uiProgramObject, i32InfoLogLength, &i32CharsWritten, pszInfoLog);

        char* pszMsg = new char[i32InfoLogLength+256];
        strcpy(pszMsg, "Failed to link program: ");
        strcat(pszMsg, pszInfoLog);
        //PVRShellSet(prefExitMessage, pszMsg);
        //cout << pszMsg;

        delete [] pszMsg;
        delete [] pszInfoLog;
        return false;
    }

    if(m_bAnalyzed)
        getAllUniformLocationsFromContext();

    m_bCompiled = true;
    return true;
}

bool GLShaderProgram::run()
{
    if(!isReadyToRun())
        return false;

    //Run program
    glUseProgram(m_uiProgramObject);

    m_bIsRunning = true;
    return true;
}

void GLShaderProgram::stop()
{
    glUseProgram(0);
    m_bIsRunning = false;
}

bool GLShaderProgram::removeAllCppComments(DAnsiStr& strCode)
{
    DAnsiStr strOutput;

    char ch1, ch2;
    bool bAppend = true;
    bool bSingleLineComment = false;

    strCode.replaceChars('\t', ' ');
    size_t i=0;
    while(i < (strCode.length() - 1))
    {
        ch1 = strCode[i];
        ch2 = strCode[i+1];
        if((ch1 == '/')&&(ch2 == '/'))
        {
            //Line Comment
            bAppend = false;
            bSingleLineComment = true;
        }
        else if((ch1 == '/')&&(ch2 == '*'))
        {
            bAppend = false;
            bSingleLineComment = false;
        }


        if(bAppend)
        {
            strOutput.appendFromT(ch1);
            if(i == strCode.length() - 2)
                strOutput.appendFromT(ch2);
        }
        else
        {
            if(bSingleLineComment)
            {
                if((ch1 == '\r')&&(ch2 == '\n'))
                {
                    bAppend = true;
                    i++;
                }
            }
            else
            {
                if((ch1 == '*')&&(ch2 == '/'))
                {
                    bAppend = true;
                    i++;
                }
            }
        }

        i++;
    }//End While

    strCode = strOutput;

    return true;
}

bool GLShaderProgram::analyze()
{
    if(m_strVertexShaderCode.length() == 0)
        return false;
    if(m_strFragmentShaderCode.length() == 0)
        return false;

    m_vAttribs.resize(0);
    m_vUniforms.resize(0);

    std::vector<DAnsiStr> vLines;
    std::vector<DAnsiStr> vContent;
    DAnsiStr strLine, strWord;
    VARPROP var;
    int step = 0;
    int ctDetected = 0;

    DAnsiStr strCode = m_strVertexShaderCode;
    //Need to replace tab with space
    removeAllCppComments(strCode);

    //Append all lines of code
    if(strCode.decompose(';', vContent) > 0)
    {
        for(size_t i=0; i<vContent.size(); i++)
            vLines.push_back(vContent[i]);
    }
    //PS::FILESTRINGUTILS::WriteTextFile(m_strVertexShaderFile + ".txt", vLines);
    vContent.resize(0);

    /////////////////////////////////////////////////////////////////////////////////////
    strCode = m_strFragmentShaderCode;
    removeAllCppComments(strCode);

    if(strCode.decompose(';', vContent) > 0)
    {
        for(size_t i=0; i<vContent.size(); i++)
            vLines.push_back(vContent[i]);
    }
    //PS::FILESTRINGUTILS::WriteTextFile(m_strFragmentShaderFile + ".txt", vLines);
    vContent.resize(0);


    //Now process line by line
    for(size_t i=0; i<vLines.size(); i++)
    {
        step = 0;
        strLine = vLines[i];
        //attribute highp vec4 myVertex;
        //attribute mediump vec4 myUV;
        //uniform mediump mat4 myMVPMatrix;
        //USAGE + PRECISION + TYPE + NAME
        int pos;

        while(strLine.length() > 0)
        {
            if(strLine.lfind(' ', pos))
            {
                strWord = strLine.substr(0, pos);
                strLine = strLine.substr(pos);
            }
            else
            {
                strWord = strLine;
                strLine.resize(0);
            }

            strWord.removeStartEndSpaces();
            strLine.removeStartEndSpaces();

            //See if this is a var line
            if(step == 0)
            {
                if(strWord == DAnsiStr("attribute"))
                    var.usage = vuAttribute;
                else if(strWord == DAnsiStr("uniform"))
                    var.usage = vuUniform;
                else
                    break;
            }
            else
            {
                if(step == 1)
                {
                    if(strWord == DAnsiStr("lowp"))
                        var.precision = vpLow;
                    else if(strWord == DAnsiStr("mediump"))
                        var.precision = vpMedium;
                    else if(strWord == DAnsiStr("highp"))
                        var.precision = vpHigh;
                    else
                    {
                        //Precision is optional so set type name here and increment step
                        var.precision = vpUndefined;
                        var.strType = strWord;
                        step++;
                    }
                }
                else if(step == 2)
                {
                    var.strType = strWord;
                }
                else if(step == 3)
                {
                    var.strName = strWord;
                    if(var.usage == vuAttribute)
                    {
                        //Set Attribute location
                        var.idxLocation = m_vAttribs.size();
                        glBindAttribLocation(m_uiProgramObject, var.idxLocation, var.strName.ptr());
                        m_vAttribs.push_back(var);
                    }
                    else
                    {
                        //Get assigned uniform location for this uniform variable
                        var.idxLocation = glGetUniformLocation(m_uiProgramObject, var.strName.ptr());
                        m_vUniforms.push_back(var);
                    }
                    ctDetected++;
                    break;
                }
            }

            step++;
        }
    }

    return true;
}


bool GLShaderProgram::readShaderCode(const DAnsiStr& strFilePath, DAnsiStr& strCode)
{
    std::ifstream fp;
    fp.open(strFilePath.ptr(), std::ios::binary);
    if(!fp.is_open())
        return false;

    size_t size;
    fp.seekg(0, std::ios::end);
    size = fp.tellg();
    fp.seekg(0, std::ios::beg);

    char * buf = new char[size+1];
    //Read file content
    fp.read(buf, size);
    buf[size] = '\0';

    strCode = DAnsiStr(buf);
    SAFE_DELETE(buf);
    fp.close();

    return true;
}

bool GLShaderProgram::isReadyToRun()
{
    GLboolean bValid = glIsProgram(m_uiProgramObject);
    return m_bCompiled && m_bAnalyzed && (bValid == GL_TRUE);
}

bool GLShaderProgram::getAllUniformLocationsFromContext()
{
    if(m_vUniforms.size() == 0)
        return false;

    for(size_t i=0; i<m_vUniforms.size(); i++)
    {
        m_vUniforms[i].idxLocation = glGetUniformLocation(m_uiProgramObject, m_vUniforms[i].strName.ptr());
    }
    return true;
}

int  GLShaderProgram::getUniformLocation(const char* chrUniformName)
{
    VARPROP var;
    if(getUniformVar(chrUniformName, var))
        return var.idxLocation;
    else
        return -1;
}

bool GLShaderProgram::setAttribLocation(const char* chrAttribName, int idxLoc)
{
    if(m_vAttribs.size() == 0)
        return false;

    for(size_t i=0; i<m_vAttribs.size(); i++)
    {
        if(m_vAttribs[i].strName == DAnsiStr(chrAttribName))
        {
            m_vAttribs[i].idxLocation = idxLoc;
            glBindAttribLocation(m_uiProgramObject, m_vAttribs[i].idxLocation, m_vAttribs[i].strName.ptr());
            return true;
        }
    }
    return false;
}

int GLShaderProgram::getAttribLocation(const char* chrAttribName)
{
    VARPROP var;
    if(getAttributeVar(chrAttribName, var))
        return var.idxLocation;
    else
        return -1;
}

bool GLShaderProgram::getAttributeVar(const char* chrAttribName, VARPROP& outAttribProp)
{
    if(m_vAttribs.size() == 0)
        return false;
    for(size_t i=0; i<m_vAttribs.size(); i++)
    {
        if(m_vAttribs[i].strName == DAnsiStr(chrAttribName))
        {
            outAttribProp = m_vAttribs[i];
            return true;
        }
    }
    return false;
}

bool GLShaderProgram::getUniformVar(const char* chrUniformName, VARPROP& outUniformProp)
{
    if(m_vUniforms.size() == 0)
        return false;
    for(size_t i=0; i<m_vUniforms.size(); i++)
    {
        if(m_vUniforms[i].strName == DAnsiStr(chrUniformName))
        {
            outUniformProp = m_vUniforms[i];
            return true;
        }
    }
    return false;
}


/*!***********************************************************************
@Function			IsGLExtensionSupported
@Input				extension extension to query for
@Returns			True if the extension is supported
@Description		Queries for support of an extension
*************************************************************************/
bool GLShaderProgram::IsGLExtensionSupported(const char *extension)
{
    // The recommended technique for querying OpenGL extensions;
    // from http://opengl.org/resources/features/OGLextensions/
    const GLubyte *extensions = NULL;
    const GLubyte *start;
    GLubyte *where, *terminator;

    // Extension names should not have spaces.
    where = (GLubyte *) strchr(extension, ' ');
    if (where || *extension == '\0')
        return 0;

    extensions = glGetString(GL_EXTENSIONS);

    // It takes a bit of care to be fool-proof about parsing the
    // OpenGL extensions string. Don't be fooled by sub-strings, etc.
    start = extensions;
    for (;;) {
        where = (GLubyte *) strstr((const char *) start, extension);
        if (!where)
            break;
        terminator = where + strlen(extension);
        if (where == start || *(where - 1) == ' ')
            if (*terminator == ' ' || *terminator == '\0')
                return true;
        start = terminator;
    }

    return false;
}

#ifdef SAVE_LOAD_BINARY_SHADER
/*!****************************************************************************
 @Function		saveBinaryProgram
 @Return		bool	True if save succeeded.
 @Description	This function takes as input the ID of a shader program object
    which should have been created prior to calling this function,
    as well as a filename to save the binary program to.
    The function will save out a file storing the binary shader
    program, and the enum value determining its format.
******************************************************************************/
bool GLShaderProgram::saveBinaryProgram(const char* Filename, GLuint &ProgramObjectID)
{
#if !defined (TARGET_OS_IPHONE)
    //Quick check to make sure that the program actually exists.
    GLint linked;
    glGetProgramiv(ProgramObjectID, GL_LINK_STATUS, &linked);
    if (!linked)
    {
        //Shaders not linked correctly, no binary to retrieve.
        return false;
    }

    // Get the length of the shader binary program in memory.
    // Doing this ensures that a sufficient amount of memory is allocated for storing the binary program you retrieve.
    GLsizei length=0;
    glGetProgramiv(ProgramObjectID,GL_PROGRAM_BINARY_LENGTH_OES,&length);

    // Pointer to the binary shader program in memory, needs to be allocated with the right size.
    GLvoid* ShaderBinary = (GLvoid*)malloc(length);

    // The format that the binary is retreived in.
    GLenum binaryFormat=0;

    // Error checking variable - this should be greater than 0 after glGetProgramBinaryOES, otherwise there was an error.
    GLsizei lengthWritten=0;

    // Get the program binary from GL and save it out.
    glGetProgramBinaryOES(ProgramObjectID,length,&lengthWritten,&binaryFormat,ShaderBinary);
    if (!lengthWritten)
    {
        // Save failed. Insufficient memory allocated to write binary shader.
        return false;
    }

    // Cache the program binary for future runs
    FILE* outfile = fopen(Filename, "wb");

    if(!outfile)
    {
        printf("Failed to open %s for writing to.\n", Filename);
        return false;
    }

    // Save the binary format.
    if(!fwrite((void*)&binaryFormat,sizeof(GLenum),1,outfile)) return false; // Couldn't write binary format to file.

    // Save the actual binary program.
    if(!fwrite(ShaderBinary, length,1,outfile)) return false;				 // Couldn't write binary data to file.

    // Close the file.
    fclose(outfile);

    // Free the memory used by Shader Binary.
    free(ShaderBinary);
    return true;
#else
    return false;
#endif
}


/*!****************************************************************************
 @Function		loadBinaryProgram
 @Return		bool	True if load succeeded.
 @Description	This function takes as input the ID of a shader program object
    which should have been created prior to calling this function,
    as well as a filename to load the binary program from.
    The function will load in a file storing the binary shader
    program, and the enum value determining its format.
    It will then load the binary into memory.

 @Note:			This function is not able to check if the shaders have changed.
    If you change the shaders then the file this saves out either
    needs to be deleted	or a new file used.
******************************************************************************/
bool GLShaderProgram::loadBinaryProgram(const char* Filename, GLuint &ProgramObjectID)
{
#if !defined (TARGET_OS_IPHONE)
    // Open the file.
    FILE* infile = fopen(Filename, "rb");

    // File open failed, either doesn't exist or is empty.
    if (!infile) return false;

    // Find initialise the shader binary.
    fseek(infile, 0, SEEK_END);
    GLsizei length = (GLint)ftell(infile)-sizeof(GLenum);

    if (!length) return false;	// File appears empty.

    // Allocate a buffer large enough to store the binary program.
    GLvoid* ShaderBinary = (GLvoid*)malloc(length);

    // Read in the binary format
    GLenum format=0;
    fseek(infile, 0, SEEK_SET);
    fread(&format, sizeof(GLenum), 1, infile);

    // Read in the program binary.
    fread(ShaderBinary, length, 1, infile);
    fclose(infile);

    // Create an empty shader program
    ProgramObjectID = glCreateProgram();

    // Load the binary into the program object -- no need to link!
    glProgramBinaryOES(ProgramObjectID, format, ShaderBinary, length);

    // Delete the binary program from memory.
    free(ShaderBinary);

    // Check that the program was loaded correctly, uses the same checks as when linking with a standard shader.
    GLint loaded;
    glGetProgramiv(ProgramObjectID, GL_LINK_STATUS, &loaded);
    if (!loaded)
    {
        // Something must have changed. Need to recompile shaders.
        int i32InfoLogLength, i32CharsWritten;
        glGetProgramiv(ProgramObjectID, GL_INFO_LOG_LENGTH, &i32InfoLogLength);
        char* pszInfoLog = new char[i32InfoLogLength];
        glGetProgramInfoLog(ProgramObjectID, i32InfoLogLength, &i32CharsWritten, pszInfoLog);
        char* pszMsg = new char[i32InfoLogLength+256];
        strcpy(pszMsg, "Failed to load binary program: ");
        strcat(pszMsg, pszInfoLog);
        printf(pszMsg);

        delete [] pszMsg;
        delete [] pszInfoLog;
        return false;
    }
    return true;
#else
    return false;
#endif
}

#endif
}
}
