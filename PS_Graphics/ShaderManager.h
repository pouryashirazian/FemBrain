#ifndef SHADERMANAGER_H
#define SHADERMANAGER_H

#include <map>
#include <string>
#include <loki/Singleton.h>
#include "PS_Base/PS_MathBase.h"
#include "PS_Base/FastAccessToNamedResource.h"

using namespace std;
using namespace Loki;
using namespace PS;


void ReleaseShader(U32 shader);

template<typename T>
struct ShaderNoopInsertGLRemove {
    static void Insert(T element) {
        PS_UNUSED(element);
    }

    static void Remove(T element) {
        ReleaseShader(element);
    }
};

/*!
 * \brief The ShaderManager class is a collection for all shader programs
 * in a graphics app and provides:
 * 1.Named access to shaders
 * 2.Cleanup and releasing shaders
 * 3.Global Singleton access
 */
typedef FastAccessNamedResource<U32, TypeValue, ShaderNoopInsertGLRemove> ShaderManagerParent;

class ShaderManager : public ShaderManagerParent  {
public:
    ShaderManager();
    virtual ~ShaderManager();
    U32 add(const char* vertexShaderCode, const char* fragmentShaderCode, const char* name);
    U32 addFromFile(const char* chrVertexShaderPath,
    				 const char* chrFragShaderPath,
    				 const char* name = NULL);

//    void addFromFolder(const char* chrShadersPath);
};

typedef SingletonHolder<ShaderManager, CreateUsingNew, PhoenixSingleton> TheShaderManager;

#endif // SHADERMANAGER_H
