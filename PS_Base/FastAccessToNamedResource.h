#ifndef FASTACCESSTONAMEDRESOURCE_H
#define FASTACCESSTONAMEDRESOURCE_H


#include <string>
#include <map>
//#include <hash_map.h>
#include <tr1/unordered_map>
#include "PS_MathBase.h"

using namespace __gnu_cxx;
using namespace std;

namespace PS {

//Logging Policies
struct NoLogging {
    static void LogArg1(const char* message, const char* arg1) {
        PS_UNUSED(message);
        PS_UNUSED(arg1);
    }
};

struct Logging {
    static void LogArg1(const char* message, const char* arg1);
};

//ResourceType Policies
template <typename T>
struct TypeValue {
    typedef T Resource;
};

template <typename T>
struct TypePointer {
    typedef T* Resource;
};

//ResouceCreator Policy
template <typename T>
struct InsertRemoveNoop {
    static void Insert(T element) {
        PS_UNUSED(element);
    }

    static void Remove(T element) {
        PS_UNUSED(element);
    }
};


template <typename T>
struct NoopInsertRemoveBySafeDelete {
    static void Insert(T element) {
        PS_UNUSED(element);
    }

    static void Remove(T element) {
        SAFE_DELETE(element);
    }
};


//Provides named access to resources
template <typename T,
          template <class> class PolicyResourceType = TypePointer,
          template <class> class PolicyResourceInsertRemove = NoopInsertRemoveBySafeDelete,
          class PolicyErrorLogging = Logging>
class FastAccessNamedResource
{

public:
    FastAccessNamedResource() { }

    virtual ~FastAccessNamedResource() {
        cleanup();
    }

    //Convenient types
    typedef typename PolicyResourceType<T>::Resource Resource;
    typedef typename std::tr1::unordered_map<string, Resource >::iterator ITER;
    typedef typename std::tr1::unordered_map<string, Resource >::const_iterator CONST_ITER;

    /*!
     * \brief has checks weather a named resource is present in the collection
     * \param name the title of the resource
     * \return true if the item is found in the collection
     */
    bool has(const char* name) const {
        return (m_hash.find(string(name)) != m_hash.end());
    }

    /*!
     * \brief get
     * \param name
     * \return
     */
    Resource get(const char* name) const {
        CONST_ITER it = m_hash.find(string(name));
        if(it != m_hash.end())
            return it->second;
        else {
            PolicyErrorLogging::LogArg1("Requested resource not found! name = %s", name);
            return -1;
        }
    }

    /*!
     * \brief add will add an element into the hashmap
     * \param element
     * \param name
     */
    void add(Resource element, const char* name) {
        PolicyResourceInsertRemove<Resource>::Insert(element);
        m_hash.insert(std::make_pair<string, Resource>(string(name), element));
    }

    //Clear the list
    void cleanup() {
        for(ITER it = m_hash.begin(); it != m_hash.end(); it++) {
            PolicyResourceInsertRemove<Resource>::Remove(it->second);
        }
        m_hash.clear();
    }

protected:
     std::tr1::unordered_map<string, Resource > m_hash;
};

}

#endif // FASTACCESSTONAMEDRESOURCE_H
