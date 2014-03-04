#ifndef PS_COMPUTEDEVICE_H
#define PS_COMPUTEDEVICE_H
#include <vector>
#include <string>
#include "base/MathBase.h"
#include "base/String.h"
#include "base/FastAccessToNamedResource.h"
#include "selectcl.h"

using namespace std;

namespace PS {
namespace CL {
    
    enum MEMACCESSMODE{memReadWrite = CL_MEM_READ_WRITE, memReadOnly = CL_MEM_READ_ONLY, memWriteOnly = CL_MEM_WRITE_ONLY};
    
    
    /*!
     * A Memory Buffer for Kernels
     */
    class Buffer {
    public:
        Buffer();
        Buffer(const Buffer& rhs);
        Buffer(cl_mem clMem,
        	   U32 size = sizeof(cl_mem),
        	   MEMACCESSMODE access = memReadWrite,
        	   bool isSharedGL = false);

        virtual ~Buffer();
        void release();

        //Setter
        void set(cl_mem clMem, U32 size, MEMACCESSMODE access, bool isSharedGL);

        //Getters
        cl_mem handle() const {return m_clMem;}
        U32 size() const {return m_size;}
        cl_mem* ptr() {return &m_clMem;}
        const cl_mem* cptr() const {return &m_clMem;}
        bool isValid() const { return (m_clMem != NULL);}
        
        MEMACCESSMODE access() const {return m_access;}
        bool isSharedWithGL() const {return m_isSharedWithGL;}

        Buffer& operator=(const Buffer& rhs);
    protected:
        void init();
        
    private:
        U32 m_size;
        cl_mem m_clMem;
        MEMACCESSMODE m_access;
        bool m_isSharedWithGL;
    };
    


/*!s
  * A Kernel is a simple function to run on device. Kernel runs can communicate through share memory on compute device only.
  * The amount of memory available to a kernel function is limited. 
  */
class ComputeKernel
{
public:
    ComputeKernel(cl_kernel kernel, const string& strTitle);
    virtual ~ComputeKernel();

    /*!
     * sets kernel argument at the specified index position.
     * @param idxArg index position of the argument
     * @param size Size of the argument
     * @lpArg pointer to the beginning of the argument to set.
     */
    bool setArg(U32 idxArg, U32 size, const void* lpArg);
    bool setArg(U32 idxArg, const Buffer* buf);
    cl_kernel getKernel() const {return m_clKernel;}
    std::string getTitle() const {return m_strTitle;}

    size_t getKernelWorkGroupSize() const {return m_szWorkGroupSize;}
    void setKernelWorkGroupSize(size_t szWorkgroup) { m_szWorkGroupSize = szWorkgroup;}


    //These functions help in computing indices
    static size_t ToMultipleOf(size_t N, size_t base)
    {
    	return (ceil((double)N / (double)base) * base);
    }

    static void ComputeLocalIndexSpace(int dim, size_t szKernelWorkGroup, size_t* arrOutLocalIndex);
    static void ComputeGlobalIndexSpace(int dim, size_t* arrInLocalIndex, size_t* arrInOutGlobalIndex);
private:
   size_t m_szWorkGroupSize;
   cl_kernel m_clKernel;
   std::string m_strTitle;
};
    
    
/*!
  * A program can contain multiple kernels.
  */
typedef FastAccessNamedResource<ComputeKernel, TypePointer, NoopInsertRemoveBySafeDelete> ProgramParent;

class ComputeProgram : public ProgramParent
{
public:
    ComputeProgram(cl_program program);
    virtual ~ComputeProgram();
    cl_program handle() const {return m_clProgram;}
private:
    cl_program m_clProgram;
};


    //Storage for compute programs
    typedef FastAccessNamedResource<ComputeProgram, TypePointer, NoopInsertRemoveBySafeDelete> ProgramStorageParent;
    class ProgramStorage : public ProgramStorageParent {
    public:
        ProgramStorage() {}
        virtual ~ProgramStorage() {
            ProgramStorageParent::cleanup();
        }
        
    };
    
/*!
  * A compute device represents and OpenCL Compute Device and can hold multiple programs.
  * An OpenCL program can be added directly from its source code or from a source file.
  * Once the program is added the binary file can be stored.
  * Compute Device can create memory buffer objects for the kernels.
  */
class ComputeDevice {
public:

	enum COMMANDEVENTYPE {cetQueued = CL_PROFILING_COMMAND_QUEUED,
						  cetSubmit = CL_PROFILING_COMMAND_SUBMIT,
						  cetStart = CL_PROFILING_COMMAND_START,
						  cetEnd = CL_PROFILING_COMMAND_END};

    enum DEVICETYPE{dtCPU = CL_DEVICE_TYPE_CPU, dtGPU = CL_DEVICE_TYPE_GPU};


	//Constructors
    ComputeDevice();
    ComputeDevice(DEVICETYPE dev,
    				bool bWithOpenGLInterOp = true,
    				bool bEnableProfiling = false,
    				const char* lpPlatformProvide = "DEFAULT");
  
	//Destructor
	virtual ~ComputeDevice();

    bool isReady() const {return m_bReady;}
    bool isDoubleFPSupported() const;

    /*!
     * Adds a new ComputeProgram from inline source code.
     * @param chrComputeCode inline source code
     * @return index of the program.
     */
    int addProgram(const char* chrComputeCode);
    int addProgramFromFile(const AnsiStr& strFP, bool tryLoadBinary = true);
    void removeProgram(int id);
    bool isProgramIndex(int id) const;
    
    //Add a new kernel to a program object
    ComputeKernel* addKernel(int prgID, const char* chrKernelName);
    

    /*!
     * Prints device info to default output screen.
     */
    void printInfo();

    /*!
     * Create memory buffer for readonly or writeonly access.
     */
    cl_mem createMemBuffer(const size_t size, MEMACCESSMODE mode);
    bool createMemBuffer(const size_t size, MEMACCESSMODE mode, Buffer& b);

    /*!
     * Create memory buffer for readonly or writeonly access from a GL Buffer.
     */
    cl_mem createMemBufferFromGL(cl_GLuint glBuffer, MEMACCESSMODE mode);
    bool createBufferFromGL(cl_GLuint glBuffer, MEMACCESSMODE mode, Buffer& b);
    
    cl_mem createImageFromGL(cl_GLuint glTex, MEMACCESSMODE mode);

    /*!
     * Enqueues a write operation in the command queue
     */
    bool enqueueWriteBuffer(cl_mem destMem, U32 size, const void* lpSource);

    /*!
     * Enqueues a read operation in the command queue
     */
    bool enqueueReadBuffer(cl_mem srcMem, U32 size, void* lpDest);

    /*!
     * Enqueues a copy buffer from source to destination
     */
    bool enqueueCopyBuffer(cl_mem srcMem, cl_mem dstMem,
    						  U32 src_offset, U32 dst_offset, U32 bufferSize);


    /*!
     * Wait to finish all commands in the command q
     */
    void finishAllCommands();


	size_t getKernelWorkgroupSize(ComputeKernel* lpKernel);

	/*!
	 * Enqueues Kernel For running.
	 * @lpKernel ComputeKernel to run
	 * @dim dimension of NDRange
	 * @arrGlobalIndex range of global index space
	 * @arrLocalIndex range of local index space
	 * @return true if successful
	 */
	bool enqueueNDRangeKernel(ComputeKernel* lpKernel, int dim, size_t* arrGlobalIndex, size_t* arrLocalIndex);

	bool enqueueAcquireGLObject(cl_uint count, const cl_mem* arrMemObjects);
	bool enqueueReleaseGLObject(cl_uint count, const cl_mem* arrMemObjects);

    //Access
    cl_device_id getDevice() const {return m_clDeviceID;}
    cl_context getContext() const {return m_clContext;}
    cl_command_queue getCommandQ() const {return m_clCommandQueue;}


    //Profile TimeStamps
    U64 profileTimeStamp(COMMANDEVENTYPE type) const;
    U64 profileExecutionTime() const;
    U64 profileSubmissionTime() const;

protected:
	/*!
	* @param dev Device type for running the compute kernels on
	* @param bWithOpenGLInterOp using openGL inter-operability
	* @param bEnableProfiling to profile and timestamp events
	* @param lpStrPlatformProvider the string name of the platform to use
	*/
	bool initDevice(DEVICETYPE dev,
					  bool bWithOpenGLInterOp = true,
					  bool bEnableProfiling = false,
					  const char* lpStrPlatformProvider = "AMD");


    //Add a new program from ptx binary file.
    ComputeProgram* loadPtxBinary(const U8* binary, size_t length);
    
    //Stores the Ptx Binary of an OpenCL program
    bool storePtxBinary(ComputeProgram* lpProgram, const char* chrFilePath);

public:
	static string ComputeCheckSum(const char* chrFilePath);
	static bool SaveSourceCheckSum(const char* chrFilePath, const string& checksum);
	static bool LoadSourceCheckSum(const char* chrFilePath, string& checksum);


private:
    std::vector<ComputeProgram*> m_lstPrograms;
    DEVICETYPE      m_deviceType;
    cl_platform_id  m_clPlatform;
    cl_device_id    m_clDeviceID;

    cl_context      m_clContext;
    cl_command_queue m_clCommandQueue;
    

    bool m_bReady;
};

    
   

}
}

#endif // PS_COMPUTEDEVICE_H
