#include "ComputeDevice.h"
#include "OclHelperFuncs.h"
#include "base/Logger.h"
#include "base/FileDirectory.h"

#include <iostream>
#include <fstream>
#include <openssl/md5.h>
#ifdef PS_OS_MAC
    #include <OpenGL/OpenGL.h>
#endif

using namespace PS;

namespace PS {
namespace CL {
    
    Buffer::Buffer() {
        init();
    }
    
    Buffer::Buffer(const Buffer& rhs) {
    	m_clMem = rhs.m_clMem;
    	m_size = rhs.m_size;
    	m_access = rhs.m_access;
    	m_isSharedWithGL = rhs.m_isSharedWithGL;
    }

    Buffer::Buffer(cl_mem clMem, U32 size, MEMACCESSMODE access, bool isSharedGL) {
    	this->set(clMem, size, access, isSharedGL);
    }
    
    Buffer::~Buffer() {
        release();
    }
    
    void Buffer::init() {
        m_clMem = NULL;
        m_access = memReadWrite;
        m_isSharedWithGL = false;
    }
    
    void Buffer::set(cl_mem clMem, U32 size, MEMACCESSMODE access, bool isSharedGL) {
        m_clMem = clMem;
        m_size = size;
        m_access = access;
        m_isSharedWithGL = isSharedGL;
    }

    void Buffer::release() {
        if(m_clMem != NULL) {
            clReleaseMemObject(m_clMem);
            m_clMem = NULL;
        }
    }
    
    Buffer& Buffer::operator=(const Buffer& rhs) {
    	this->m_clMem = rhs.m_clMem;
    	this->m_size = rhs.m_size;
    	this->m_access = rhs.m_access;
    	this->m_isSharedWithGL = rhs.m_isSharedWithGL;
    	return (*this);
    }

/////////////////////////////////////////////////////////////////////////////////////
ComputeKernel::ComputeKernel(cl_kernel kernel, const string& strTitle)
{
	m_szWorkGroupSize = 0;
    m_clKernel = kernel;
    m_strTitle = strTitle;
}

ComputeKernel::~ComputeKernel() {
	clReleaseKernel(m_clKernel);
}

bool ComputeKernel::setArg(U32 idxArg, U32 size, const void * lpArg)
{
	cl_int err  = clSetKernelArg(m_clKernel, idxArg, size, lpArg);
	if(err != CL_SUCCESS)
	{
		LogErrorArg2("Kernel Title: %s Failed to set kernel arguments! Ocl Error: %s",
					 m_strTitle.c_str(), oclErrorString(err));
        return false;
	}

	return true;
}

bool ComputeKernel::setArg(U32 idxArg, const Buffer* buf) {
    return this->setArg(idxArg, sizeof(cl_mem), buf->cptr());
}
    
void ComputeKernel::ComputeLocalIndexSpace(int dim, size_t szKernelWorkGroup, size_t* arrOutLocalIndex)
{
	for(int i=0; i<dim; i++)
		arrOutLocalIndex[i] = 1;

	size_t szMultiple = 1;
	int i=0;
	do{
		arrOutLocalIndex[i] <<=1;
		szMultiple = 1;
		for(int j=0; j<dim; j++)
			szMultiple *= arrOutLocalIndex[j];
		//Increment index
		i = (i + 1)%dim;
	}while(szMultiple < szKernelWorkGroup);
}

void ComputeKernel::ComputeGlobalIndexSpace(int dim, size_t* arrInLocalIndex, size_t* arrInOutGlobalIndex)
{
	for(int i=0; i<dim; i++)
	{
		arrInOutGlobalIndex[i] = ToMultipleOf(arrInOutGlobalIndex[i], arrInLocalIndex[i]);
	}
}

/////////////////////////////////////////////////////////////////////////////////////
ComputeProgram::ComputeProgram(cl_program program) {
	m_clProgram = program;
}

ComputeProgram::~ComputeProgram()
{
    ProgramParent::cleanup();
    clReleaseProgram(this->m_clProgram);
}
    
/////////////////////////////////////////////////////////////////////////////////////
ComputeDevice::ComputeDevice()
{
	m_bReady = false;
}

ComputeDevice::ComputeDevice(DEVICETYPE dev,
								 bool bWithOpenGLInterOp,
								 bool bEnableProfiling,
								 const char* lpPlatformProvide)
{
	assert(initDevice(dev, bWithOpenGLInterOp, bEnableProfiling, lpPlatformProvide) == true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
ComputeDevice::~ComputeDevice()
{
	//Erase all programs
	//If a program has associated kernel objects those will be erased as well
	for(U32 i=0; i<m_lstPrograms.size(); i++)
		SAFE_DELETE(m_lstPrograms[i]);

	if(m_bReady)
	{
		clReleaseCommandQueue(m_clCommandQueue);
		clReleaseContext(m_clContext);
	}
}

/*!
* Initializes a device selected by devicetype. A compute context can be created to be InterOperable with
* OpenGL buffers. In case there are multiple platforms installed on this machine, one can be selected
* by specifying it at lpStrPlatformProvider. 
*/
bool ComputeDevice::initDevice(DEVICETYPE dev,
								   bool bWithOpenGLInterOp,
								   bool bEnableProfiling,
								   const char* lpStrPlatformProvider)
{
	m_bReady = false;
    m_deviceType = dev;

    //Get platform, there can be more than one platform installed
	cl_int err = oclPlatformID(&m_clPlatform, lpStrPlatformProvider);
    if (err != CL_SUCCESS)
    {
		LogError("Failed to find a platform!");       
        return false;
    }

    // Get a device of the appropriate type
    err = clGetDeviceIDs(m_clPlatform, dev, 1, &m_clDeviceID, NULL);
    if (err != CL_SUCCESS) {
        LogError("Failed to create a device group!");      
        return false;
    }

    // Create a compute context
    if(bWithOpenGLInterOp)
    {

#ifdef PS_OS_WINDOWS
        HGLRC glCtx = wglGetCurrentContext();
        cl_context_properties props[] = { CL_CONTEXT_PLATFORM,
                          (cl_context_properties)m_clPlatform,
                          CL_WGL_HDC_KHR, (intptr_t) wglGetCurrentDC(),
                          CL_GL_CONTEXT_KHR, (intptr_t) glCtx, 0};
#elif defined(PS_OS_MAC)
        CGLContextObj glContext = CGLGetCurrentContext();
        CGLShareGroupObj shareGroup = CGLGetShareGroup(glContext);
        cl_context_properties props[] = {
           CL_CONTEXT_PROPERTY_USE_CGL_SHAREGROUP_APPLE,
           (cl_context_properties)shareGroup, 0};
#else
        GLXContext glCtx = glXGetCurrentContext();
        cl_context_properties props[] = { CL_CONTEXT_PLATFORM,
                          (cl_context_properties)m_clPlatform,
                          CL_GLX_DISPLAY_KHR, (intptr_t) glXGetCurrentDisplay(),
                          CL_GL_CONTEXT_KHR, (intptr_t) glCtx, 0};
#endif
		m_clContext = clCreateContext(props, 1, &m_clDeviceID, NULL, NULL, &err);
    }
    else
    	m_clContext = clCreateContext(NULL, 1, &m_clDeviceID, NULL, NULL, &err);

    if (!m_clContext)
    {
        LogError("Failed to create a compute context!");       
        return false;
    }

    // Create a command commands
    if(bEnableProfiling)
    	m_clCommandQueue = clCreateCommandQueue(m_clContext, m_clDeviceID, CL_QUEUE_PROFILING_ENABLE, &err);
    else
    	m_clCommandQueue = clCreateCommandQueue(m_clContext, m_clDeviceID, 0, &err);
    if (!m_clCommandQueue) 
	{
        LogError("Failed to create a command commands!");        
        return false;
    }

    m_bReady = true;
	return m_bReady;
}

U64 ComputeDevice::profileTimeStamp(COMMANDEVENTYPE type) const {
	cl_event timing_event = 0;
	cl_ulong timestamp;
	return clGetEventProfilingInfo(timing_event, type, sizeof(timestamp), &timestamp, NULL);
}

U64 ComputeDevice::profileExecutionTime() const {
	cl_event timing_event = 0;
	cl_ulong starttime;
	cl_ulong endtime;

	cl_int errcode = clGetEventProfilingInfo(timing_event, cetStart, sizeof(starttime), &starttime, NULL);
    if(errcode != CL_SUCCESS) {
        if(errcode == CL_PROFILING_INFO_NOT_AVAILABLE) {
            LogError("CL_QUEUE_PROFILING_ENABLE flag is not set for the command-queue or the profiling information is currently not available");
        }
    }
    
	clGetEventProfilingInfo(timing_event, cetEnd, sizeof(endtime), &endtime, NULL);
	return (endtime - starttime);
}

U64 ComputeDevice::profileSubmissionTime() const {
	cl_event timing_event = 0;
	cl_ulong queuedtime;
	cl_ulong submissiontime;

	cl_int errcode = clGetEventProfilingInfo(timing_event, cetQueued, sizeof(queuedtime), &queuedtime, NULL);
    if(errcode != CL_SUCCESS) {
        if(errcode == CL_PROFILING_INFO_NOT_AVAILABLE) {
            LogError("CL_QUEUE_PROFILING_ENABLE flag is not set for the command-queue or the profiling information is currently not available");
        }
    }
	errcode = clGetEventProfilingInfo(timing_event, cetEnd, sizeof(submissiontime), &submissiontime, NULL);
	return (submissiontime - queuedtime);
}


bool ComputeDevice::isDoubleFPSupported() const {
	cl_device_fp_config fp;
    cl_int err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_DOUBLE_FP_CONFIG, sizeof(fp), &fp, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_DEVICE_DOUBLE_FP_CONFIG" << endl;
        return false;
    }
    //CL_FP_FMA | CL_FP_ROUND_TO_NEAREST | CL_FP_ROUND_TO_ZERO | CL_FP_ROUND_TO_INF | CL_FP_INF_NAN | CL_FP_DENORM
    bool bitfield = ((fp & CL_FP_FMA) != 0)&&((fp & CL_FP_ROUND_TO_NEAREST) != 0)&&((fp & CL_FP_ROUND_TO_ZERO) != 0)&&
                    ((fp & CL_FP_ROUND_TO_INF) != 0)&&((fp & CL_FP_INF_NAN) != 0)&&((fp & CL_FP_DENORM) != 0);
    return bitfield;
}

void ComputeDevice::printInfo()
{
	if(!m_bReady) return;

    //Get Platform Info
    char buffer[1024];
	
	cout << "OpenCL Selected Platform Info:" << endl;

	//Platform Profile	
	cl_int err = clGetPlatformInfo(m_clPlatform, CL_PLATFORM_PROFILE, sizeof(buffer), buffer, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_PLATFORM_PROFILE" << endl;
		return;
	}
	cout << "Profile:\t" << buffer << endl;

	//Platform Version
	err = clGetPlatformInfo(m_clPlatform, CL_PLATFORM_VERSION, sizeof(buffer), buffer, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_PLATFORM_VERSION" << endl;
        return;
    }
    cout << "Version:\t" << buffer << endl;

	//Platform Name
	err = clGetPlatformInfo(m_clPlatform, CL_PLATFORM_NAME, sizeof(buffer), buffer, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_PLATFORM_NAME" << endl;
		return;
	}
	cout << "Name:\t" << buffer << endl;

	//Platform Vendor
	err = clGetPlatformInfo(m_clPlatform, CL_PLATFORM_VENDOR, sizeof(buffer), buffer, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_PLATFORM_VENDOR" << endl;
		return;
	}
	cout << "Vendor:\t" << buffer << endl;

	//Platform Extensions
	err = clGetPlatformInfo(m_clPlatform, CL_PLATFORM_EXTENSIONS, sizeof(buffer), buffer, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_PLATFORM_EXTENSIONS" << endl;
		return;
	}
	cout << "Extensions:\t" << buffer << endl;

	/////////////////////////////////////////////////////////////////
	cout << "****************************" << endl;
	cout << "OpenCL Selected Device Info:" << endl;
    //Vendor Name
    err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_VENDOR, sizeof(buffer), buffer, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_DEVICE_VENDOR" << endl;
        return;
    }
    cout << "Vendor:\t" << buffer << endl;

    //Device Name
    err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_NAME, sizeof(buffer), buffer, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_DEVICE_NAME" << endl;
        return;
    }
    cout << "Name:\t" << buffer << endl;
    
    //Device Type
    size_t infoLen;
    cl_device_type infoType;
    
    err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_TYPE, sizeof(infoType), &infoType, &infoLen);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_DEVICE_TYPE" << endl;
        return;
    }
    else {
        if(infoType & CL_DEVICE_TYPE_CPU)
            cout << "TYPE:\t CPU" << endl;
        else if(infoType & CL_DEVICE_TYPE_GPU)
            cout << "TYPE:\t GPU" << endl;
    }
    
    //Max compute units
    size_t units;
    err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(units), &units, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_DEVICE_MAX_COMPUTE_UNITS" << endl;
        return;
    }
    sprintf(buffer, "Max compute units = %lu", units);
    cout << buffer << endl;

	//Max global memory size
	U64 szMem;
	err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(szMem), &szMem, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_DEVICE_GLOBAL_MEM_SIZE" << endl;
		return;
	}
	sprintf(buffer, "Global memory size = %u MB", (U32)(szMem >> 20));
	cout << buffer << endl;

	//Max local memory size
	err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(szMem), &szMem, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_DEVICE_LOCAL_MEM_SIZE" << endl;
		return;
	}
	sprintf(buffer, "Local memory size = %u Bytes", (U32)szMem);
	cout << buffer << endl;

	//Max workgroup size
	err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(units), &units, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_DEVICE_MAX_WORK_GROUP_SIZE" << endl;
		return;
	}
	sprintf(buffer, "CL_DEVICE_MAX_WORK_GROUP_SIZE = %lu", units);
	cout << buffer << endl;

	//Max workitem sizes
	size_t dims[3];
	err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(size_t) * 3, &dims[0], NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_DEVICE_MAX_WORK_ITEM_SIZES" << endl;
		return;
	}
	sprintf(buffer, "CL_DEVICE_MAX_WORK_ITEM_SIZES = [%lu, %lu, %lu]", dims[0], dims[1], dims[2]);
	cout << buffer << endl;
}


cl_mem ComputeDevice::createMemBuffer(const size_t size, MEMACCESSMODE mode)
{
	cl_mem output = clCreateBuffer(m_clContext, mode, size, NULL, NULL);
    if (!output)
    {
    	LogError("Failed to allocate device memory!");
    }

    return output;
}

bool ComputeDevice::createMemBuffer(const size_t size, MEMACCESSMODE mode, Buffer& b) {
	cl_mem m = createMemBuffer(size, mode);
	b.set(m, size, mode, false);
	return (m != NULL);
}

cl_mem ComputeDevice::createMemBufferFromGL(cl_GLuint glBuffer, MEMACCESSMODE mode)
{
	cl_int errCode;
	cl_mem output = clCreateFromGLBuffer(m_clContext, mode, glBuffer, &errCode);
    if (!output)
    {
    	LogErrorArg1("Error: Failed to allocate device gl-interop memory! %s", oclErrorString(errCode));
    }

    return output;
}

bool ComputeDevice::createBufferFromGL(cl_GLuint glBuffer, MEMACCESSMODE mode, Buffer& b) {
	cl_mem m = createMemBufferFromGL(glBuffer, mode);
	b.set(m, sizeof(cl_mem), mode, true);
	return (m !=NULL);
}
    
cl_mem ComputeDevice::createImageFromGL(cl_GLuint glTex, MEMACCESSMODE mode) {
    cl_int errCode;
    cl_mem output = clCreateFromGLTexture2D(m_clContext, mode, GL_TEXTURE_2D, 0, glTex, &errCode);
    if(!output) {
        LogErrorArg1("Error: Failed to allocate device gl-interop texture! %s", oclErrorString(errCode));
    }
    
    return output;
}

/*!
 * Enqueues a write operation in the command queue
 */
bool ComputeDevice::enqueueWriteBuffer(cl_mem destMem, U32 size, const void* lpSource)
{
    cl_int err = clEnqueueWriteBuffer(m_clCommandQueue, destMem,
                               	   	  CL_TRUE, 0, size,
                               	   	  lpSource, 0, NULL, NULL);
    if (err != CL_SUCCESS) {
    	LogErrorArg1("Failed to write to source array! Ocl Error: %s", oclErrorString(err));
        return false;
    }

    return true;
}

/*!
 * Enqueues a read operation in the command queue
 */
bool ComputeDevice::enqueueReadBuffer(cl_mem srcMem, U32 size, void* lpDest)
{
    cl_int err = clEnqueueReadBuffer(m_clCommandQueue, srcMem,
    								 CL_TRUE, 0, size,
								     lpDest, 0, NULL, NULL );
    if (err != CL_SUCCESS) {
    	LogErrorArg1("Failed to read output array! Ocl Error: %s", oclErrorString(err));
        return false;
    }

    return true;
}

bool ComputeDevice::enqueueCopyBuffer(cl_mem srcMem, cl_mem dstMem,
										   U32 src_offset, U32 dst_offset,
										   U32 bufferSize) {

	cl_int err = clEnqueueCopyBuffer(m_clCommandQueue,
										srcMem, dstMem,	src_offset, dst_offset,
										bufferSize, 0, NULL, NULL);
	if(err != CL_SUCCESS) {
		LogErrorArg1("Failed to copy buffer from source to destination. Ocl Error: %s", oclErrorString(err));
		return false;
	}

	return true;
}


void ComputeDevice::finishAllCommands()
{
	clFinish(m_clCommandQueue);
}

    //Compile code
    int ComputeDevice::addProgram(const char* chrComputeCode)
    {
        if(!m_bReady)
        	return -1;
        
        cl_int err = 0;
        
        // Create the compute program from the source buffer
        cl_program program = clCreateProgramWithSource(m_clContext, 1,
                                                       (const char **) &chrComputeCode,
                                                       NULL, &err);
        if (!program) {
            LogErrorArg1("Failed to create compute program! Ocl Error: %s", oclErrorString(err));
            return -1;
        }
        
        // Build the program executable
        err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
        if (err != CL_SUCCESS)
        {
            size_t len;
            char buffer[2048];
            err = clGetProgramBuildInfo(program, m_clDeviceID, CL_PROGRAM_BUILD_LOG,
                                        sizeof(buffer), buffer, &len);
            LogErrorArg1("Failed to build program executable! Ocl Reason: %s", oclErrorString(err));
            LogErrorArg1("Build Error: %s", buffer);
            return NULL;
        }
        
        //Add to the list of programs
        ComputeProgram* lpCompute = new ComputeProgram(program);
        m_lstPrograms.push_back(lpCompute);
        return (int)(m_lstPrograms.size()-1);
    }
    

//Read Program from file and then compile it
int ComputeDevice::addProgramFromFile(const AnsiStr& strFP, bool tryLoadBinary)
{
    if(!m_bReady)
        return -1;
    
    if(!PS::FILESTRINGUTILS::FileExists(strFP)) {
    	LogErrorArg1("File does not exist! Path: %s", strFP.cptr());
    	return -1;
    }

    std::ifstream fp;
    fp.open(strFP.cptr(), std::ios::binary);
    if(!fp.is_open())
        return -1;

    //Compute Source File Checksum for bin compatibility and future storage
    LogInfoArg1("Computing source file MD5 checksum for %s", strFP.cptr());
    string checkSumOrg = ComputeCheckSum(strFP.cptr());
    LogInfoArg1("Computed checksum is %s", checkSumOrg.c_str());
    
    if(tryLoadBinary) {
        string checkSumLoaded;
        ComputeProgram* lpProgram = NULL;
        if(LoadSourceCheckSum(strFP.cptr(), checkSumLoaded)) {
            if(checkSumOrg == checkSumLoaded) {
                
                LogInfo("Checksums matched. Attempting to load ptx binfile.");
                string strBinFile = string(strFP.cptr()) + ".ptx";
                std::ifstream binFile;
                binFile.open(strBinFile.c_str(), std::ios::binary);
                if(binFile.is_open()) {
                    size_t size;
                    binFile.seekg(0, std::ios::end);
                    size = binFile.tellg();
                    binFile.seekg(0, std::ios::beg);
                    
                    U8* buf = new U8[size];
                    binFile.read((char*)buf, size);
                    binFile.close();
                    
                    lpProgram = loadPtxBinary(buf, size);
                    if(lpProgram)
                        LogInfoArg1("Kernels loaded from binfile at: %s", strBinFile.c_str());
                    else
                        LogErrorArg1("An error occured while trying to load ptx binfile from: %s", strBinFile.c_str());
                    
                    SAFE_DELETE(buf);
                }
            }
        }
        
        if(lpProgram) {
            //Add loaded to list
            m_lstPrograms.push_back(lpProgram);
            return ((int)m_lstPrograms.size() - 1);
        }
    }
    
    //From Source Code
    LogInfo("Compiling and storing bin file along with the computed md5 checksum for future loads.");
    size_t size;
    fp.seekg(0, std::ios::end);
    size = fp.tellg();
    fp.seekg(0, std::ios::beg);

    char * buf = new char[size+1];
    //Read file content
    fp.read(buf, size);
    buf[size] = '\0';

    string strCode = string(buf);
    SAFE_DELETE(buf);
    fp.close();
	
	//Save Binary
	int id = addProgram(strCode.c_str());
    if(isProgramIndex(id)) {
        string strBinFile = string(strFP.cptr()) + ".ptx";
        storePtxBinary(m_lstPrograms[id], strBinFile.c_str());
        SaveSourceCheckSum(strFP.cptr(), checkSumOrg);
    }
    
    return id;
}
    
    
    void ComputeDevice::removeProgram(int id)
    {
        if(!isProgramIndex(id))
            return;
        
        m_lstPrograms.erase(m_lstPrograms.begin() + id);
    }

    
    bool ComputeDevice::isProgramIndex(int id) const {
        return (id >=0 && id < (int)m_lstPrograms.size());
    }
    
    ComputeKernel* ComputeDevice::addKernel(int prgID, const char* chrKernelName) {
        if(!isProgramIndex(prgID))
            return NULL;
        
        cl_int err;
        // Create the compute kernel in the program
        cl_kernel k = clCreateKernel(m_lstPrograms[prgID]->handle(), chrKernelName, &err);
        if (!k || err != CL_SUCCESS) {
            LogErrorArg2("Failed to create compute kernel. KernelName:%s,  Error: %s", chrKernelName, oclErrorString(err));
            return NULL;
        }
        
        //Create Kernel Instance and get its workgroup size
        ComputeKernel* kernel = new ComputeKernel(k, chrKernelName);
        getKernelWorkgroupSize(kernel);
        m_lstPrograms[prgID]->add(kernel, chrKernelName);
        
        return kernel;
    }

ComputeProgram* ComputeDevice::loadPtxBinary(const U8* binary, size_t length) {

    cl_int binary_status;
    cl_int err = 0;
    cl_program program = clCreateProgramWithBinary(m_clContext, 1,
    											   &m_clDeviceID,
    											   &length, &binary,
    											   &binary_status,
                                                   &err);
    if (!program || (binary_status != CL_SUCCESS)) {
    	LogErrorArg1("Failed to create compute program from binary! Ocl Error: %s", oclErrorString(err));
        return NULL;
    }

    // Build the program executable
    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[2048];
        err = clGetProgramBuildInfo(program, m_clDeviceID, CL_PROGRAM_BUILD_LOG,
                              sizeof(buffer), buffer, &len);
        LogErrorArg1("Failed to build program executable! Ocl Reason: %s", oclErrorString(err));
        LogErrorArg1("Build Error: %s", buffer);
        return NULL;
    }

	//Add to the list of programs
    ComputeProgram* lpCompute = new ComputeProgram(program);
    return lpCompute;
}



bool ComputeDevice::storePtxBinary(ComputeProgram* lpProgram,
											const char* chrFilePath) {
	cl_uint numDevices = 0;
	cl_int errNum;

	// 1 - Query for number of devices attached to program
	errNum = clGetProgramInfo(lpProgram->handle(), CL_PROGRAM_NUM_DEVICES, sizeof(cl_uint),
								&numDevices, NULL);
	if (errNum != CL_SUCCESS) {
		LogError("Error querying for number of devices.");
		return false;
	}

	// 2 - Get all of the Device IDs
	cl_device_id *devices = new cl_device_id[numDevices];
	errNum = clGetProgramInfo(lpProgram->handle(), CL_PROGRAM_DEVICES,
								sizeof(cl_device_id) * numDevices, devices, NULL);
	if (errNum != CL_SUCCESS) {
		LogError("Error querying for devices.");
		SAFE_DELETE_ARRAY(devices);
		return false;
	}

	// 3 - Determine the size of each program binary
	size_t *programBinarySizes = new size_t[numDevices];
	errNum = clGetProgramInfo(lpProgram->handle(), CL_PROGRAM_BINARY_SIZES,
								sizeof(size_t) * numDevices, programBinarySizes, NULL);
	if (errNum != CL_SUCCESS) {
		LogError("Error querying for program binary sizes.");
		SAFE_DELETE_ARRAY(devices);
		SAFE_DELETE_ARRAY(programBinarySizes);
		return false;
	}

	unsigned char **programBinaries = new unsigned char*[numDevices];
	for (cl_uint i = 0; i < numDevices; i++) {
		programBinaries[i] = new unsigned char[programBinarySizes[i]];
	}

	// 4 - Get all of the program binaries
	errNum = clGetProgramInfo(lpProgram->handle(), CL_PROGRAM_BINARIES,
								sizeof(unsigned char*) * numDevices, programBinaries, NULL);
	if (errNum != CL_SUCCESS) {
		LogError("Error querying for program binaries.");

		SAFE_DELETE_ARRAY(devices);
		SAFE_DELETE_ARRAY(programBinarySizes);
		for (cl_uint i = 0; i < numDevices; i++) {
			SAFE_DELETE_ARRAY(programBinaries[i]);
		}
		SAFE_DELETE_ARRAY(programBinaries);
		return false;
	}

	// 5 - Finally store the binaries for the device requested out to disk for future reading.
	for (cl_uint i = 0; i < numDevices; i++) {
		// Store the binary just for the device requested.  In a scenario where
		// multiple devices were being used you would save all of the binaries out here.
		if (devices[i] == m_clDeviceID) {
			FILE *fp = fopen(chrFilePath, "wb");
			fwrite(programBinaries[i], 1, programBinarySizes[i], fp);
			fclose(fp);
			break;
		}
	}

	// Cleanup
	SAFE_DELETE_ARRAY(devices);
	SAFE_DELETE_ARRAY(programBinarySizes);
	for (cl_uint i = 0; i < numDevices; i++) {
		SAFE_DELETE_ARRAY(programBinaries[i]);
	}
	SAFE_DELETE_ARRAY(programBinaries);

	return true;
}


string ComputeDevice::ComputeCheckSum(const char* chrFilePath) {
	unsigned char c[MD5_DIGEST_LENGTH];
	MD5_CTX mdContext;

	int bytes;
	U8 data[1024];
	FILE* inFile = fopen(chrFilePath, "rb");
	if(inFile == NULL) {
		LogError("Unable to load the text file!");
		return string("error");
	}

	MD5_Init(&mdContext);
	while ((bytes = fread(data, 1, 1024, inFile)) != 0) {
		MD5_Update(&mdContext, data, bytes);
	}
	MD5_Final(c, &mdContext);
	fclose(inFile);

	string strOut;
	char buffer[32];
	for(int i = 0; i < MD5_DIGEST_LENGTH; i++) {
		sprintf(buffer, "%02x", c[i]);
		strOut += string(buffer);
	}
	return strOut;
}

bool ComputeDevice::LoadSourceCheckSum(const char* chrFilePath, string& checksum) {
	string strCheckSumFP = string(chrFilePath) + string(".md5");
	ifstream checksumFile(strCheckSumFP.c_str());
	if(checksumFile.is_open()) {
		checksumFile >> checksum;
		checksumFile.close();
		return true;
	}

	return false;
}

bool ComputeDevice::SaveSourceCheckSum(const char* chrFilePath, const string& checksum) {
	string strCheckSumFP = string(chrFilePath) + string(".md5");
	ofstream checksumFile;
	checksumFile.open(strCheckSumFP.c_str());
	checksumFile << checksum;
	checksumFile.close();
	return true;
}

size_t ComputeDevice::getKernelWorkgroupSize(ComputeKernel* lpKernel)
{
	size_t szWorkGroup = 0;
	cl_int err = clGetKernelWorkGroupInfo(lpKernel->getKernel(), 
										  m_clDeviceID,
										  CL_KERNEL_WORK_GROUP_SIZE,
										  sizeof(szWorkGroup), &szWorkGroup, NULL);
	if (err != CL_SUCCESS) 
	{
		LogErrorArg1("Failed to retrieve kernel work group info! Ocl Error: %s", oclErrorString(err));
	}

	//Set Kernel WorkGroup Size
	lpKernel->setKernelWorkGroupSize(szWorkGroup);

	return szWorkGroup;
}

bool ComputeDevice::enqueueNDRangeKernel(ComputeKernel* lpKernel, int dim, size_t* arrGlobalIndex, size_t* arrLocalIndex)
{
	cl_int err = clEnqueueNDRangeKernel(m_clCommandQueue, lpKernel->getKernel(),
								 	    dim, NULL, arrGlobalIndex, arrLocalIndex,
								 	    0, NULL, NULL);

	if (err) {
		LogErrorArg2("Failed to execute %s kernel! Ocl Error: %s",
					 lpKernel->getTitle().c_str(),
					 oclErrorString(err));
		return false;
	}
	else
		return true;
}

bool ComputeDevice::enqueueAcquireGLObject(cl_uint count, const cl_mem* arrMemObjects)
{
	cl_int err = clEnqueueAcquireGLObjects(m_clCommandQueue, count, arrMemObjects, 0, 0, 0);
	if (err) {
		LogErrorArg1("Failed to acquire OpenGL object! Ocl Error: %s",
					 oclErrorString(err));
		return false;
	}
	else
		return true;
}

bool ComputeDevice::enqueueReleaseGLObject(cl_uint count, const cl_mem* arrMemObjects)
{
	cl_int err = clEnqueueReleaseGLObjects(m_clCommandQueue, count, arrMemObjects, 0, 0, 0);
	if (err) {
		LogErrorArg1("Failed to release OpenGL object! Ocl Error: %s",
					 oclErrorString(err));
		return false;
	}
	else
		return true;
}



}
}
