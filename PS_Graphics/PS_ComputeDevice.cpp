#include "PS_ComputeDevice.h"
#include "../PS_Base/PS_ErrorManager.h"
#include "../PS_Base/PS_Logger.h"
#include <iostream>
#include <fstream>
#include <openssl/md5.h>
#ifdef PS_OS_MAC
    #include <CGLCurrent.h>
#endif

using namespace PS;

namespace PS{
namespace HPC{
/////////////////////////////////////////////////////////////////////////////////////
bool ComputeKernel::setArg(U32 idxArg, U32 size, const void * lpArg)
{
	cl_int err  = clSetKernelArg(m_clKernel, idxArg, size, lpArg);
	if(err != CL_SUCCESS)
	{
		LogErrorArg2("Kernel Title: %s Failed to set kernel arguments! Ocl Error: %s",
					 m_strTitle.c_str(), ComputeDevice::oclErrorString(err));
        return false;
	}

	return true;
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
ComputeProgram::~ComputeProgram()
{
	for(size_t i=0; i<m_lstKernels.size(); i++)
	{
		SAFE_DELETE(m_lstKernels[i]);
	}
	m_lstKernels.resize(0);
	clReleaseProgram(this->m_clProgram);
}

ComputeKernel* ComputeProgram::addKernel(const char *chrKernelTitle)
{
    cl_int err;
    // Create the compute kernel in the program
    cl_kernel k = clCreateKernel(m_clProgram, chrKernelTitle, &err);
    if (!k || err != CL_SUCCESS) {
    	LogErrorArg1("Failed to create compute kernel. Ocl Error: %s", ComputeDevice::oclErrorString(err));
        return NULL;
    }

    ComputeKernel* kernel = new ComputeKernel(k, chrKernelTitle);
    m_lstKernels.push_back(kernel);
    return kernel;
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
	cl_int err = this->oclPlatformID(&m_clPlatform, lpStrPlatformProvider);
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
	cl_event timing_event;
	cl_ulong timestamp;
	return clGetEventProfilingInfo(timing_event, type, sizeof(timestamp), &timestamp, NULL);
}

U64 ComputeDevice::profileExecutionTime() const {
	cl_event timing_event;
	cl_ulong starttime;
	cl_ulong endtime;

	cl_int errcode = clGetEventProfilingInfo(timing_event, cetStart, sizeof(starttime), &starttime, NULL);

	clGetEventProfilingInfo(timing_event, cetEnd, sizeof(endtime), &endtime, NULL);
	return (endtime - starttime);
}

U64 ComputeDevice::profileSubmissionTime() const {
	cl_event timing_event;
	cl_ulong queuedtime;
	cl_ulong submissiontime;

	cl_int errcode = clGetEventProfilingInfo(timing_event, cetQueued, sizeof(queuedtime), &queuedtime, NULL);
	errcode = clGetEventProfilingInfo(timing_event, cetEnd, sizeof(submissiontime), &submissiontime, NULL);
	return (submissiontime - queuedtime);
}

//////////////////////////////////////////////////////////////////////////////////////////////
cl_int ComputeDevice::oclPlatformID(cl_platform_id* clSelectedPlatformID, const char* lpStrProvider)
{
    char chBuffer[1024];
    cl_uint num_platforms;
    cl_platform_id* clPlatformIDs;
    cl_int ciErrNum;
    *clSelectedPlatformID = NULL;

    // Get OpenCL platform count
    ciErrNum = clGetPlatformIDs (0, NULL, &num_platforms);
    if (ciErrNum != CL_SUCCESS)
    {
        LogErrorArg1("Error code: %i in clGetPlatformIDs Call!!", ciErrNum);
        return -1000;
    }
    else
    {
        if(num_platforms == 0)
        {
        	LogError("No OpenCL platform found!");
            return -2000;
        }
        else
        {
            // if there's a platform or more, make space for ID's
            if ((clPlatformIDs = (cl_platform_id*)malloc(num_platforms * sizeof(cl_platform_id))) == NULL)
            {
            	LogError("Failed to allocate memory for cl_platform ID's!");
                return -3000;
            }

            // get platform info for each platform and trap the NVIDIA platform if found
            ciErrNum = clGetPlatformIDs (num_platforms, clPlatformIDs, NULL);
            for(cl_uint i = 0; i < num_platforms; ++i)
            {
                ciErrNum = clGetPlatformInfo (clPlatformIDs[i], CL_PLATFORM_NAME, 1024, &chBuffer, NULL);
                if(ciErrNum == CL_SUCCESS)
                {
					if(strstr(chBuffer, lpStrProvider) != NULL)
                    {
                        *clSelectedPlatformID = clPlatformIDs[i];
                        break;
                    }
                }
            }

            // default to zeroeth platform if NVIDIA not found
            if(*clSelectedPlatformID == NULL)
            {
				LogWarningArg1("%s OpenCL platform not found - defaulting to first platform!", lpStrProvider);
                *clSelectedPlatformID = clPlatformIDs[0];
            }

            free(clPlatformIDs);
        }
    }

    return CL_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////////////////////
int ComputeDevice::oclGetDevCap(cl_device_id device)
{
    char cDevString[1024];
    bool bDevAttributeQuery = false;
    int iDevArch = -1;

    // Get device extensions, and if any then search for cl_nv_device_attribute_query
    clGetDeviceInfo(device, CL_DEVICE_EXTENSIONS, sizeof(cDevString), &cDevString, NULL);
    if (cDevString != 0)
    {
        std::string stdDevString;
        stdDevString = std::string(cDevString);
        size_t szOldPos = 0;
        size_t szSpacePos = stdDevString.find(' ', szOldPos); // extensions string is space delimited
        while (szSpacePos != stdDevString.npos)
        {
            if( strcmp("cl_nv_device_attribute_query", stdDevString.substr(szOldPos, szSpacePos - szOldPos).c_str()) == 0 )
            {
                bDevAttributeQuery = true;
            }

            do {
                szOldPos = szSpacePos + 1;
                szSpacePos = stdDevString.find(' ', szOldPos);
            } while (szSpacePos == szOldPos);
        }
    }

    // if search succeeded, get device caps
    if(bDevAttributeQuery)
    {
        cl_int iComputeCapMajor, iComputeCapMinor;
#ifdef CL_DEVICE_COMPUTE_CAPABILITY_MAJOR_NV
        clGetDeviceInfo(device, CL_DEVICE_COMPUTE_CAPABILITY_MAJOR_NV, sizeof(cl_uint), (void*)&iComputeCapMajor, NULL);
#endif
#ifdef CL_DEVICE_COMPUTE_CAPABILITY_MINOR_NV
        clGetDeviceInfo(device, CL_DEVICE_COMPUTE_CAPABILITY_MINOR_NV, sizeof(cl_uint), (void*)&iComputeCapMinor, NULL);
#endif
        iDevArch = (10 * iComputeCapMajor) + iComputeCapMinor;
    }

    return iDevArch;
}

//////////////////////////////////////////////////////////////////////////////////////////////
cl_device_id ComputeDevice::oclGetDev(cl_context cxGPUContext, unsigned int nr)
{
    size_t szParmDataBytes;
    cl_device_id* cdDevices;

    // get the list of GPU devices associated with context
    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);

    if( szParmDataBytes / sizeof(cl_device_id) <= nr ) {
      return (cl_device_id)-1;
    }

    cdDevices = (cl_device_id*) malloc(szParmDataBytes);

    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);

    cl_device_id device = cdDevices[nr];
    free(cdDevices);

    return device;
}

//////////////////////////////////////////////////////////////////////////////////////////////
cl_device_id ComputeDevice::oclGetMaxFlopsDev(cl_context cxGPUContext)
{
    size_t szParmDataBytes;
    cl_device_id* cdDevices;

    // get the list of GPU devices associated with context
    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);
    cdDevices = (cl_device_id*) malloc(szParmDataBytes);
    size_t device_count = szParmDataBytes / sizeof(cl_device_id);

    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);

    cl_device_id max_flops_device = cdDevices[0];
    int max_flops = 0;

    size_t current_device = 0;

    // CL_DEVICE_MAX_COMPUTE_UNITS
    cl_uint compute_units;
    clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(compute_units), &compute_units, NULL);

    // CL_DEVICE_MAX_CLOCK_FREQUENCY
    cl_uint clock_frequency;
    clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(clock_frequency), &clock_frequency, NULL);

    max_flops = compute_units * clock_frequency;
    ++current_device;

    while( current_device < device_count )
    {
        // CL_DEVICE_MAX_COMPUTE_UNITS
        cl_uint compute_units;
        clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(compute_units), &compute_units, NULL);

        // CL_DEVICE_MAX_CLOCK_FREQUENCY
        cl_uint clock_frequency;
        clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(clock_frequency), &clock_frequency, NULL);

        int flops = compute_units * clock_frequency;
        if( flops > max_flops )
        {
            max_flops        = flops;
            max_flops_device = cdDevices[current_device];
        }
        ++current_device;
    }

    free(cdDevices);

    return max_flops_device;
}


void ComputeDevice::oclLogPtx(cl_program cpProgram, cl_device_id cdDevice, const char* cPtxFileName)
{
    // Grab the number of devices associated with the program
    cl_uint num_devices;
    clGetProgramInfo(cpProgram, CL_PROGRAM_NUM_DEVICES, sizeof(cl_uint), &num_devices, NULL);

    // Grab the device ids
    cl_device_id* devices = (cl_device_id*) malloc(num_devices * sizeof(cl_device_id));
    clGetProgramInfo(cpProgram, CL_PROGRAM_DEVICES, num_devices * sizeof(cl_device_id), devices, 0);

    // Grab the sizes of the binaries
    size_t* binary_sizes = (size_t*)malloc(num_devices * sizeof(size_t));
    clGetProgramInfo(cpProgram, CL_PROGRAM_BINARY_SIZES, num_devices * sizeof(size_t), binary_sizes, NULL);

    // Now get the binaries
    char** ptx_code = (char**)malloc(num_devices * sizeof(char*));
    for( unsigned int i=0; i<num_devices; ++i)
    {
        ptx_code[i] = (char*)malloc(binary_sizes[i]);
    }
    clGetProgramInfo(cpProgram, CL_PROGRAM_BINARIES, 0, ptx_code, NULL);

    // Find the index of the device of interest
    unsigned int idx = 0;
    while((idx < num_devices) && (devices[idx] != cdDevice))
    {
        ++idx;
    }

    // If the index is associated, log the result
    if(idx < num_devices)
    {
        // if a separate filename is supplied, dump ptx there
        if (NULL != cPtxFileName)
        {
            ReportErrorExt("\nWriting ptx to separate file: %s ...\n\n", cPtxFileName);
            FILE* pFileStream = NULL;
            #ifdef _WIN32
                fopen_s(&pFileStream, cPtxFileName, "wb");
            #else
                pFileStream = fopen(cPtxFileName, "wb");
            #endif

            fwrite(ptx_code[idx], binary_sizes[idx], 1, pFileStream);
            fclose(pFileStream);
        }
        else // log to logfile and console if no ptx file specified
        {
           //ReportErrorExt("\n%s\nProgram Binary:\n%s\n%s\n", HDASHLINE, ptx_code[idx], HDASHLINE);
        }
    }

    // Cleanup
    free(devices);
    free(binary_sizes);
    for(unsigned int i = 0; i < num_devices; ++i)
    {
        free(ptx_code[i]);
    }
    free( ptx_code );
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

//Read Program from file and then compile it
ComputeProgram* ComputeDevice::addProgramFromFile(const char *chrFilePath)
{
    if(!m_bReady) return NULL;
    std::ifstream fp;
    fp.open(chrFilePath, std::ios::binary);
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

    string strCode = string(buf);
    SAFE_DELETE(buf);
    fp.close();
	
	//Save Binary
	return addProgram(strCode.c_str());
}

ComputeProgram* ComputeDevice::addProgramFromPtxBinary(const U8* binary, size_t length) {

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
    m_lstPrograms.push_back(lpCompute);
    return lpCompute;
}

ComputeProgram* ComputeDevice::tryLoadBinaryThenCompile(const char* chrFilePath) {

	LogInfoArg1("Computing source file MD5 checksum for %s", chrFilePath);
	string checkSumOrg = ComputeCheckSum(chrFilePath);

	LogInfoArg1("Computed checksum is %s", checkSumOrg.c_str());
	string checkSumLoaded;
	ComputeProgram* lpProgram = NULL;
	if(LoadSourceCheckSum(chrFilePath, checkSumLoaded)) {
		if(checkSumOrg == checkSumLoaded) {

			LogInfo("Checksums matched. Attempting to load ptx binfile.");
    	    string strBinFile = string(chrFilePath) + ".ptx";
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

    	    	 lpProgram = addProgramFromPtxBinary(buf, size);
    	    	 if(lpProgram)
    	    		 LogInfoArg1("Kernels loaded from binfile at: %s", strBinFile.c_str());
    	    	 else
    	    		 LogErrorArg1("An error occured while trying to load ptx binfile from: %s", strBinFile.c_str());

    	    	 SAFE_DELETE(buf);
    	    }
		}
	}

	if(!lpProgram) {
		LogInfo("Compiling and storing bin file along with the computed md5 checksum for future loads.");
		lpProgram = addProgramFromFile(chrFilePath);

		//Saving Binary
		if(lpProgram) {
			string strBinFile = string(chrFilePath) + ".ptx";
			storeProgramBinary(lpProgram, strBinFile.c_str());
			SaveSourceCheckSum(chrFilePath, checkSumOrg);
		}
	}

	return lpProgram;
}

bool ComputeDevice::storeProgramBinary(ComputeProgram* lpProgram,
											const char* chrFilePath) {
	cl_uint numDevices = 0;
	cl_int errNum;

	// 1 - Query for number of devices attached to program
	errNum = clGetProgramInfo(lpProgram->getProgram(), CL_PROGRAM_NUM_DEVICES, sizeof(cl_uint),
								&numDevices, NULL);
	if (errNum != CL_SUCCESS) {
		LogError("Error querying for number of devices.");
		return false;
	}

	// 2 - Get all of the Device IDs
	cl_device_id *devices = new cl_device_id[numDevices];
	errNum = clGetProgramInfo(lpProgram->getProgram(), CL_PROGRAM_DEVICES,
								sizeof(cl_device_id) * numDevices, devices, NULL);
	if (errNum != CL_SUCCESS) {
		LogError("Error querying for devices.");
		SAFE_DELETE_ARRAY(devices);
		return false;
	}

	// 3 - Determine the size of each program binary
	size_t *programBinarySizes = new size_t[numDevices];
	errNum = clGetProgramInfo(lpProgram->getProgram(), CL_PROGRAM_BINARY_SIZES,
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
	errNum = clGetProgramInfo(lpProgram->getProgram(), CL_PROGRAM_BINARIES,
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
#ifdef PS_OS_LINUX
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
#else
	return string("OPENSSL Not Supported!");
#endif

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

//Compile code
ComputeProgram* ComputeDevice::addProgram(const char* chrComputeCode)
{
    if(!m_bReady) return NULL;

    cl_int err = 0;

    // Create the compute program from the source buffer
    cl_program program = clCreateProgramWithSource(m_clContext, 1,
                                                  (const char **) &chrComputeCode,
                                                  NULL, &err);
    if (!program) {
    	LogErrorArg1("Failed to create compute program! Ocl Error: %s", oclErrorString(err));
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
    m_lstPrograms.push_back(lpCompute);
    return lpCompute;
}

void ComputeDevice::removeProgram(const ComputeProgram* lpProgram)
{
	U32 index = 0;
	for(U32 i=0; i<m_lstPrograms.size(); i++)
	{
		if(m_lstPrograms[i] == lpProgram)
		{
			index = i;
			break;
		}
	}

	m_lstPrograms.erase(m_lstPrograms.begin() + index);
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
					 ComputeDevice::oclErrorString(err));
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
					 ComputeDevice::oclErrorString(err));
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
					 ComputeDevice::oclErrorString(err));
		return false;
	}
	else
		return true;
}

const char* ComputeDevice::oclErrorString(cl_int error)
{
    static const char* errorString[] = {
        "CL_SUCCESS",
        "CL_DEVICE_NOT_FOUND",
        "CL_DEVICE_NOT_AVAILABLE",
        "CL_COMPILER_NOT_AVAILABLE",
        "CL_MEM_OBJECT_ALLOCATION_FAILURE",
        "CL_OUT_OF_RESOURCES",
        "CL_OUT_OF_HOST_MEMORY",
        "CL_PROFILING_INFO_NOT_AVAILABLE",
        "CL_MEM_COPY_OVERLAP",
        "CL_IMAGE_FORMAT_MISMATCH",
        "CL_IMAGE_FORMAT_NOT_SUPPORTED",
        "CL_BUILD_PROGRAM_FAILURE",
        "CL_MAP_FAILURE",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "",
        "CL_INVALID_VALUE",
        "CL_INVALID_DEVICE_TYPE",
        "CL_INVALID_PLATFORM",
        "CL_INVALID_DEVICE",
        "CL_INVALID_CONTEXT",
        "CL_INVALID_QUEUE_PROPERTIES",
        "CL_INVALID_COMMAND_QUEUE",
        "CL_INVALID_HOST_PTR",
        "CL_INVALID_MEM_OBJECT",
        "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR",
        "CL_INVALID_IMAGE_SIZE",
        "CL_INVALID_SAMPLER",
        "CL_INVALID_BINARY",
        "CL_INVALID_BUILD_OPTIONS",
        "CL_INVALID_PROGRAM",
        "CL_INVALID_PROGRAM_EXECUTABLE",
        "CL_INVALID_KERNEL_NAME",
        "CL_INVALID_KERNEL_DEFINITION",
        "CL_INVALID_KERNEL",
        "CL_INVALID_ARG_INDEX",
        "CL_INVALID_ARG_VALUE",
        "CL_INVALID_ARG_SIZE",
        "CL_INVALID_KERNEL_ARGS",
        "CL_INVALID_WORK_DIMENSION",
        "CL_INVALID_WORK_GROUP_SIZE",
        "CL_INVALID_WORK_ITEM_SIZE",
        "CL_INVALID_GLOBAL_OFFSET",
        "CL_INVALID_EVENT_WAIT_LIST",
        "CL_INVALID_EVENT",
        "CL_INVALID_OPERATION",
        "CL_INVALID_GL_OBJECT",
        "CL_INVALID_BUFFER_SIZE",
        "CL_INVALID_MIP_LEVEL",
        "CL_INVALID_GLOBAL_WORK_SIZE",
    };

    const int errorCount = sizeof(errorString) / sizeof(errorString[0]);

    const int index = -error;

    return (index >= 0 && index < errorCount) ? errorString[index] : "Unspecified Error";
}

// Helper function to get OpenCL image format string (channel order and type) from constant
// *********************************************************************
const char* ComputeDevice::oclImageFormatString(cl_uint uiImageFormat)
{
    // cl_channel_order
    if (uiImageFormat == CL_R)return "CL_R";
    if (uiImageFormat == CL_A)return "CL_A";
    if (uiImageFormat == CL_RG)return "CL_RG";
    if (uiImageFormat == CL_RA)return "CL_RA";
    if (uiImageFormat == CL_RGB)return "CL_RGB";
    if (uiImageFormat == CL_RGBA)return "CL_RGBA";
    if (uiImageFormat == CL_BGRA)return "CL_BGRA";
    if (uiImageFormat == CL_ARGB)return "CL_ARGB";
    if (uiImageFormat == CL_INTENSITY)return "CL_INTENSITY";
    if (uiImageFormat == CL_LUMINANCE)return "CL_LUMINANCE";

    // cl_channel_type
    if (uiImageFormat == CL_SNORM_INT8)return "CL_SNORM_INT8";
    if (uiImageFormat == CL_SNORM_INT16)return "CL_SNORM_INT16";
    if (uiImageFormat == CL_UNORM_INT8)return "CL_UNORM_INT8";
    if (uiImageFormat == CL_UNORM_INT16)return "CL_UNORM_INT16";
    if (uiImageFormat == CL_UNORM_SHORT_565)return "CL_UNORM_SHORT_565";
    if (uiImageFormat == CL_UNORM_SHORT_555)return "CL_UNORM_SHORT_555";
    if (uiImageFormat == CL_UNORM_INT_101010)return "CL_UNORM_INT_101010";
    if (uiImageFormat == CL_SIGNED_INT8)return "CL_SIGNED_INT8";
    if (uiImageFormat == CL_SIGNED_INT16)return "CL_SIGNED_INT16";
    if (uiImageFormat == CL_SIGNED_INT32)return "CL_SIGNED_INT32";
    if (uiImageFormat == CL_UNSIGNED_INT8)return "CL_UNSIGNED_INT8";
    if (uiImageFormat == CL_UNSIGNED_INT16)return "CL_UNSIGNED_INT16";
    if (uiImageFormat == CL_UNSIGNED_INT32)return "CL_UNSIGNED_INT32";
    if (uiImageFormat == CL_HALF_FLOAT)return "CL_HALF_FLOAT";
    if (uiImageFormat == CL_FLOAT)return "CL_FLOAT";

    // unknown constant
    return "Unknown";
}


}
}
