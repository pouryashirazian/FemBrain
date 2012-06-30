#include "PS_ComputeDevice.h"
#include "PS_ErrorManager.h"
#include "PS_Logger.h"
#include <iostream>
#include <fstream>

using namespace PS;

namespace PS{
namespace HPC{
/////////////////////////////////////////////////////////////////////////////////////
bool ComputeKernel::setArg(U32 idxArg, U32 size, const void * lpArg)
{
	cl_int err  = clSetKernelArg(m_clKernel, idxArg, size, lpArg);
	if(err != CL_SUCCESS)
	{
        cerr << "Error: Failed to set kernel arguments! " << err << endl;
        ReportError("Error: Failed to set kernel arguments!");
        FlushAllErrors();
        return false;
	}

	return true;
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
        ReportError("Error: Failed to create compute kernel!");
        FlushAllErrors();
        return NULL;
    }

    ComputeKernel* kernel = new ComputeKernel(k, chrKernelTitle);
    m_lstKernels.push_back(kernel);
    return kernel;
}


bool ComputeProgram::saveBinary(const char* chrBinFilePath)
{
	//Get the binary
	U64 szBinFile;
	cl_int err = clGetProgramInfo(m_clProgram, CL_PROGRAM_BINARY_SIZES, sizeof(szBinFile), &szBinFile, NULL);
	if(err != CL_SUCCESS)
	{
		ReportError("Unable to retrieve the size of the program binary.");
		return false;
	}
	else
	{
		bool bres = false;
		char* lpBinFile = new char[szBinFile];
		err = clGetProgramInfo(m_clProgram, CL_PROGRAM_BINARIES, szBinFile, lpBinFile, NULL);
		if(err != CL_SUCCESS)		
			ReportError("Unable to retrieve the program binary.");					
		else
		{
			ofstream binFile;
			binFile.open (chrBinFilePath, ios::out | ios::binary);
			binFile.write(lpBinFile, szBinFile);			
			binFile.close();			
			bres = true;
		}

		SAFE_DELETE(lpBinFile);
		return bres;
	}

	return false;
}
/////////////////////////////////////////////////////////////////////////////////////
ComputeDevice::ComputeDevice(DEVICETYPE dev, bool bWithOpenGLInterOp, const char* lpPlatformProvide)
{
	assert(initDevice(dev, bWithOpenGLInterOp, lpPlatformProvide) == true);
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

bool ComputeDevice::initDevice(DEVICETYPE dev, bool bWithOpenGLInterOp, const char* lpStrPlatformProvider)
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
		#else //!_WIN32
		  GLXContext glCtx = glXGetCurrentContext();
		#endif //!_WIN32

		cl_context_properties props[] = { CL_CONTEXT_PLATFORM,
						  (cl_context_properties)m_clPlatform,
		#ifdef PS_OS_WINDOWS
						  CL_WGL_HDC_KHR, (intptr_t) wglGetCurrentDC(),
		#else //!_WIN32
						  CL_GLX_DISPLAY_KHR, (intptr_t) glXGetCurrentDisplay(),
		#endif //!_WIN32
						  CL_GL_CONTEXT_KHR, (intptr_t) glCtx, 0};
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
    m_clCommandQueue = clCreateCommandQueue(m_clContext, m_clDeviceID, 0, &err);
    if (!m_clCommandQueue) 
	{
        LogError("Failed to create a command commands!");        
        return false;
    }

    m_bReady = true;
	return m_bReady;
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
        clGetDeviceInfo(device, CL_DEVICE_COMPUTE_CAPABILITY_MAJOR_NV, sizeof(cl_uint), (void*)&iComputeCapMajor, NULL);
        clGetDeviceInfo(device, CL_DEVICE_COMPUTE_CAPABILITY_MINOR_NV, sizeof(cl_uint), (void*)&iComputeCapMinor, NULL);
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

void ComputeDevice::oclGetProgBinary( cl_program cpProgram, cl_device_id cdDevice, char** binary, size_t* length)
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
    char** ptx_code = (char**) malloc(num_devices * sizeof(char*));
    for( unsigned int i=0; i<num_devices; ++i) {
        ptx_code[i]= (char*)malloc(binary_sizes[i]);
    }
    clGetProgramInfo(cpProgram, CL_PROGRAM_BINARIES, 0, ptx_code, NULL);

    // Find the index of the device of interest
    unsigned int idx = 0;
    while( idx<num_devices && devices[idx] != cdDevice ) ++idx;

    // If it is associated prepare the result
    if( idx < num_devices )
    {
        *binary = ptx_code[idx];
        *length = binary_sizes[idx];
    }

    // Cleanup
    free( devices );
    free( binary_sizes );
    for( unsigned int i=0; i<num_devices; ++i) {
        if( i != idx ) free(ptx_code[i]);
    }
    free( ptx_code );
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
    U32 units;
    err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(units), &units, NULL);
    if (err != CL_SUCCESS) {
        cerr << "Error: Failed to get CL_DEVICE_MAX_COMPUTE_UNITS" << endl;
        return;
    }
    sprintf(buffer, "Max compute units = %u", units);
    cout << buffer << endl;

	//Max global memory size
	U64 szMem;
	err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(szMem), &szMem, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_DEVICE_GLOBAL_MEM_SIZE" << endl;
		return;
	}
	sprintf(buffer, "Global memory size = %u MB", (szMem >> 20));
	cout << buffer << endl;

	//Max local memory size
	err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(szMem), &szMem, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_DEVICE_LOCAL_MEM_SIZE" << endl;
		return;
	}
	sprintf(buffer, "Local memory size = %u Bytes", szMem);
	cout << buffer << endl;

	//Max workgroup size
	err = clGetDeviceInfo(m_clDeviceID, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(units), &units, NULL);
	if (err != CL_SUCCESS) {
		cerr << "Error: Failed to get CL_DEVICE_MAX_WORK_GROUP_SIZE" << endl;
		return;
	}
	sprintf(buffer, "Max workgroup size = %u", units);
	cout << buffer << endl;
}


cl_mem ComputeDevice::createMemBuffer(const U32 size, MEMACCESSMODE mode)
{
	cl_mem output = clCreateBuffer(m_clContext, mode, size, NULL, NULL);
    if (!output)
    {
        cerr << "Error: Failed to allocate device memory!" << endl;
        ReportError("Error: Failed to allocate device memory!");
        FlushAllErrors();
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
        cerr << "Error: Failed to write to source array!" << endl;
        ReportError("Error: Failed to write to source array!");
        FlushAllErrors();
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
        cerr << "Error: Failed to read output array! " <<  err << endl;
        ReportError("Error: Failed to read output array!");
        FlushAllErrors();

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
	ComputeProgram* lpProgram = addProgram(strCode.c_str());
	std::string strBin = string(chrFilePath) + string(".ptx");
	lpProgram->saveBinary(strBin.c_str());
	return lpProgram;
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
        ReportError("Error: Failed to create compute program!");
        FlushAllErrors();
        return NULL;
    }

    // Build the program executable
    err = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[2048];
        clGetProgramBuildInfo(program, m_clDeviceID, CL_PROGRAM_BUILD_LOG,
                              sizeof(buffer), buffer, &len);
        cerr << buffer << endl;

        ReportError("Error: Failed to build program executable!");
        ReportError(buffer);
        FlushAllErrors();
        return NULL;
    }

	//Add to the list of programs
    ComputeProgram* compute = new ComputeProgram(program);
    m_lstPrograms.push_back(compute);
    return compute;
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
