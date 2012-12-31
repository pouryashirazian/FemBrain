#include "clpp/clppContext.h"

#include<assert.h>
#include <iostream>
#include <string.h>

using namespace std;

void clppContext::setup()
{
	setup(0, 0);
}

void clppContext::setup(unsigned int platformId, unsigned int deviceId)
{
	isGPU = isCPU = false;
	Vendor = Vendor_Unknown;

	cl_int clStatus;

	size_t infoLen;
	char infoStr[1024];
	cl_device_type infoType;

	//---- Retreive information about platforms
	cl_uint platformsCount;
	clStatus = clGetPlatformIDs(0, NULL, &platformsCount);
	assert(clStatus == CL_SUCCESS);

	cl_platform_id* platforms = new cl_platform_id[platformsCount];
	clStatus = clGetPlatformIDs(platformsCount, platforms, NULL);
	assert(clStatus == CL_SUCCESS);

	platformId = min(platformId, platformsCount - 1);
	clPlatform = platforms[platformId];

	clGetPlatformInfo (clPlatform, CL_PLATFORM_VENDOR, sizeof(infoStr), infoStr, &infoLen);
	//clGetPlatformInfo (clPlatform, CL_DEVICE_VENDOR, sizeof(infoStr), infoStr, &infoLen);
	if (stristr(infoStr, "Intel") != NULL)
		Vendor = Vendor_Intel;
	else if (stristr(infoStr, "AMD") != NULL)
		Vendor = Vendor_AMD;
	else if (stristr(infoStr, "NVidia") != NULL)
		Vendor = Vendor_NVidia;
	else if (stristr(infoStr, "Apple") != NULL)
		Vendor = Vendor_NVidia;

	//---- Devices
	cl_uint devicesCount;
	clStatus = clGetDeviceIDs(clPlatform, CL_DEVICE_TYPE_ALL, 0, NULL, &devicesCount);
	assert(clStatus == CL_SUCCESS);
	assert(devicesCount > 0);
	
	cl_device_id* devices = new cl_device_id[devicesCount];
	clStatus = clGetDeviceIDs(clPlatform, CL_DEVICE_TYPE_ALL, devicesCount, devices, NULL);
	assert(clStatus == CL_SUCCESS);

	clDevice = devices[min(deviceId, devicesCount - 1)];
		
	clGetDeviceInfo(clDevice, CL_DEVICE_TYPE, sizeof(infoType), &infoType, &infoLen);
	if (infoType & CL_DEVICE_TYPE_CPU)
		isCPU = true;
	if (infoType & CL_DEVICE_TYPE_GPU)
		isGPU = true;

	//---- Context
	clContext = clCreateContext(0, 1, &clDevice, NULL, NULL, &clStatus);
	assert(clStatus == CL_SUCCESS);

	//---- Queue
	clQueue = clCreateCommandQueue(clContext, clDevice, CL_QUEUE_PROFILING_ENABLE, &clStatus);
	assert(clStatus == CL_SUCCESS);

	//---- Display some info about the context
	char platformName[500];
	clGetPlatformInfo(clPlatform, CL_PLATFORM_NAME, 500, platformName, NULL);

	cl_device_type deviceType;
	clStatus = clGetDeviceInfo(clDevice, CL_DEVICE_TYPE, sizeof(cl_device_type), (void*)&deviceType,NULL);
	
	char deviceName[500];
	clStatus = clGetDeviceInfo(clDevice, CL_DEVICE_NAME, 500, deviceName, NULL);

	cout << "Platform[" << platformName << "] Device[" << deviceName << "]" << endl << endl<< endl;
}

char* clppContext::stristr(const char *String, const char *Pattern)
{
    char *pptr, *sptr, *start;
    unsigned int slen, plen;
    for (start = (char *)String,
            pptr = (char *)Pattern,
            slen = strlen(String),
            plen = strlen(Pattern);
            // while string length not shorter than pattern length
            slen >= plen;
            start++, slen--)
    {
        // find start of pattern in string
        while (toupper(*start) != toupper(*Pattern))
        {
            start++;
            slen--;
            // if pattern longer than string
            if (slen < plen)
                return(NULL);
        }
        sptr = start;
        pptr = (char *)Pattern;
        while (toupper(*sptr) == toupper(*pptr))
        {
            sptr++;
            pptr++;
            // if end of pattern then pattern was found
            if ('\0' == *pptr)
                return (start);
        }
    }
    return(NULL);
}

int clppContext::GetSIMTCapability()
{
	// NVidia : 32
	if (Vendor == Vendor_NVidia && isGPU)
		return 32;

	if (Vendor == Vendor_AMD && isGPU)
	{
		// ATI : Actually the wavefront size is only 64 for the highend cards(48XX, 58XX, 57XX), but 32 for the middleend cards and 16 for the lowend cards.	
		//clGetKernelWorkGroupInfo(kernel__scan, _context->clDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &_workgroupSize, 0);
		//clGetKernelWorkGroupInfo(kernel__scan, _context->clDevice, CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, sizeof(size_t), &_workgroupSize, 0);

		// http://developer.amd.com/sdks/amdappsdk/assets/AMD_Accelerated_Parallel_Processing_OpenCL_Programming_Guide.pdf
		size_t returned_size;
		size_t max_work_group_size;
		clGetDeviceInfo(clDevice, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(max_work_group_size), &max_work_group_size, &returned_size);
		if (max_work_group_size == 256)
			return 64;

		return 32;
	}

	return 1;
}
