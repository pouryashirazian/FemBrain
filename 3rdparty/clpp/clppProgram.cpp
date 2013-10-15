#include "clpp/clppProgram.h"

#ifdef WIN32
#include <windows.h>
#endif

#if defined(__linux__) || defined(__APPLE__)
#include <sys/time.h>
#endif

string clppProgram::_basePath;

clppProgram::clppProgram()
{
	_clProgram = 0;
	_context = 0;
}

clppProgram::~clppProgram()
{
    cl_int clStatus;

	if (_clProgram)
	{
		clStatus = clReleaseProgram(_clProgram);
		checkCLStatus(clStatus);
	}
}

string clppProgram::getBasePath()
{
	return _basePath;
}

void clppProgram::setBasePath(string basePath)
{
	_basePath = basePath;
}

bool clppProgram::compile(clppContext* context, string fileName)
{
	cl_int clStatus;

	_context = context;

	//printf("BASE PATH = %s\n", _basePath.c_str());
	string programSource = loadSource(_basePath + fileName);

	//---- Some preprocessing
	programSource = compilePreprocess(programSource);

	//---- Build the program
	const char* ptr = programSource.c_str();
	size_t len = programSource.length();
	_clProgram = clCreateProgramWithSource(context->clContext, 1, (const char **)&ptr, &len, &clStatus);
	checkCLStatus(clStatus);

#ifdef __APPLE__
    const char* buildOptions = "-DMAC -cl-fast-relaxed-math";
#else
    //const char* buildOptions = "-cl-fast-relaxed-math";
	const char* buildOptions = "";
#endif

	clStatus = clBuildProgram(_clProgram, 0, NULL, NULL, NULL, NULL);
  
	if (clStatus != CL_SUCCESS)
	{
		size_t len;
		char buffer[5000];
		printf("Error: Failed to build program executable!\n");
		clGetProgramBuildInfo(_clProgram, context->clDevice, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);

		printf("%s\n", buffer);
		printf("%s\n", getOpenCLErrorString(clStatus));

		checkCLStatus(clStatus);
		return false;
	}

	return true;
}

string clppProgram::compilePreprocess(string programSource)
{
	string source = "";
	if (_context->Vendor == Vendor_AMD)
		source += "#define OCL_PLATFORM_AMD\n";
	else if (_context->Vendor == Vendor_Intel)
		source += "#define OCL_PLATFORM_INTEL\n";
	else if (_context->Vendor == Vendor_NVidia)
		source += "#define OCL_PLATFORM_NVIDIA\n";
	else
		source += "#define OCL_PLATFORM_UNKNOW\n";

	if (_context->isGPU)
		source += "#define OCL_DEVICE_GPU\n";
	else if (_context->isCPU)
		source += "#define OCL_DEVICE_CPU\n";

	return source + programSource;
}

string clppProgram::loadSource(string path)
{
	string kernel = "";
	char buffer[5000];
	ifstream infile(path.c_str(), ios_base::in | ios_base::binary);

	if (!infile)
        return "";

	while(!infile.eof())
	{
		infile.getline(buffer, 5000);

		string text(buffer);
		kernel += text + "\n";
	}
	infile.close();

	return kernel;
}

const char* clppProgram::getOpenCLErrorString(cl_int err)
{
	switch (err) {
		case CL_SUCCESS: return "Success!";
		case CL_DEVICE_NOT_FOUND: return "Device not found.";
		case CL_DEVICE_NOT_AVAILABLE: return "Device not available";
		case CL_COMPILER_NOT_AVAILABLE: return "Compiler not available";
		case CL_MEM_OBJECT_ALLOCATION_FAILURE: return "Memory object allocation failure";
		case CL_OUT_OF_RESOURCES: return "Out of resources";
		case CL_OUT_OF_HOST_MEMORY: return "Out of host memory";
		case CL_PROFILING_INFO_NOT_AVAILABLE: return "Profiling information not available";
		case CL_MEM_COPY_OVERLAP: return "Memory copy overlap";
		case CL_IMAGE_FORMAT_MISMATCH: return "Image format mismatch";
		case CL_IMAGE_FORMAT_NOT_SUPPORTED: return "Image format not supported";
		case CL_BUILD_PROGRAM_FAILURE: return "Program build failure";
		case CL_MAP_FAILURE: return "Map failure";
		case CL_INVALID_VALUE: return "Invalid value";
		case CL_INVALID_DEVICE_TYPE: return "Invalid device type";
		case CL_INVALID_PLATFORM: return "Invalid platform";
		case CL_INVALID_DEVICE: return "Invalid device";
		case CL_INVALID_CONTEXT: return "Invalid context";
		case CL_INVALID_QUEUE_PROPERTIES: return "Invalid queue properties";
		case CL_INVALID_COMMAND_QUEUE: return "Invalid command queue";
		case CL_INVALID_HOST_PTR: return "Invalid host pointer";
		case CL_INVALID_MEM_OBJECT: return "Invalid memory object";
		case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR: return "Invalid image format descriptor";
		case CL_INVALID_IMAGE_SIZE: return "Invalid image size";
		case CL_INVALID_SAMPLER: return "Invalid sampler";
		case CL_INVALID_BINARY: return "Invalid binary";
		case CL_INVALID_BUILD_OPTIONS: return "Invalid build options";
		case CL_INVALID_PROGRAM: return "Invalid program";
		case CL_INVALID_PROGRAM_EXECUTABLE: return "Invalid program executable";
		case CL_INVALID_KERNEL_NAME: return "Invalid kernel name";
		case CL_INVALID_KERNEL_DEFINITION: return "Invalid kernel definition";
		case CL_INVALID_KERNEL: return "Invalid kernel";
		case CL_INVALID_ARG_INDEX: return "Invalid argument index";
		case CL_INVALID_ARG_VALUE: return "Invalid argument value";
		case CL_INVALID_ARG_SIZE: return "Invalid argument size";
		case CL_INVALID_KERNEL_ARGS: return "Invalid kernel arguments";
		case CL_INVALID_WORK_DIMENSION: return "Invalid work dimension";
		case CL_INVALID_WORK_GROUP_SIZE: return "Invalid work group size";
		case CL_INVALID_WORK_ITEM_SIZE: return "Invalid work item size";
		case CL_INVALID_GLOBAL_OFFSET: return "Invalid global offset";
		case CL_INVALID_EVENT_WAIT_LIST: return "Invalid event wait list";
		case CL_INVALID_EVENT: return "Invalid event";
		case CL_INVALID_OPERATION: return "Invalid operation";
		case CL_INVALID_GL_OBJECT: return "Invalid OpenGL object";
		case CL_INVALID_BUFFER_SIZE: return "Invalid buffer size";
		case CL_INVALID_MIP_LEVEL: return "Invalid mip-map level";
		default: return "Unknown";
	}
}

void clppProgram::checkCLStatus(cl_int clStatus)
{
	const char* e = getOpenCLErrorString(clStatus);
	assert(clStatus == CL_SUCCESS);
}

void clppProgram::waitCompletion()
{
	clFinish(_context->clQueue);
}
