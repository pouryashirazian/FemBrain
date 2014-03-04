#ifndef __CLPP_PROGRAM_H__
#define __CLPP_PROGRAM_H__

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <assert.h>
#include <math.h>

#if defined (__APPLE__) || defined(MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif

#include "clpp/clppContext.h"

//enum clppVendor { Vendor_Unknown, Vendor_NVidia, Vendor_AMD, Vendor_Intel };


using namespace std;

class clppProgram
{
public:
	clppProgram();
	virtual ~clppProgram();

	bool compile(clppContext* context, string fileName);
	virtual string compilePreprocess(string programSource);

	// Wait for the end of the program
	virtual void waitCompletion();

	// Helper method : use to retreive textual error message
	static void checkCLStatus(cl_int clStatus);

	// Load the cl source code
	static string loadSource(string path);

	// Set/Get the base path for all the OpenCL kernels.
	static string getBasePath();
	static void setBasePath(string basePath);

protected:
	cl_program _clProgram;
	clppContext* _context;

	static string _basePath;

protected:
	static const char* getOpenCLErrorString(cl_int err);

	static size_t toMultipleOf(size_t N, size_t base) 
	{
		return (ceil((double)N / (double)base) * base);
	}
};

#endif
