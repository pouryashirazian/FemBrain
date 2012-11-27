/*
 * OclRayTracer.cpp
 *
 *  Created on: Nov 24, 2012
 *      Author: pourya
 */
#include "OclRayTracer.h"
#include "../PS_Base/PS_Logger.h"
#include "../PS_Base/PS_FileDirectory.h"
#include "lodepng.h"

using namespace PS;
using namespace PS::FILESTRINGUTILS;

namespace PS{
namespace HPC{

RayTracer::RayTracer(){
	setup(DEFAULT_SURFACE_WIDTH, DEFAULT_SURFACE_HEIGHT);
}

RayTracer::RayTracer(int w, int h)
{
	setup(w, h);
}

RayTracer::~RayTracer(){
	cleanup();
}

void RayTracer::setup(int w, int h)
{
	m_lpDevice = new ComputeDevice(ComputeDevice::dtGPU, true, "AMD");
	m_lpDevice->printInfo();

	DAnsiStr strFP = ExtractFilePath(GetExePath());
	strFP = ExtractOneLevelUp(strFP);
	strFP += DAnsiStr("PS_Shaders/Raytracer.cl");

	//Create the OCL Scan Primitive
	//PS::HPC::Scan* lpScanner = new PS::HPC::Scan(lpGPU);
	//lpScanner->scanExclusiveLarge()
	LogInfo("2.Compile OpenCL program.");
	ComputeProgram* lpProgram = m_lpDevice->addProgramFromFile(strFP.cptr());
	assert(lpProgram != NULL);

	//Build Kernel
	LogInfo("3.Setup Kernel Functions.");
	m_lpKernelPrimary = lpProgram->addKernel("RayTrace");

	//Create a Surface
	m_screenDim = vec2i(w, h);
	m_lpSurface = new GLSurface(w, h);
}

void RayTracer::cleanup(){
	SAFE_DELETE(m_lpDevice);
	SAFE_DELETE(m_lpSurface);
}

bool RayTracer::run(){

	cl_image_format imageFormat;
	imageFormat.image_channel_order = CL_RGBA;
	imageFormat.image_channel_data_type = CL_FLOAT;

#ifdef CL_API_SUFFIX__VERSION_1_2
	cl_int errNum;
	cl_image_desc desc;
	desc.image_type = CL_MEM_OBJECT_IMAGE2D;
	desc.image_width = m_screenDim.x;
	desc.image_height = m_screenDim.y;
	desc.image_row_pitch = 0;
	desc.num_mip_levels = 0;
	desc.num_samples = 0;
	desc.buffer = NULL;

	//CL_INVALID_VALUE
	cl_mem outMemFrameBuffer = clCreateImage(m_lpDevice->getContext(),
			CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR, &imageFormat, &desc,
			NULL, &errNum);
	if (errNum != CL_SUCCESS) {
		char buffer[1024];
		sprintf(buffer, "Error: Failed to create image! (%s)",
				ComputeDevice::oclErrorString(errNum));
		LogError(buffer);
		cerr << buffer << endl;
		return false;
	}
#else
	cl_int errNum;
	cl_mem outMemFrameBuffer = clCreateImage2D(m_lpDevice->getContext(), CL_MEM_WRITE_ONLY,
			&imageFormat, m_screenDim.x, m_screenDim.y, 0, NULL, &errNum);
	if(errNum != CL_SUCCESS)
	return false;
#endif

	m_lpKernelPrimary->setArg(0, sizeof(cl_mem), &outMemFrameBuffer);

	size_t local = m_lpDevice->getKernelWorkgroupSize(m_lpKernelPrimary);
	size_t szDim[3];
	szDim[0] = m_screenDim.x;
	szDim[1] = m_screenDim.y;
	szDim[2] = 1;


	//Run Config Kernel to classify the cells
	cl_int err = clEnqueueNDRangeKernel(m_lpDevice->getCommandQ(), m_lpKernelPrimary->getKernel(),
										2, NULL, szDim, NULL, 0, NULL, NULL);
	if (err) {
		LogErrorArg1("Error: Failed to execute RayTrace kernel! (%s)", ComputeDevice::oclErrorString(err));
		return false;
	}

	// Wait for all commands to complete
	m_lpDevice->finishAllCommands();

	size_t origin[3] = {0, 0, 0};
	const U32 ctPixels = m_screenDim.x * m_screenDim.y;
	float* lpImageData = new float[ctPixels * 4];
	err = clEnqueueReadImage(m_lpDevice->getCommandQ(), outMemFrameBuffer,
								CL_TRUE, origin, szDim, 0, 0, lpImageData, 0, NULL, NULL);
	if (err) {
		LogErrorArg1("Error: Failed to readback RayTraced image! (%s)", ComputeDevice::oclErrorString(err));
		SAFE_DELETE_ARRAY(lpImageData);
		return false;
	}

	//Save as png
	DAnsiStr strFP = ExtractFilePath(GetExePath()) + "raytracer_output.png";
	std::vector<U8> arrImageData;
	//Width x Height x RGBA pixels
	arrImageData.resize(m_screenDim.x * m_screenDim.y * 4);
	for(U32 i=0; i<ctPixels; i++)
	{
		for(int j=0; j<4; j++)
		{
			float v = lpImageData[i * 4 + j];
			arrImageData[i * 4 + j] = v * 255.0f;
		}

	}
	SAFE_DELETE_ARRAY(lpImageData);

	//Encode and save
	U32 error = lodepng::encode(string(strFP.cptr()), arrImageData, m_screenDim.x, m_screenDim.y);
	if(error)
	{
		LogErrorArg1("Error: Encode RayTraced image! (%s)", lodepng_error_text(error));
		return false;
	}

	arrImageData.resize(0);

	return true;
}


}
}




