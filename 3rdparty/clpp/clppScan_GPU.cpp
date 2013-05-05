#include "clpp/clppScan_GPU.h"

#include <iostream>

// Next :
// 1 - Allow templating
// 2 - 

#define SIMT_SIZE 32

#pragma region Constructor

clppScan_GPU::clppScan_GPU(clppContext* context, size_t valueSize, unsigned int maxElements) :
	clppScan(context, valueSize, maxElements) 
{
	cl_int clStatus;
	_clBuffer_values = 0;

	//---- Compilation
	if (!compile(context, "clppScan_GPU.cl"))
		return;

	//---- Prepare all the kernels
	kernel__scan = clCreateKernel(_clProgram, "kernel__scan_block_anylength", &clStatus);
	checkCLStatus(clStatus);

	//---- Get the workgroup size
	// ATI : Actually the wavefront size is only 64 for the highend cards(48XX, 58XX, 57XX), but 32 for the middleend cards and 16 for the lowend cards.
	// NVidia : 32
	clGetKernelWorkGroupInfo(kernel__scan, _context->clDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &_workgroupSize, 0);
	//clGetKernelWorkGroupInfo(kernel__scan, _context->clDevice, CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, sizeof(size_t), &_workgroupSize, 0);

	_is_clBuffersOwner = false;
}

clppScan_GPU::~clppScan_GPU()
{
	if (_is_clBuffersOwner && _clBuffer_values)
		clReleaseMemObject(_clBuffer_values);
}

#pragma endregion

#pragma region compilePreprocess

string clppScan_GPU::compilePreprocess(string kernel)
{
	//ostringstream lines;

	//if (SIMT_SIZE == 32)
	//{
	//	lines << "#define SIMT_SIZE 32" << std::endl;
	//	lines << "#define WORKGROUP_SIZE " <<  _workgroupSize << std::endl;
	//}
	//else
	//{
	//	lines << "#define SIMT_SIZE 64" << std::endl;
	//	lines << "#define WORKGROUP_SIZE " <<  _workgroupSize << std::endl;		
	//}

	//lines << kernel << std::endl;
	//return lines.str();

	return clppScan::compilePreprocess(kernel);
}

#pragma endregion

#pragma region scan

void clppScan_GPU::scan()
{
	cl_int clStatus;

	int blockSize = _datasetSize / _workgroupSize;
	int B = blockSize * _workgroupSize;
	if ((_datasetSize % _workgroupSize) > 0) { blockSize++; };
	size_t localWorkSize = {_workgroupSize};
	size_t globalWorkSize = {toMultipleOf(_datasetSize / blockSize, _workgroupSize)};

	clStatus  = clSetKernelArg(kernel__scan, 0, _workgroupSize * _valueSize, 0);
	clStatus |= clSetKernelArg(kernel__scan, 1, sizeof(cl_mem), &_clBuffer_values);
	clStatus |= clSetKernelArg(kernel__scan, 2, sizeof(int), &B);
	clStatus |= clSetKernelArg(kernel__scan, 3, sizeof(int), &_datasetSize);
	clStatus |= clSetKernelArg(kernel__scan, 4, sizeof(int), &blockSize);

	clStatus |= clEnqueueNDRangeKernel(_context->clQueue, kernel__scan, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
	checkCLStatus(clStatus);
}


//void clppScan_GPU::scan()
//{
//	cl_int clStatus;
//
//	int multiple = _datasetSize - (_datasetSize%_workgroupSize);
//	int multipleFactor = multiple / _workgroupSize;
//
//	int B = _workgroupSize * multipleFactor;
//
//	//---- Apply the scan to each level
//	size_t localWorkSize = {_workgroupSize};
//	size_t globalWorkSize = {toMultipleOf(_datasetSize / multipleFactor, _workgroupSize)};
//
//	int delta = _datasetSize - (globalWorkSize*multipleFactor);
//
//	unsigned int passes = (float)ceil( B / ((float)_workgroupSize) );
//	if (delta > 0) passes++;
//
//	clStatus  = clSetKernelArg(kernel__scan, 0, _workgroupSize * _valueSize, 0);
//	clStatus |= clSetKernelArg(kernel__scan, 1, sizeof(cl_mem), &_clBuffer_values);
//	clStatus |= clSetKernelArg(kernel__scan, 2, sizeof(int), &B);
//	clStatus |= clSetKernelArg(kernel__scan, 3, sizeof(int), &_datasetSize);
//	clStatus |= clSetKernelArg(kernel__scan, 4, sizeof(int), &passes);
//
//	clStatus |= clEnqueueNDRangeKernel(_context->clQueue, kernel__scan, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
//	checkCLStatus(clStatus);
//}

#pragma endregion

#pragma region pushDatas

void clppScan_GPU::pushDatas(void* values, size_t datasetSize) 
{
	cl_int clStatus;

	//---- Store some values
	_values = values;
	bool reallocate = datasetSize > _datasetSize || !_is_clBuffersOwner;
	_datasetSize = datasetSize;

	//---- Copy on the device
	if (reallocate)
	{
		//---- Release
		if (_clBuffer_values)
			clReleaseMemObject(_clBuffer_values);

		//---- Allocate & copy on the device
		_clBuffer_values = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, _valueSize * _datasetSize, _values, &clStatus);
		_is_clBuffersOwner = true;
		checkCLStatus(clStatus);

		_is_clBuffersOwner = true;
	}
	else
		// Just resend
		clEnqueueWriteBuffer(_context->clQueue, _clBuffer_values, CL_FALSE, 0, _valueSize * _datasetSize, _values, 0, 0, 0);
}

void clppScan_GPU::pushDatas(cl_mem clBuffer_values, size_t datasetSize)
{
	_values = 0;

	_is_clBuffersOwner = false;

	_clBuffer_values = clBuffer_values;
	_datasetSize = datasetSize;
}

#pragma endregion

#pragma region popDatas

void clppScan_GPU::popDatas()
{
	cl_int clStatus = clEnqueueReadBuffer(_context->clQueue, _clBuffer_values, CL_TRUE, 0, _valueSize * _datasetSize, _values, 0, NULL, NULL);
	checkCLStatus(clStatus);
}

#pragma endregion