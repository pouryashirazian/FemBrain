#include "clpp/clppScan_Default.h"

// Next :
// 1 - Allow templating
// 2 - 

#pragma region Constructor

clppScan_Default::clppScan_Default(clppContext* context, size_t valueSize, unsigned int maxElements) :
	clppScan(context, valueSize, maxElements) 
{
	_clBuffer_values = 0;
	_clBuffer_BlockSums = 0;

	if (!compile(context, "clppScan_Default.cl"))
		return;

	//---- Prepare all the kernels
	cl_int clStatus;

	_kernel_Scan = clCreateKernel(_clProgram, "kernel__ExclusivePrefixScan", &clStatus);
	checkCLStatus(clStatus);

	//_kernel_ScanSmall = clCreateKernel(_clProgram, "kernel__ExclusivePrefixScanSmall", &clStatus);
	//checkCLStatus(clStatus);

	_kernel_UniformAdd = clCreateKernel(_clProgram, "kernel__UniformAdd", &clStatus);
	checkCLStatus(clStatus);

	//---- Get the workgroup size
	clGetKernelWorkGroupInfo(_kernel_Scan, _context->clDevice, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &_workgroupSize, 0);
	//_workgroupSize = 128;
	//_workgroupSize = 256;
	//_workgroupSize = 512;
	//clGetKernelWorkGroupInfo(_kernel_Scan, _context->clDevice, CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, sizeof(size_t), &_workgroupSize, 0);

	//---- Prepare all the buffers
	allocateBlockSums(maxElements);
}

clppScan_Default::~clppScan_Default()
{
	if (_is_clBuffersOwner && _clBuffer_values)
		clReleaseMemObject(_clBuffer_values);

	freeBlockSums();
}

#pragma endregion

#pragma region scan

void clppScan_Default::scan()
{
	cl_int clStatus;

	clStatus  = clSetKernelArg(_kernel_Scan, 1, _workgroupSize * _valueSize, 0);

	checkCLStatus(clStatus);
	
	//---- Apply the scan to each level
	cl_mem clValues = _clBuffer_values;
	for(unsigned int i = 0; i < _pass; i++)
	{
		size_t globalWorkSize = {toMultipleOf(_blockSumsSizes[i] / 2, _workgroupSize / 2)};
		size_t localWorkSize = {_workgroupSize / 2};

		clStatus = clSetKernelArg(_kernel_Scan, 0, sizeof(cl_mem), &clValues);
		clStatus |= clSetKernelArg(_kernel_Scan, 2, sizeof(cl_mem), &_clBuffer_BlockSums[i]);
		clStatus |= clSetKernelArg(_kernel_Scan, 3, sizeof(int), &_blockSumsSizes[i]);

		clStatus |= clEnqueueNDRangeKernel(_context->clQueue, _kernel_Scan, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		checkCLStatus(clStatus);

		clValues = _clBuffer_BlockSums[i];
    }

	//---- Uniform addition
	for(int i = _pass - 2; i >= 0; i--)
	{
		size_t globalWorkSize = {toMultipleOf(_blockSumsSizes[i] / 2, _workgroupSize / 2)};
		size_t localWorkSize = {_workgroupSize / 2};

        cl_mem dest = (i > 0) ? _clBuffer_BlockSums[i-1] : _clBuffer_values;

		clStatus = clSetKernelArg(_kernel_UniformAdd, 0, sizeof(cl_mem), &dest);
		checkCLStatus(clStatus);
		clStatus = clSetKernelArg(_kernel_UniformAdd, 1, sizeof(cl_mem), &_clBuffer_BlockSums[i]);
		checkCLStatus(clStatus);
		clStatus = clSetKernelArg(_kernel_UniformAdd, 2, sizeof(int), &_blockSumsSizes[i]);
		checkCLStatus(clStatus);

		clStatus = clEnqueueNDRangeKernel(_context->clQueue, _kernel_UniformAdd, 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
		checkCLStatus(clStatus);
    }
}

#pragma endregion

#pragma region pushDatas

void clppScan_Default::pushDatas(void* values, size_t datasetSize)
{
	cl_int clStatus;

	//---- Store some values
	_values = values;
	bool reallocate = datasetSize > _datasetSize || !_is_clBuffersOwner;
	bool recompute =  datasetSize != _datasetSize;
	_datasetSize = datasetSize;

	//---- Compute the size of the different block we can use for '_datasetSize' (can be < maxElements)
	// Compute the number of levels requested to do the scan
	if (recompute)
	{
		_pass = 0;
		unsigned int n = _datasetSize;
		do
		{
			n = (n + _workgroupSize - 1) / _workgroupSize; // round up
			_pass++;
		}
		while(n > 1);

		// Compute the block-sum sizes
		n = _datasetSize;
		for(unsigned int i = 0; i < _pass; i++)
		{
			_blockSumsSizes[i] = n;
			n = (n + _workgroupSize - 1) / _workgroupSize; // round up
		}
		_blockSumsSizes[_pass] = n;
	}

	//---- Copy on the device
	if (reallocate)
	{
		//---- Release
		if (_clBuffer_values)
			clReleaseMemObject(_clBuffer_values);

		_clBuffer_values  = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, _valueSize * _datasetSize, _values, &clStatus);
		_is_clBuffersOwner = true;
		checkCLStatus(clStatus);
	}
	else
		// Just resend
		clEnqueueWriteBuffer(_context->clQueue, _clBuffer_values, CL_FALSE, 0, _valueSize * _datasetSize, _values, 0, 0, 0);
}

void clppScan_Default::pushDatas(cl_mem clBuffer_values, size_t datasetSize)
{
	_values = 0;
	_clBuffer_values = clBuffer_values;
	bool recompute =  datasetSize != _datasetSize;
	_datasetSize = datasetSize;

	//---- Compute the size of the different block we can use for '_datasetSize' (can be < maxElements)
	// Compute the number of levels requested to do the scan
	if (recompute)
	{
		_pass = 0;
		unsigned int n = _datasetSize;
		do
		{
			n = (n + _workgroupSize - 1) / _workgroupSize; // round up
			_pass++;
		}
		while(n > 1);

		// Compute the block-sum sizes
		n = _datasetSize;
		for(unsigned int i = 0; i < _pass; i++)
		{
			_blockSumsSizes[i] = n;
			n = (n + _workgroupSize - 1) / _workgroupSize; // round up
		}
		_blockSumsSizes[_pass] = n;
	}

	_is_clBuffersOwner = false;
}

#pragma endregion

#pragma region popDatas

void clppScan_Default::popDatas()
{
	cl_int clStatus = clEnqueueReadBuffer(_context->clQueue, _clBuffer_values, CL_TRUE, 0, _valueSize * _datasetSize, _values, 0, NULL, NULL);
	checkCLStatus(clStatus);
}

#pragma endregion

#pragma region allocateBlockSums

void clppScan_Default::allocateBlockSums(unsigned int maxElements)
{
	// Compute the number of buffers we need for the scan
	cl_int clStatus;
	_pass = 0;
	unsigned int n = maxElements;
	do
	{
		n = (n + _workgroupSize - 1) / _workgroupSize; // round up
		_pass++;
	}
	while(n > 1);

	// Allocate the arrays
	_clBuffer_BlockSums = new cl_mem[_pass];
	_blockSumsSizes = new unsigned int[_pass + 1];

	// Create the cl-buffers
	n = maxElements;
	for(unsigned int i = 0; i < _pass; i++)
	{
		_blockSumsSizes[i] = n;

		_clBuffer_BlockSums[i] = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE, sizeof(int) * n, NULL, &clStatus);
		checkCLStatus(clStatus);

		n = (n + _workgroupSize - 1) / _workgroupSize; // round up
	}
	_blockSumsSizes[_pass] = n;

	checkCLStatus(clStatus);
}

void clppScan_Default::freeBlockSums()
{
	if (!_clBuffer_BlockSums)
		return;

    cl_int clStatus;
    
	for(unsigned int i = 0; i < _pass; i++)
		clStatus = clReleaseMemObject(_clBuffer_BlockSums[i]);

	delete [] _clBuffer_BlockSums;
	delete [] _blockSumsSizes;
	_clBuffer_BlockSums = 0;
	_blockSumsSizes = 0;
}

#pragma endregion