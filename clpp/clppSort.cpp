#include "clpp/clppSort.h"

void clppSort::pushDatas(void* dataSet, size_t datasetSize)
{
	_keySize = _valueSize = 4;
	_dataSet = dataSet;
	_datasetSize = datasetSize;

	cl_int clStatus;
	_clBuffer_dataSet = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE, _keySize * datasetSize, NULL, &clStatus);
	checkCLStatus(clStatus);

	pushCLDatas(_clBuffer_dataSet, datasetSize);
}