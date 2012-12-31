#ifndef __CLPP_SCAN_H__
#define __CLPP_SCAN_H__

#include "clpp/clppProgram.h"

class clppScan : public clppProgram
{
public:
	
	// Create a new scan.
	// maxElements : the maximum number of elements to scan.
	clppScan(clppContext* context, size_t valueSize, unsigned int maxElements)
	{
		_values = 0;
		_context = context;
		_valueSize = valueSize;
		_datasetSize = 0;
		_clBuffer_values = 0;
		_workgroupSize = 0;
		_is_clBuffersOwner = false;
	}

	// Returns the algorithm name
	virtual string getName() = 0;

	// Start the scan operation
	virtual void scan() = 0;

	// Send a Host data set to the device
	virtual void pushDatas(void* values, size_t datasetSize) = 0;

	// Push a buffer that is already on the device side. (Data are not sended)
	virtual void pushDatas(cl_mem clBuffer_values, size_t datasetSize) = 0;

	// Retreive the datas (To the same zone as the source).
	virtual void popDatas() = 0;

protected:
	size_t _datasetSize;	// The number of values to scan

	void* _values;			// The associated data set to scan
	size_t _valueSize;		// The size of a value in bytes

	cl_mem _clBuffer_values;
	bool _is_clBuffersOwner;

	size_t _workgroupSize;
};

#endif