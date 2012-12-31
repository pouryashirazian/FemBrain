#ifndef __CLPP_SORT_H__
#define __CLPP_SORT_H__

#include "clpp/clppContext.h"
#include "clpp/clppProgram.h"

using namespace std;

/// Base class to sort a set of datas with the OpenCL Parallel Primitives library.
/// 
/// \version 1.0
class clppSort : public clppProgram
{
public:
	/// Returns the algorithm name
	virtual string getName() = 0;

	/// Sort the pushed data set 
	virtual void sort() = 0;

	/// Push the data on the device
	///
	/// \param dataSet			Pointer to the set of datas to be sorted.
	/// \param datasetSize		Number of elements to be sorted.  Must be <= maxElements passed to the constructor
	virtual void pushDatas(void* dataSet, size_t datasetSize);

	/// Push the data on the device
	///
	/// \param clBuffer_dataSet		Array of data to be sorted.
	/// \param datasetSize			Number of elements to be sorted. Must be <= maxElements passed to the constructor
	virtual void pushCLDatas(cl_mem clBuffer_dataSet, size_t datasetSize) = 0;

	/// Pop the data from the device
	virtual void popDatas() = 0;

protected:
	
	void* _dataSet;				// The associated data set to sort
	cl_mem _clBuffer_dataSet;	// The cl buffers for the values
	size_t _dataSize;			// The size of a value in bytes

	unsigned int _keySize;
	unsigned int _valueSize;

	size_t _datasetSize;	// The number of items to sort

	unsigned int _keyBits;	// The bits used by the key
};

#endif