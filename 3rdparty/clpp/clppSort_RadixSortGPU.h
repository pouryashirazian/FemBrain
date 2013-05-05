#ifndef __CLPP_SORT_RADIXSORT_GPU_H__
#define __CLPP_SORT_RADIXSORT_GPU_H__

#include "clpp/clppSort.h"
#include "clpp/clppScan.h"

class clppSort_RadixSortGPU : public clppSort
{
public:
	clppSort_RadixSortGPU(clppContext* context, unsigned int maxElements, unsigned int bits, bool keysOnly);
	~clppSort_RadixSortGPU();

	string getName() { return "Radix sort"; }

	void sort();

	void pushDatas(void* dataSet, size_t datasetSize);
	void pushCLDatas(cl_mem clBuffer_dataSet, size_t datasetSize);

	void popDatas();

	string compilePreprocess(string kernel);

private:
	bool _keysOnly;			// Key-Values or Keys-only
	size_t _datasetSize;	// The number of keys to sort

	void* _dataSetOut;
	cl_mem _clBuffer_dataSetOut;

	cl_kernel _kernel_RadixLocalSort;
	cl_kernel _kernel_LocalHistogram;
	cl_kernel _kernel_RadixPermute;	

	size_t _workgroupSize;

	unsigned int _bits;

	void radixLocal(const size_t* global, const size_t* local, cl_mem data, cl_mem hist, cl_mem blockHists, int bitOffset);
	void localHistogram(const size_t* global, const size_t* local, cl_mem data, cl_mem hist, cl_mem blockHists, int bitOffset);
	void radixPermute(const size_t* global, const size_t* local, cl_mem dataIn, cl_mem dataOut, cl_mem histScan, cl_mem blockHists, int bitOffset, unsigned int numBlocks);
	void freeUpRadixMems();

	clppScan* _scan;

	cl_mem _clBuffer_radixHist1;
	cl_mem _clBuffer_radixHist2;
	cl_mem radixDataB;

	bool _is_clBuffersOwner;
};

#endif