#ifndef __CLPP_SCAN_DEFAULT_H__
#define __CLPP_SCAN_DEFAULT_H__

#include "clpp/clppScan.h"

class clppScan_Default : public clppScan
{
public:
	clppScan_Default(clppContext* context, size_t valueSize, unsigned int maxElements);
	~clppScan_Default();

	string getName() { return "Prefix sum (exclusive)"; }

	void scan();

	void pushDatas(void* values, size_t datasetSize);
	void pushDatas(cl_mem clBuffer_values, size_t datasetSize);

	void popDatas();

private:
	cl_kernel _kernel_Scan;
	cl_kernel _kernel_ScanSmall;
	cl_kernel _kernel_UniformAdd;

	cl_mem _clBuffer_Temp;

	unsigned int* _temp;

	int _pass;
	cl_mem* _clBuffer_BlockSums;
	unsigned int* _blockSumsSizes;

	void allocateBlockSums(unsigned int maxElements);
	void freeBlockSums();
};

#endif