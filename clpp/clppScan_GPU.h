#ifndef __CLPP_SCAN_GPU_H__
#define __CLPP_SCAN_GPU_H__

#include "clpp/clppScan.h"

class clppScan_GPU : public clppScan
{
public:
	clppScan_GPU(clppContext* context, size_t valueSize, unsigned int maxElements);
	~clppScan_GPU();

	string getName() { return "Prefix sum (exclusive) for the GPU"; }

	void scan();

	void pushDatas(void* values, size_t datasetSize);
	void pushDatas(cl_mem clBuffer_values, size_t datasetSize);

	void popDatas();

	string compilePreprocess(string kernel);

private:
	cl_kernel kernel__scan;
};

#endif