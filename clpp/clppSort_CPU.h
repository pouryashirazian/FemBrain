#ifndef __CLPP_SORT_CPU_H__
#define __CLPP_SORT_CPU_H__

#include "clpp/clppSort.h"

class clppSort_CPU : public clppSort
{
public:
	clppSort_CPU(clppContext* context);
	~clppSort_CPU();

	string getName() { return "CPU Brute force"; }

	void sort();

	void pushDatas(void* dataSet, size_t datasetSize);
	void pushCLDatas(cl_mem clBuffer_dataSet, size_t datasetSize);

	void popDatas();

	void waitCompletion() {}
};

#endif