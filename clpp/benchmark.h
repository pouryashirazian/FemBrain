#ifndef __CLPP_BENCHMARK_H__
#define __CLPP_BENCHMARK_H__

// Base class to sort a set of datas with clpp
class clppSort_Blelloch
{
	void sort(void* keys, void* values, unsigned int keyBits, size_t numElements);
};

#endif