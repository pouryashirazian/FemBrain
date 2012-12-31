#ifndef __CLPP_H__
#define __CLPP_H__

#include "clpp/clppContext.h"
#include "clpp/clppSort.h"
#include "clpp/clppScan.h"

class clpp
{
public:
	
	// Create the best scan primitive for the context and a number of elements to scan.
	static clppScan* createBestScan(clppContext* context, size_t valueSize, unsigned int maxElements);

	// Create the best sort primitive for the context and a number of elements to sort.
	static clppSort* createBestSort(clppContext* context, unsigned int maxElements, unsigned int bits);

	// Create the best sort (Key+Value) primitive for the context and a number of elements to sort.
	static clppSort* createBestSortKV(clppContext* context, unsigned int maxElements, unsigned int bits);
};

#endif