#ifndef __CLPP_SORT_BLELLOCH_H__
#define __CLPP_SORT_BLELLOCH_H__

#include<math.h>

#include "clpp/clppSort.h"

#define _TOTALBITS 25  // number of bits for the integer in the list (max=32)
#define _BITS 5  // number of bits in the radix

// these parameters can be changed
#define _ITEMS  32// number of items in a group
#define _GROUPS 32 // the number of virtual processors is _ITEMS * _GROUPS
#define  _HISTOSPLIT 512 // number of splits of the histogram

// max size of the sorted vector
// it has to be divisible by  _ITEMS * _GROUPS
// (for other sizes, pad the list with big values)
//#define _N (_ITEMS * _GROUPS * 16)  
//#define _N (1<<23)  // maximal size of the list  
#define _N (1 << 21)  // maximal size of the list  

#define PERMUT 0  // store the final permutation
#define TRANSPOSE

// the following parameters are computed from the previous
#define _RADIX (1 << _BITS) //  radix  = 2^_BITS
#define _PASS (_TOTALBITS/_BITS) // number of needed passes to sort the list
#define _HISTOSIZE (_ITEMS * _GROUPS * _RADIX ) // size of the histogram

// maximal value of integers for the sort to be correct
#define _MAXINT (1 << (_TOTALBITS-1))

class clppSort_Blelloch : public clppSort
{
public:
	clppSort_Blelloch(clppContext* context, unsigned int maxElements);

	string compilePreprocess(string kernel);

	string getName() { return "Blelloch"; }

	void sort();

	void pushCLDatas(cl_mem clBuffer_dataSet, size_t datasetSize);

	void popDatas();

private:
	cl_kernel _kernel_Histogram;
	cl_kernel _kernel_ScanHistogram;
	cl_kernel _kernel_PasteHistogram;
	cl_kernel _kernel_Reorder;
	cl_kernel _kernel_Transpose;

	int _permutations[_N];

	cl_mem _clBuffer_outKeys;
	cl_mem _clBuffer_inPermutations;
	cl_mem _clBuffer_outPermutations;
	cl_mem _clBuffer_Histograms;
	cl_mem _clBuffer_globsum;
	cl_mem _clBuffer_temp;

	// Resize the sorted vector
	void resize(int nn);

	// Transpose the list for faster memory access
	void transpose(int nbrow, int nbcol);

	void histogram(int pass);

	void scanHistogram();

	// Reorder the data from the scanned histogram
	void reorder(int pass);

	int nkeys;
	int nkeys_rounded;

	// timers
	float _timerHisto;
	float _timerScan;
	float _timerReorder;
	float _timerSort;
	float _timerTranspose;
};

#endif
