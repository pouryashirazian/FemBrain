/*
 * PS_OclSumScan.h
 *
 *  Created on: Dec 28, 2012
 *      Author: pourya
 */

#ifndef PS_OCLSUMSCAN_H_
#define PS_OCLSUMSCAN_H_

#include <vector>
#include "ComputeDevice.h"
#include "clpp/clpp.h"

using namespace std;

namespace PS{
namespace CL{

/*!
 *	Scans the input array and computes the sum scan
 */
class SumScan{
public:
	SumScan();

	/*!
	 * Replaces the input array with it sum scan.
	 * @return total sum
	 */
	U32 compute(U32* arrData, U32 count);

	//Get Data
	U32 compute(cl_mem inMemData, U32 count);

private:
	clppContext m_context;
};

/*
class OclSumScan{
public:
	OclSumScan(ComputeDevice* lpDevice);
	virtual ~OclSumScan();


	int compute(cl_mem inMemBuffer, cl_mem outMemBuffer, U32 count, float* lpInData);

private:

	inline int floorPow2(int n)
	{
	    int exp;
	    frexp((float)n, &exp);
	    return 1 << (exp - 1);
	}


	void scanReference( float* reference, float* input, const unsigned int count);

	int createPartialSumBuffers(U32 count);

	void releasePartialSums();

	int PreScan(
	    size_t *global,
	    size_t *local,
	    size_t shared,
	    cl_mem output_data,
	    cl_mem input_data,
	    unsigned int n,
	    int group_index,
	    int base_index);

	int
	PreScanStoreSum(
	    size_t *global,
	    size_t *local,
	    size_t shared,
	    cl_mem output_data,
	    cl_mem input_data,
	    cl_mem partial_sums,
	    unsigned int n,
	    int group_index,
	    int base_index);

	int PreScanStoreSumNonPowerOfTwo(
	    size_t *global,
	    size_t *local,
	    size_t shared,
	    cl_mem output_data,
	    cl_mem input_data,
	    cl_mem partial_sums,
	    unsigned int n,
	    int group_index,
	    int base_index);

	int PreScanNonPowerOfTwo(
	    size_t *global,
	    size_t *local,
	    size_t shared,
	    cl_mem output_data,
	    cl_mem input_data,
	    unsigned int n,
	    int group_index,
	    int base_index);

	int
	UniformAdd(
	    size_t *global,
	    size_t *local,
	    cl_mem output_data,
	    cl_mem partial_sums,
	    unsigned int n,
	    unsigned int group_offset,
	    unsigned int base_index);


	int PreScanBufferRecursive(
	    cl_mem output_data,
	    cl_mem input_data,
	    int max_group_size,
	    int max_work_item_count,
	    int element_count,
	    int level);

	void	PreScanBuffer(
	    cl_mem output_data,
	    cl_mem input_data,
	    unsigned int max_group_size,
	    unsigned int max_work_item_count,
	    unsigned int element_count);

private:
	U32	m_ctElementsAllocated;
	U32 m_ctLevelsAllocated;
	int m_szKernelGroup;

	cl_mem*	m_lpMemScanPartialSums;
	ComputeDevice* m_lpDevice;
	ComputeProgram* m_lpProgram;
	vector<ComputeKernel*> m_vKernels;

};
*/
}
}


#endif /* PS_OCLSUMSCAN_H_ */
