/*
 * PS_OclSumScan.cpp
 *
 *  Created on: Dec 28, 2012
 *      Author: pourya
 */
#include <iostream>
#include <stdio.h>
#include "OclSumScan.h"
#include "base/FileDirectory.h"

#include "clpp/clppScan.h"
#include "clpp/clppScan_Default.h"


using namespace std;
using namespace PS::FILESTRINGUTILS;

////////////////////////////////////////////////////////////////////////////////////////////////////

#define DEBUG_INFO      (0)
#define NUM_BANKS       (16)
#define MAX_ERROR       (1e-7)
#define SEPARATOR       ("----------------------------------------------------------------------\n")
#define min(A,B) ((A) < (B) ? (A) : (B))

////////////////////////////////////////////////////////////////////////////////////////////////////
static int iterations = 1000;
static int count      = 1024 * 1024;
////////////////////////////////////////////////////////////////////////////////////////////////////

enum KernelMethods
{
    PRESCAN                             = 0,
    PRESCAN_STORE_SUM                   = 1,
    PRESCAN_STORE_SUM_NON_POWER_OF_TWO  = 2,
    PRESCAN_NON_POWER_OF_TWO            = 3,
    UNIFORM_ADD                         = 4
};

static const char* KernelNames[] =
{
    "PreScanKernel",
    "PreScanStoreSumKernel",
    "PreScanStoreSumNonPowerOfTwoKernel",
    "PreScanNonPowerOfTwoKernel",
    "UniformAddKernel"
};

static const unsigned int KernelCount = sizeof(KernelNames) / sizeof(char *);

namespace PS {
namespace CL {

SumScan::SumScan(){

	//Load Kernel
	AnsiStr strFP = ExtractOneLevelUp(ExtractFilePath(GetExePath())) + "data/opencl/clpp/";
	clppProgram::setBasePath(string(strFP.cptr()));

	//Setup context
	m_context.setup(0, 0);
}

U32 SumScan::compute(U32* arrData, U32 count)
{
	U32 lastElement = arrData[count - 1];

	//SumScan
	clppScan* lpSumScan =  new clppScan_Default(&m_context, sizeof(U32), count);
	lpSumScan->pushDatas(arrData, count);
	lpSumScan->scan();
	lpSumScan->waitCompletion();
	lpSumScan->popDatas();

	SAFE_DELETE(lpSumScan);

	return arrData[count-1] + lastElement;
}

U32 SumScan::compute(cl_mem inMemData, U32 count)
{
	//SumScan
	clppScan* lpSumScan =  new clppScan_Default(&m_context, sizeof(U32), count);
	lpSumScan->pushDatas(inMemData, count);
	lpSumScan->scan();
	lpSumScan->waitCompletion();
	lpSumScan->popDatas();

	SAFE_DELETE(lpSumScan);

	return 1;
}

/*
OclSumScan::OclSumScan(ComputeDevice* lpDevice)
{
	m_lpDevice = lpDevice;

	//Reading kernels for opencl
	AnsiStr strFP = ExtractFilePath(GetExePath());
	strFP = ExtractOneLevelUp(strFP);
	strFP += AnsiStr("PS_Shaders/scan_kernel.cl");

	m_lpProgram =  m_lpDevice->addProgramFromFile(strFP.cptr());
	assert(m_lpProgram != NULL);

	m_ctElementsAllocated = 0;
	m_ctLevelsAllocated = 0;

	m_szKernelGroup = 256;
	m_vKernels.resize(KernelCount);
	for (U32 i = 0; i < KernelCount; i++)
	{
		m_vKernels[i] = m_lpProgram->addKernel(KernelNames[i]);

		if(m_vKernels[i])
		{
			size_t szKernelWG = m_lpDevice->getKernelWorkgroupSize(m_vKernels[i]);
			m_szKernelGroup = MATHMIN(m_szKernelGroup, szKernelWG );
		}
	}
}

OclSumScan::~OclSumScan(){
	m_lpDevice->removeProgram(m_lpProgram);
	SAFE_DELETE(m_lpProgram);
	m_vKernels.resize(0);
}


int OclSumScan::createPartialSumBuffers(U32 count)
{
    m_ctElementsAllocated = count;
    unsigned int element_count = count;

    int level = 0;

    do
    {
        unsigned int group_count = (int)fmax(1, (int)ceil((float)element_count / (2.0f * m_szKernelGroup)));
        if (group_count > 1)
        {
            level++;
        }
        element_count = group_count;

    } while (element_count > 1);

    m_lpMemScanPartialSums = (cl_mem*) malloc(level * sizeof(cl_mem));
    m_ctLevelsAllocated = level;
    memset(m_lpMemScanPartialSums, 0, sizeof(cl_mem) * level);

    element_count = count;
    level = 0;

    do
    {
        U32 group_count = (int)fmax(1, (int)ceil((float)element_count / (2.0f * m_szKernelGroup)));
        if (group_count > 1)
        {
            size_t buffer_size = group_count * sizeof(float);
            m_lpMemScanPartialSums[level++] = m_lpDevice->createMemBuffer(buffer_size, ComputeDevice::memReadWrite);
        }

        element_count = group_count;

    } while (element_count > 1);

    return CL_SUCCESS;
}

void OclSumScan::releasePartialSums()
{
    unsigned int i;
    for (i = 0; i < m_ctLevelsAllocated; i++)
    {
        clReleaseMemObject(m_lpMemScanPartialSums[i]);
    }

    free(m_lpMemScanPartialSums);
    m_lpMemScanPartialSums = 0;
    m_ctElementsAllocated = 0;
    m_ctLevelsAllocated = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int OclSumScan::PreScan(
    size_t *global,
    size_t *local,
    size_t shared,
    cl_mem output_data,
    cl_mem input_data,
    unsigned int n,
    int group_index,
    int base_index)
{
#if DEBUG_INFO
    printf("PreScan: Global[%4d] Local[%4d] Shared[%4d] BlockIndex[%4d] BaseIndex[%4d] Entries[%d]\n",
        (int)global[0], (int)local[0], (int)shared, group_index, base_index, n);
#endif

    unsigned int k = PRESCAN;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &output_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, shared,         0);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &group_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(m_lpDevice->getCommandQ(), m_vKernels[k]->getKernel(), 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    return CL_SUCCESS;
}

int OclSumScan::PreScanStoreSum(
    size_t *global,
    size_t *local,
    size_t shared,
    cl_mem output_data,
    cl_mem input_data,
    cl_mem partial_sums,
    unsigned int n,
    int group_index,
    int base_index)
{
#if DEBUG_INFO
    printf("PreScan: Global[%4d] Local[%4d] Shared[%4d] BlockIndex[%4d] BaseIndex[%4d] Entries[%d]\n",
        (int)global[0], (int)local[0], (int)shared, group_index, base_index, n);
#endif

    unsigned int k = PRESCAN_STORE_SUM;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &output_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &partial_sums);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, shared,         0);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &group_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(m_lpDevice->getCommandQ(), m_vKernels[k]->getKernel(), 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    return CL_SUCCESS;
}

int OclSumScan::PreScanStoreSumNonPowerOfTwo(
    size_t *global,
    size_t *local,
    size_t shared,
    cl_mem output_data,
    cl_mem input_data,
    cl_mem partial_sums,
    unsigned int n,
    int group_index,
    int base_index)
{
#if DEBUG_INFO
    printf("PreScanStoreSumNonPowerOfTwo: Global[%4d] Local[%4d] BlockIndex[%4d] BaseIndex[%4d] Entries[%d]\n",
        (int)global[0], (int)local[0], group_index, base_index, n);
#endif

    unsigned int k = PRESCAN_STORE_SUM_NON_POWER_OF_TWO;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &output_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &partial_sums);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, shared,         0);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &group_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(m_lpDevice->getCommandQ(), m_vKernels[k]->getKernel(), 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    return CL_SUCCESS;
}

int OclSumScan::PreScanNonPowerOfTwo(
    size_t *global,
    size_t *local,
    size_t shared,
    cl_mem output_data,
    cl_mem input_data,
    unsigned int n,
    int group_index,
    int base_index)
{
#if DEBUG_INFO
    printf("PreScanNonPowerOfTwo: Global[%4d] Local[%4d] BlockIndex[%4d] BaseIndex[%4d] Entries[%d]\n",
        (int)global[0], (int)local[0], group_index, base_index, n);
#endif

    unsigned int k = PRESCAN_NON_POWER_OF_TWO;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &output_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &input_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, shared,         0);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &group_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(m_lpDevice->getCommandQ(), m_vKernels[k]->getKernel(), 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }
    return CL_SUCCESS;
}

int OclSumScan::UniformAdd(
    size_t *global,
    size_t *local,
    cl_mem output_data,
    cl_mem partial_sums,
    unsigned int n,
    unsigned int group_offset,
    unsigned int base_index)
{
#if DEBUG_INFO
    printf("UniformAdd: Global[%4d] Local[%4d] BlockOffset[%4d] BaseIndex[%4d] Entries[%d]\n",
        (int)global[0], (int)local[0], group_offset, base_index, n);
#endif

    unsigned int k = UNIFORM_ADD;
    unsigned int a = 0;

    int err = CL_SUCCESS;
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &output_data);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_mem), &partial_sums);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(float),  0);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &group_offset);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &base_index);
    err |= clSetKernelArg(m_vKernels[k]->getKernel(),  a++, sizeof(cl_int), &n);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to set kernel arguments!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    err = CL_SUCCESS;
    err |= clEnqueueNDRangeKernel(m_lpDevice->getCommandQ(), m_vKernels[k]->getKernel(), 1, NULL, global, local, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        printf("Error: %s: Failed to execute kernel!\n", KernelNames[k]);
        return EXIT_FAILURE;
    }

    return CL_SUCCESS;
}

int OclSumScan::PreScanBufferRecursive(
    cl_mem output_data,
    cl_mem input_data,
    int max_group_size,
    int max_work_item_count,
    int element_count,
    int level)
{
    unsigned int m_szKernelGroup = max_group_size;
    unsigned int group_count = (int)fmax(1.0f, (int)ceil((float)element_count / (2.0f * m_szKernelGroup)));
    unsigned int work_item_count = 0;

    if (group_count > 1)
        work_item_count = m_szKernelGroup;
    else if (IsPow2(element_count))
        work_item_count = element_count / 2;
    else
        work_item_count = floorPow2(element_count);

    work_item_count = (work_item_count > max_work_item_count) ? max_work_item_count : work_item_count;

    unsigned int element_count_per_group = work_item_count * 2;
    unsigned int last_group_element_count = element_count - (group_count-1) * element_count_per_group;
    unsigned int remaining_work_item_count = (int)fmax(1.0f, last_group_element_count / 2);
    remaining_work_item_count = (remaining_work_item_count > max_work_item_count) ? max_work_item_count : remaining_work_item_count;
    unsigned int remainder = 0;
    size_t last_shared = 0;


    if (last_group_element_count != element_count_per_group)
    {
        remainder = 1;

        if(!IsPow2(last_group_element_count))
            remaining_work_item_count = floorPow2(last_group_element_count);

        remaining_work_item_count = (remaining_work_item_count > max_work_item_count) ? max_work_item_count : remaining_work_item_count;
        unsigned int padding = (2 * remaining_work_item_count) / NUM_BANKS;
        last_shared = sizeof(float) * (2 * remaining_work_item_count + padding);
    }

    remaining_work_item_count = (remaining_work_item_count > max_work_item_count) ? max_work_item_count : remaining_work_item_count;
    size_t global[] = { (int)fmax(1, group_count - remainder) * work_item_count, 1 };
    size_t local[]  = { work_item_count, 1 };

    unsigned int padding = element_count_per_group / NUM_BANKS;
    size_t shared = sizeof(float) * (element_count_per_group + padding);

    cl_mem partial_sums = m_lpMemScanPartialSums[level];
    int err = CL_SUCCESS;

    if (group_count > 1)
    {
        err = PreScanStoreSum(global, local, shared, output_data, input_data, partial_sums, work_item_count * 2, 0, 0);
        if(err != CL_SUCCESS)
            return err;

        if (remainder)
        {
            size_t last_global[] = { 1 * remaining_work_item_count, 1 };
            size_t last_local[]  = { remaining_work_item_count, 1 };

            err = PreScanStoreSumNonPowerOfTwo(
                    last_global, last_local, last_shared,
                    output_data, input_data, partial_sums,
                    last_group_element_count,
                    group_count - 1,
                    element_count - last_group_element_count);

            if(err != CL_SUCCESS)
                return err;

        }

        err = PreScanBufferRecursive(partial_sums, partial_sums, max_group_size, max_work_item_count, group_count, level + 1);
        if(err != CL_SUCCESS)
            return err;

        err = UniformAdd(global, local, output_data, partial_sums,  element_count - last_group_element_count, 0, 0);
        if(err != CL_SUCCESS)
            return err;

        if (remainder)
        {
            size_t last_global[] = { 1 * remaining_work_item_count, 1 };
            size_t last_local[]  = { remaining_work_item_count, 1 };

            err = UniformAdd(
                    last_global, last_local,
                    output_data, partial_sums,
                    last_group_element_count,
                    group_count - 1,
                    element_count - last_group_element_count);

            if(err != CL_SUCCESS)
                return err;
        }
    }
    else if (IsPow2(element_count))
    {
        err = PreScan(global, local, shared, output_data, input_data, work_item_count * 2, 0, 0);
        if(err != CL_SUCCESS)
            return err;
    }
    else
    {
        err = PreScanNonPowerOfTwo(global, local, shared, output_data, input_data, element_count, 0, 0);
        if(err != CL_SUCCESS)
            return err;
    }

    return CL_SUCCESS;
}

void OclSumScan::PreScanBuffer(
    cl_mem output_data,
    cl_mem input_data,
    unsigned int max_group_size,
    unsigned int max_work_item_count,
    unsigned int element_count)
{
    PreScanBufferRecursive(output_data, input_data, max_group_size, max_work_item_count, element_count, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void OclSumScan::scanReference( float* reference, float* input, const unsigned int count)
{
    reference[0] = 0;
    double total_sum = 0;

    unsigned int i = 1;
    for( i = 1; i < count; ++i)
    {
        total_sum += input[i-1];
        reference[i] = input[i-1] + reference[i-1];
    }
    if (total_sum != reference[count-1])
        printf("Warning: Exceeding single-precision accuracy.  Scan will be inaccurate.\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int OclSumScan::compute(cl_mem inMemBuffer, cl_mem outMemBuffer, U32 count, float* lpInData)
{
	size_t buffer_size = sizeof(float) * count;
	float* result = (float*)malloc(buffer_size);
	memset(result, 0, buffer_size);

	if(!m_lpDevice->enqueueWriteBuffer(outMemBuffer, buffer_size, result))
	{
		printf("Error: Failed to write to source array!\n");
		return EXIT_FAILURE;
	}


	//Run
	createPartialSumBuffers(count);
	PreScanBuffer(outMemBuffer, inMemBuffer, m_szKernelGroup, m_szKernelGroup, count);

	m_lpDevice->finishAllCommands();

	// Read back the results that were computed on the device
	if(!m_lpDevice->enqueueReadBuffer(outMemBuffer, buffer_size, result))
	{
		printf("Error: Failed to read back results from the device!\n");
		return EXIT_FAILURE;
	}

	// Verify the results are correct
	//
	float* reference = (float*) malloc(buffer_size);
	scanReference(reference, lpInData, count);

	float error = 0.0f;
	float diff = 0.0f;
	for (U32 i = 0; i < count; i++) {
		diff = fabs(reference[i] - result[i]);
		error = diff > error ? diff : error;
	}

	if (error > MAX_ERROR) {
		printf("Error:   Incorrect results obtained! Max error = %f\n", error);
		return EXIT_FAILURE;
	} else {
		printf("Results Validated!\n");
		printf(SEPARATOR);
	}

	// Shutdown and cleanup
	releasePartialSums();

	free(reference);
	free(result);

	return CL_SUCCESS;
}
*/
}
}




