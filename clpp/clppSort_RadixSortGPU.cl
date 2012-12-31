//------------------------------------------------------------
// Purpose :
// ---------
//
// Algorithm :
// -----------
// Radix sort algorithm for key-value pairs. This work is based on the Blelloch
// paper and optimized with the technique described in the Satish/Harris/Garland paper.
//
// References :
// ------------
// Designing Efficient Sorting Algorithms for Manycore GPUs. Nadathur Satish, Mark Harris, Michael Garland. http://mgarland.org/files/papers/gpusort-ipdps09.pdf
// http://www.sci.utah.edu/~csilva/papers/cgf.pdf
// Radix Sort For Vector Multiprocessors, Marco Zagha and Guy E. Blelloch
//------------------------------------------------------------

// To do : visiting logic and multi-scan.

#pragma OPENCL EXTENSION cl_amd_printf : enable

#define WGZ 32
#define WGZ_x2 (WGZ*2)
#define WGZ_x3 (WGZ*3)
#define WGZ_x4 (WGZ*4)
#define WGZ_1 (WGZ-1)
#define WGZ_2 (WGZ-2)
#define WGZ_x2_1 (WGZ_x2-1)
#define WGZ_x3_1 (WGZ_x3-1)
#define WGZ_x4_1 (WGZ_x4-1)
#define WGZ_x4_2 (WGZ_x4-2)

#if KEYS_ONLY
#define KEY(DATA) (DATA)
#else
#define KEY(DATA) (DATA.x)
#endif

#define EXTRACT_KEY_BIT(VALUE,BIT) ((KEY(VALUE)>>BIT)&0x1)
#define EXTRACT_KEY_4BITS(VALUE,BIT) ((KEY(VALUE)>>BIT)&0xF)

// Because our workgroup size = SIMT size, we use the natural synchronization provided by SIMT.
// So, we don't need any barrier to synchronize
#define BARRIER_LOCAL barrier(CLK_LOCAL_MEM_FENCE)
#define SIMT 32
#define SIMT_1 (SIMT-1)
#define SIMT_2 (SIMT-2)
#define COMPUTE_UNITS 4
#define TPG (COMPUTE_UNITS * SIMT)
#define TPG_2 (TPG-2)

//------------------------------------------------------------
// exclusive_scan_128
//
// Purpose : Do a scan of 128 elements in once.
//------------------------------------------------------------

inline 
uint4 inclusive_scan_128(volatile __local uint* localBuffer, const uint tid, uint block, uint lane, uint4 initialValue, __local uint* bitsOnCount)
{	
	//---- scan : 4 bits
	uint4 localBits = initialValue;
	localBits.y += localBits.x;
	localBits.z += localBits.y;
	localBits.w += localBits.z;
	
	//---- scan the last 4x32 bits (The sum in the previous scan)
	
	// The following is the same as 2 * SIMT_SIZE * simtId + threadInSIMT = 
    // 64*(threadIdx.x >> 5) + (threadIdx.x & (:WARP_SIZE - 1))
    //int localId = get_local_id(0);
    //int idx = 2 * localId - (localId & (WARP_SIZE - 1));
	//uint tid2 = 2 * tid - lane;
	
	uint tid2 = block * 2 * SIMT + lane;
	
	localBuffer[tid2] = 0;
	tid2 += SIMT;
	localBuffer[tid2] = localBits.w;
	
	localBuffer[tid2] += localBuffer[tid2 - 1];
	localBuffer[tid2] += localBuffer[tid2 - 2];
	localBuffer[tid2] += localBuffer[tid2 - 4];
	localBuffer[tid2] += localBuffer[tid2 - 8];
	localBuffer[tid2] += localBuffer[tid2 - 16];
	
	//---- Add the sum to create a scan of 128 bits
	return localBits + localBuffer[tid2 - 1];
}

inline 
uint4 exclusive_scan_512(const uint tid, uint4 initialValue, __local uint* bitsOnCount)
{
	__local uint localBuffer[TPG*2];
	uint lane = tid & SIMT_1;
	uint block = tid >> 5;
	
	uint4 localBits = inclusive_scan_128(localBuffer, tid, block, lane, initialValue, bitsOnCount);
	
	barrier(CLK_LOCAL_MEM_FENCE);
	
	//---- Scan 512
	if (lane > SIMT_2)
	{
		localBuffer[block] = 0;
		localBuffer[4 + block] = localBits.w;
	}
		
	barrier(CLK_LOCAL_MEM_FENCE);
	
	// Use the SIMT capabilities
	if (tid < 4)
	{
		uint tid2 = tid + 4;		
		localBuffer[tid2] += localBuffer[tid2 - 1];
		localBuffer[tid2] += localBuffer[tid2 - 2];
	}
	
	barrier(CLK_LOCAL_MEM_FENCE);
	
	// Add the sum
	localBits += localBuffer[block + 4 - 1];
	
	// Total number of '1' in the array, retreived from the inclusive scan
	if (tid > TPG_2)
		bitsOnCount[0] = localBits.w;
		
	// To exclusive scan
	return localBits - initialValue;
}

//------------------------------------------------------------
// kernel__radixLocalSort
//
// Purpose :
// 1) Each workgroup sorts its tile by using local memory
// 2) Create an histogram of d=2^b digits entries
//------------------------------------------------------------

__kernel
void kernel__radixLocalSort(
	//__local KV_TYPE* localDataOLD,
	__global KV_TYPE* data,
	const int bitOffset,
	const int N)
{
	const uint tid = (uint)get_local_id(0);
	const uint4 tid4 = (const uint4)(tid << 2) + (const uint4)(0,1,2,3);
	const uint4 gid4 = (const uint4)(get_global_id(0) << 2) + (const uint4)(0,1,2,3);
	
	// Local memory
	__local KV_TYPE localDataArray[TPG*4*2]; // Faster than using it as a parameter !!!
	__local KV_TYPE* localData = localDataArray;
	__local KV_TYPE* localTemp = localData + TPG;
    __local uint bitsOnCount[1];

    // Each thread copies 4 (Cell,Tri) pairs into local memory
    localData[tid4.x] = (gid4.x < N) ? data[gid4.x] : MAX_KV_TYPE;
    localData[tid4.y] = (gid4.y < N) ? data[gid4.y] : MAX_KV_TYPE;
    localData[tid4.z] = (gid4.z < N) ? data[gid4.z] : MAX_KV_TYPE;
    localData[tid4.w] = (gid4.w < N) ? data[gid4.w] : MAX_KV_TYPE;
	
	//-------- 1) 4 x local 1-bit split	
	#pragma unroll
    for(uint shift = bitOffset; shift < (bitOffset+4); shift++) // Radix 4
    {
		//barrier(CLK_LOCAL_MEM_FENCE);
		
		//---- Setup the array of 4 bits (of level shift)
		// Create the '1s' array as explained at : http://http.developer.nvidia.com/GPUGems3/gpugems3_ch39.html
		// In fact we simply inverse the bits	
		// Local copy and bits extraction
		uint4 flags;
		flags.x = ! EXTRACT_KEY_BIT(localData[tid4.x], shift);
        flags.y = ! EXTRACT_KEY_BIT(localData[tid4.y], shift);
        flags.z = ! EXTRACT_KEY_BIT(localData[tid4.z], shift);
        flags.w = ! EXTRACT_KEY_BIT(localData[tid4.w], shift);

		//---- Do a scan of the 128 bits and retreive the total number of '1' in 'bitsOnCount'
		uint4 localBitsScan = exclusive_scan_512(tid, flags, bitsOnCount);
		
		// Waiting for 'bitsOnCount'
		barrier(CLK_LOCAL_MEM_FENCE);
		
		//---- Relocate to the right position	
		uint4 offset = (1-flags) * ((uint4)(bitsOnCount[0]) + tid4 - localBitsScan) + flags * localBitsScan;
		localTemp[offset.x] = localData[tid4.x];
		localTemp[offset.y] = localData[tid4.y];
		localTemp[offset.z] = localData[tid4.z];
		localTemp[offset.w] = localData[tid4.w];
		
		// Wait before swapping the "local" buffer pointers. They are shared by the whole local context
		barrier(CLK_LOCAL_MEM_FENCE);

		// Swap the buffer pointers
		__local KV_TYPE* swBuf = localData;
		localData = localTemp;
		localTemp = swBuf;
    }
	
	//barrier(CLK_LOCAL_MEM_FENCE);
	
	// Write sorted data back to global memory
	if (gid4.x < N) data[gid4.x] = localData[tid4.x];
    if (gid4.y < N) data[gid4.y] = localData[tid4.y];
    if (gid4.z < N) data[gid4.z] = localData[tid4.z];
    if (gid4.w < N) data[gid4.w] = localData[tid4.w];
}

//------------------------------------------------------------
// kernel__localHistogram
//
// Purpose :
//------------------------------------------------------------

__kernel
void kernel__localHistogram(__global KV_TYPE* data, const int bitOffset, __global int* hist, __global int* blockHists, const int N)
{
    const int tid = (int)get_local_id(0);
    const int4 tid4 = (int4)(tid << 2) + (const int4)(0,1,2,3);
	const int4 gid4 = (int4)(get_global_id(0) << 2) + (const int4)(0,1,2,3);
	const int blockId = (int)get_group_id(0);
	
	__local uint localData[WGZ*4];
    __local int localHistStart[16];
    __local int localHistEnd[16];
	
    localData[tid4.x] = (gid4.x < N) ? EXTRACT_KEY_4BITS(data[gid4.x], bitOffset) : EXTRACT_KEY_4BITS(MAX_KV_TYPE, bitOffset);
    localData[tid4.y] = (gid4.y < N) ? EXTRACT_KEY_4BITS(data[gid4.y], bitOffset) : EXTRACT_KEY_4BITS(MAX_KV_TYPE, bitOffset);
    localData[tid4.z] = (gid4.z < N) ? EXTRACT_KEY_4BITS(data[gid4.z], bitOffset) : EXTRACT_KEY_4BITS(MAX_KV_TYPE, bitOffset);
    localData[tid4.w] = (gid4.w < N) ? EXTRACT_KEY_4BITS(data[gid4.w], bitOffset) : EXTRACT_KEY_4BITS(MAX_KV_TYPE, bitOffset);
	
	//-------- 2) Histogram

    // init histogram values
    BARRIER_LOCAL;
    if (tid < 16)
    {
        localHistStart[tid] = 0;
        localHistEnd[tid] = -1;
    }
	BARRIER_LOCAL;

    // Start computation    
    if (tid4.x > 0 && localData[tid4.x] != localData[tid4.x-1])
    {
		localHistStart[localData[tid4.x]] = tid4.x;
        localHistEnd[localData[tid4.x-1]] = tid4.x - 1;        
    }

    if (localData[tid4.y] != localData[tid4.x])
    {
        localHistEnd[localData[tid4.x]] = tid4.x;
        localHistStart[localData[tid4.y]] = tid4.y;
    }

    if (localData[tid4.z] != localData[tid4.y])
    {
        localHistEnd[localData[tid4.y]] = tid4.y;
        localHistStart[localData[tid4.z]] = tid4.z;
    }

    if (localData[tid4.w] != localData[tid4.z])
    {
        localHistEnd[localData[tid4.z]] = tid4.z;
        localHistStart[localData[tid4.w]] = tid4.w;
    }

    if (tid < 1)
    {
		localHistEnd[localData[WGZ_x4-1]] = WGZ_x4 - 1;
		localHistStart[localData[0]] = 0;
    }
    BARRIER_LOCAL;

    // Write histogram to global memomry
    if (tid < 16)
    {
        hist[tid * get_num_groups(0) + blockId] = localHistEnd[tid] - localHistStart[tid] + 1;
		blockHists[(blockId << 5) + tid] = localHistStart[tid];
    }
}

//------------------------------------------------------------
// kernel__radixPermute
//
// Purpose : Prefix sum results are used to scatter each work-group's elements to their correct position.
//------------------------------------------------------------

__kernel
void kernel__radixPermute(
	__global const KV_TYPE* dataIn,		// size 4*4 int2s per block
	__global KV_TYPE* dataOut,				// size 4*4 int2s per block
	__global const int* histSum,		// size 16 per block (64 B)
	__global const int* blockHists,		// size 16 int2s per block (64 B)
	const int bitOffset,				// k*4, k=0..7
	const int N,
	const int numBlocks)
{    
    const int tid = get_local_id(0);	
	const int groupId = get_group_id(0);
    const int4 tid4 = ((const int4)tid) + (const int4)(0,WGZ,WGZ_x2,WGZ_x3);		
	const int4 gid4 = tid4 + ((const int4)groupId<<2);
	
    __local int sharedHistSum[16];
    __local int localHistStart[16];

    // Fetch per-block KV_TYPE histogram and int histogram sums
    if (tid < 16)
    {
        sharedHistSum[tid] = histSum[tid * numBlocks + groupId];
        localHistStart[tid] = blockHists[(groupId << 5) + tid]; // groupId * 32 + tid
		//localHistStart[tid] = blockHists[(groupId << 4) + tid]; // groupId * 16 + tid
    }
	
	BARRIER_LOCAL;
	
	KV_TYPE myData;
    uint myShiftedKeys;
	uint finalOffset;	

	myData = (gid4.x < N) ? dataIn[gid4.x] : MAX_KV_TYPE;
    myShiftedKeys = EXTRACT_KEY_4BITS(myData, bitOffset);
	finalOffset = tid4.x - localHistStart[myShiftedKeys] + sharedHistSum[myShiftedKeys];
	if (finalOffset < N) dataOut[finalOffset] = myData;
	
	myData = (gid4.y < N) ? dataIn[gid4.y] : MAX_KV_TYPE;
    myShiftedKeys = EXTRACT_KEY_4BITS(myData, bitOffset);
	finalOffset = tid4.y - localHistStart[myShiftedKeys] + sharedHistSum[myShiftedKeys];
	if (finalOffset < N) dataOut[finalOffset] = myData;
	
	myData = (gid4.z < N) ? dataIn[gid4.z] : MAX_KV_TYPE;
    myShiftedKeys = EXTRACT_KEY_4BITS(myData, bitOffset);
	finalOffset = tid4.z - localHistStart[myShiftedKeys] + sharedHistSum[myShiftedKeys];
	if (finalOffset < N) dataOut[finalOffset] = myData;

	myData = (gid4.w < N) ? dataIn[gid4.w] : MAX_KV_TYPE;	
    myShiftedKeys = EXTRACT_KEY_4BITS(myData, bitOffset);
    finalOffset = tid4.w - localHistStart[myShiftedKeys] + sharedHistSum[myShiftedKeys];
    if (finalOffset < N) dataOut[finalOffset] = myData;
}