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
// http://http.developer.nvidia.com/GPUGems3/gpugems3_ch39.html
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

#if KEYS_ONLY
#define KEY(DATA) (DATA)
#else
#define KEY(DATA) (DATA.x)
#endif

#define EXTRACT_KEY_BIT(VALUE,BIT) ((KEY(VALUE)>>BIT)&0x1)
#define EXTRACT_KEY_4BITS(VALUE,BIT) ((KEY(VALUE)>>BIT)&0xF)

#if defined(OCL_DEVICE_GPU) && defined(OCL_PLATFORM_NVIDIA)

// Because our workgroup size = SIMT size, we use the natural synchronization provided by SIMT.
// So, we don't need any barrier to synchronize
#define BARRIER_LOCAL

#else

#define BARRIER_LOCAL barrier(CLK_LOCAL_MEM_FENCE)

#endif

//------------------------------------------------------------
// exclusive_scan_128
//
// Purpose : Do a scan of 128 elements in once.
//------------------------------------------------------------

#if defined(OCL_DEVICE_GPU) && defined(OCL_PLATFORM_NVIDIA)

inline 
uint4 exclusive_scan_128(const uint tid, uint4 initialValue, __local uint* bitsOnCount)
{
	__local uint localBuffer[64];
		
	// local scan
	uint4 localBits = initialValue;
	localBits.y += localBits.x;
	localBits.z += localBits.y;
	localBits.w += localBits.z;
	
	// Scan each 4st values (The sums)
	
	uint tid2 = tid + 32;
	
	localBuffer[tid] = 0;
	localBuffer[tid2] = localBits.w;
	
	localBuffer[tid2] += localBuffer[tid2 - 1];
	localBuffer[tid2] += localBuffer[tid2 - 2];
	localBuffer[tid2] += localBuffer[tid2 - 4];
	localBuffer[tid2] += localBuffer[tid2 - 8];
	localBuffer[tid2] += localBuffer[tid2 - 16];
	
	// Total number of '1' in the array, retreived from the inclusive scan
	if (tid > WGZ_2)
		bitsOnCount[0] = localBuffer[63];
	
	// 1 - To exclusive scan
	// 2 - Add the sums
	//int toAdd = (tid > 0) ? localBuffer[tid2-1] : 0;
	//return localBits - initialValue + toAdd;
	
	return localBits - initialValue + localBuffer[tid2-1];
}

#else

inline
void exclusive_scan_4(const uint tid, const int4 tid4, __local uint* localBuffer, __local uint* bitsOnCount)
{
    const int tid2_0 = tid << 1;
    const int tid2_1 = tid2_0 + 1;
	
	int offset = 4;
	//#pragma unroll
	for (uint d = 16; d > 0; d >>= 1)
    {
        barrier(CLK_LOCAL_MEM_FENCE);
		
        if (tid < d)
        {
            const uint ai = mad24(offset, (tid2_1+0), -1);	// offset*(tid2_0+1)-1 = offset*(tid2_1+0)-1
            const uint bi = mad24(offset, (tid2_1+1), -1);	// offset*(tid2_1+1)-1;
			
            localBuffer[bi] += localBuffer[ai];
        }
		
		offset <<= 1;
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    if (tid > WGZ_2)
    {
        bitsOnCount[0] = localBuffer[tid4.w];
        localBuffer[tid4.w] = 0;
    }

    // expansion
	//#pragma unroll
    for (uint d = 1; d < 32; d <<= 1)
    {
        barrier(CLK_LOCAL_MEM_FENCE);
		offset >>= 1;
		
        if (tid < d)
        {
            const uint ai = mad24(offset, (tid2_1+0), -1); // offset*(tid2_0+1)-1 = offset*(tid2_1+0)-1
            const uint bi = mad24(offset, (tid2_1+1), -1); // offset*(tid2_1+1)-1;
			
            K_TYPE tmp = localBuffer[ai];
            localBuffer[ai] = localBuffer[bi];
            localBuffer[bi] += tmp;
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
}

inline 
void exclusive_scan_128(const uint tid, const int4 tid4, __local uint* localBuffer, __local uint* bitsOnCount)
{
	// local serial reduction
	localBuffer[tid4.y] += localBuffer[tid4.x];
	localBuffer[tid4.w] += localBuffer[tid4.z];
	localBuffer[tid4.w] += localBuffer[tid4.y];
		
	// Exclusive scan starting with an offset of 4
	exclusive_scan_4(tid, tid4, localBuffer, bitsOnCount);
		
	// local 4-element serial expansion
	uint tmp;
	tmp = localBuffer[tid4.y];    localBuffer[tid4.y] = localBuffer[tid4.w];  localBuffer[tid4.w] += tmp;
	tmp = localBuffer[tid4.x];    localBuffer[tid4.x] = localBuffer[tid4.y];  localBuffer[tid4.y] += tmp;
	tmp = localBuffer[tid4.z];    localBuffer[tid4.z] = localBuffer[tid4.w];  localBuffer[tid4.w] += tmp;
}

#endif

//------------------------------------------------------------
// kernel__radixLocalSort
//
// Purpose :
// 1) Each workgroup sorts its tile by using local memory
// 2) Create an histogram of d=2^b digits entries
//------------------------------------------------------------

#if defined(OCL_DEVICE_GPU) && defined(OCL_PLATFORM_NVIDIA)

__kernel
void kernel__radixLocalSort(
	__local KV_TYPE* localData,			// size 4*4 int2s (8 kB)
	__global KV_TYPE* data,				// size 4*4 int2s per block (8 kB)
	const int bitOffset,				// k*4, k=0..7
	const int N)						// Total number of items to sort
{
	const uint tid = get_local_id(0);		
    
	// BUGGY but faster
	//const uint4 tid4 = ((const uint4)tid) + (const uint4)(0,WGZ,WGZ_x2,WGZ_x3);		
	//const uint4 gid4 = tid4 + ((const uint4)get_group_id(0)<<2);
	
	// ORIGINAL
	//const int gid4 = (int)(get_global_id(0) << 2);
    //const int4 tid4 = (int4)(tid << 2) + (const int4)(0,1,2,3);
	
	// CURRENT : SLOW !!! (Bank conflict)
	const uint4 tid4 = (const uint4)(tid << 2) + (const uint4)(0,1,2,3);
	const uint4 gid4 = (const uint4)(get_global_id(0) << 2) + (const uint4)(0,1,2,3);
	
	/*const uint4 tid4_g = ((const uint4)tid) + (const uint4)(0,WGZ,WGZ_x2,WGZ_x3);
	const uint4 gid4 = (const uint4)(get_group_id(0) * 16) + tid4_g;
	const uint4 tid4 = (const uint4)(tid << 2) + (const uint4)(0,1,2,3);*/
	
	// Local memory
    __local uint bitsOnCount[1];

    // Each thread copies 4 (Cell,Tri) pairs into local memory
    localData[tid4.x] = (gid4.x < N) ? data[gid4.x] : MAX_KV_TYPE;
    localData[tid4.y] = (gid4.y < N) ? data[gid4.y] : MAX_KV_TYPE;
    localData[tid4.z] = (gid4.z < N) ? data[gid4.z] : MAX_KV_TYPE;
    localData[tid4.w] = (gid4.w < N) ? data[gid4.w] : MAX_KV_TYPE;
	
	//barrier(CLK_LOCAL_MEM_FENCE);
	
	//-------- 1) 4 x local 1-bit split

	__local KV_TYPE* localTemp = localData + WGZ_x4;
	#pragma unroll // SLOWER on some cards (cheap cards) and with small data-sets !!
    for(uint shift = bitOffset; shift < (bitOffset+4); shift++) // Radix 4
    {
		BARRIER_LOCAL;
		
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
		uint4 localBitsScan = exclusive_scan_128(tid, flags, bitsOnCount);
		
		BARRIER_LOCAL;
				
		//---- Relocate to the right position	
		uint4 offset = (1-flags) * ((uint4)(bitsOnCount[0]) + tid4 - localBitsScan) + flags * localBitsScan;
		localTemp[offset.x] = localData[tid4.x];
		localTemp[offset.y] = localData[tid4.y];
		localTemp[offset.z] = localData[tid4.z];
		localTemp[offset.w] = localData[tid4.w];
		
		BARRIER_LOCAL;

		// Swap the buffer pointers
		__local KV_TYPE* swBuf = localData;
		localData = localTemp;
		localTemp = swBuf;
		
		BARRIER_LOCAL;
    }
	
	// FASTER !!
	//barrier(CLK_LOCAL_MEM_FENCE); // NO CRASH !!
	
	// Write sorted data back to global memory
	if (gid4.x < N) data[gid4.x] = localData[tid4.x];
    if (gid4.y < N) data[gid4.y] = localData[tid4.y];
    if (gid4.z < N) data[gid4.z] = localData[tid4.z];
    if (gid4.w < N) data[gid4.w] = localData[tid4.w];
}

#else

__kernel
void kernel__radixLocalSort(
	__local KV_TYPE* localData,			// size 4*4 int2s (8 kB)
	__global KV_TYPE* data,				// size 4*4 int2s per block (8 kB)
	const int bitOffset,				// k*4, k=0..7
	const int N)						// Total number of items to sort
{
	const int tid = (int)get_local_id(0);
		
    const int4 gid4 = (int4)(get_global_id(0) << 2) + (const int4)(0,1,2,3);    
    const int4 tid4 = (int4)(tid << 2) + (const int4)(0,1,2,3);
    
	// Local memory
	__local uint localBitsScan[WGZ_x4];
    __local uint bitsOnCount[1];

    // Each thread copies 4 (Cell,Tri) pairs into local memory
    localData[tid4.x] = (gid4.x < N) ? data[gid4.x] : MAX_KV_TYPE;
    localData[tid4.y] = (gid4.y < N) ? data[gid4.y] : MAX_KV_TYPE;
    localData[tid4.z] = (gid4.z < N) ? data[gid4.z] : MAX_KV_TYPE;
    localData[tid4.w] = (gid4.w < N) ? data[gid4.w] : MAX_KV_TYPE;
	
	//-------- 1) 4 x local 1-bit split

	__local KV_TYPE* localTemp = localData + WGZ_x4;
	#pragma unroll // SLOWER on some cards!!
    for(uint shift = bitOffset; shift < (bitOffset+4); shift++) // Radix 4
    {
		BARRIER_LOCAL;
		
		//---- Setup the array of 4 bits (of level shift)
		// Create the '1s' array as explained at : http://http.developer.nvidia.com/GPUGems3/gpugems3_ch39.html
		// In fact we simply inverse the bits	
		// Local copy and bits extraction
		int4 flags;
		flags.x = localBitsScan[tid4.x] = ! EXTRACT_KEY_BIT(localData[tid4.x], shift);
        flags.y = localBitsScan[tid4.y] = ! EXTRACT_KEY_BIT(localData[tid4.y], shift);
        flags.z = localBitsScan[tid4.z] = ! EXTRACT_KEY_BIT(localData[tid4.z], shift);
        flags.w = localBitsScan[tid4.w] = ! EXTRACT_KEY_BIT(localData[tid4.w], shift);
						
		//---- Do a scan of the 128 bits and retreive the total number of '1' in 'bitsOnCount'
		exclusive_scan_128(tid, tid4, localBitsScan, bitsOnCount);
		
		BARRIER_LOCAL;
		
		//----
		int offset;
		int4 invFlags = 1 - flags;
		offset = invFlags.x * (bitsOnCount[0] + tid4.x - localBitsScan[tid4.x]) + flags.x * localBitsScan[tid4.x];
		localTemp[offset] = localData[tid4.x];
		
		offset = invFlags.y * (bitsOnCount[0] + tid4.y - localBitsScan[tid4.y]) + flags.y * localBitsScan[tid4.y];
		localTemp[offset] = localData[tid4.y];
		
		offset = invFlags.z * (bitsOnCount[0] + tid4.z - localBitsScan[tid4.z]) + flags.z * localBitsScan[tid4.z];
		localTemp[offset] = localData[tid4.z];
				
		offset = invFlags.w * (bitsOnCount[0] + tid4.w - localBitsScan[tid4.w]) + flags.w * localBitsScan[tid4.w];
		localTemp[offset] = localData[tid4.w];
		
		BARRIER_LOCAL;

		// Swap the buffer pointers
		__local KV_TYPE* swBuf = localData;
		localData = localTemp;
		localTemp = swBuf;
		
		//barrier(CLK_LOCAL_MEM_FENCE); // NO CRASH !!			
    }
	
	// FASTER !!
	//barrier(CLK_LOCAL_MEM_FENCE); // NO CRASH !!
	
	// Write sorted data back to global memory
	if (gid4.x < N) data[gid4.x] = localData[tid4.x];
    if (gid4.y < N) data[gid4.y] = localData[tid4.y];
    if (gid4.z < N) data[gid4.z] = localData[tid4.z];
    if (gid4.w < N) data[gid4.w] = localData[tid4.w];	
}

#endif

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

#if defined(OCL_DEVICE_GPU) && defined(OCL_PLATFORM_NVIDIA)

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

#else

__kernel
void kernel__radixPermute(
	__global const KV_TYPE* dataIn,		// size 4*4 int2s per block
	__global KV_TYPE* dataOut,				// size 4*4 int2s per block
	__global const int* histSum,		// size 16 per block (64 B)
	__global const int* blockHists,		// size 16 int2s per block (64 B)
	const uint bitOffset,				// k*4, k=0..7
	const uint N,
	const int numBlocks)
{
    const uint4 gid4 = ((const uint4)(get_global_id(0) << 2)) + (const uint4)(0,1,2,3);
    const uint tid = get_local_id(0);
    const uint4 tid4 = ((const uint4)(tid << 2)) + (const uint4)(0,1,2,3);
    
    //const int numBlocks = get_num_groups(0); // Can be passed as a parameter !
    __local int sharedHistSum[16];
    __local int localHistStart[16];

    // Fetch per-block KV_TYPE histogram and int histogram sums
    if (tid < 16)
    {
		const uint blockId = get_group_id(0);
        sharedHistSum[tid] = histSum[tid * numBlocks + blockId];
		//localHistStart[tid] = blockHists[(blockId << 4) + tid];
        localHistStart[tid] = blockHists[(blockId << 5) + tid];
    }
	
	BARRIER_LOCAL;

    // Copy data, each thread copies 4 (Cell,Tri) pairs into local memory
    KV_TYPE myData[4];
    uint myShiftedKeys[4];
    myData[0] = (gid4.x < N) ? dataIn[gid4.x] : MAX_KV_TYPE;
    myData[1] = (gid4.y < N) ? dataIn[gid4.y] : MAX_KV_TYPE;
    myData[2] = (gid4.z < N) ? dataIn[gid4.z] : MAX_KV_TYPE;
    myData[3] = (gid4.w < N) ? dataIn[gid4.w] : MAX_KV_TYPE;

    myShiftedKeys[0] = EXTRACT_KEY_4BITS(myData[0], bitOffset);
    myShiftedKeys[1] = EXTRACT_KEY_4BITS(myData[1], bitOffset);
    myShiftedKeys[2] = EXTRACT_KEY_4BITS(myData[2], bitOffset);
    myShiftedKeys[3] = EXTRACT_KEY_4BITS(myData[3], bitOffset);

	// Necessary ?
    //BARRIER_LOCAL;

    // Compute the final indices
    uint4 finalOffset;
    finalOffset.x = tid4.x - localHistStart[myShiftedKeys[0]] + sharedHistSum[myShiftedKeys[0]];
    finalOffset.y = tid4.y - localHistStart[myShiftedKeys[1]] + sharedHistSum[myShiftedKeys[1]];
    finalOffset.z = tid4.z - localHistStart[myShiftedKeys[2]] + sharedHistSum[myShiftedKeys[2]];
    finalOffset.w = tid4.w - localHistStart[myShiftedKeys[3]] + sharedHistSum[myShiftedKeys[3]];

    // Permute the data to the final offsets
	if (finalOffset.x < N) dataOut[finalOffset.x] = myData[0];
    if (finalOffset.y < N) dataOut[finalOffset.y] = myData[1];
    if (finalOffset.z < N) dataOut[finalOffset.z] = myData[2];
    if (finalOffset.w < N) dataOut[finalOffset.w] = myData[3];
}

#endif