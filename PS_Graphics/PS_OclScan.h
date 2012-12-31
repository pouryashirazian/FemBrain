#ifndef PS_OCLSCAN_H
#define PS_OCLSCAN_H

#include "PS_ComputeDevice.h"

namespace PS{
namespace HPC{


	class Scan
	{
	public:
		Scan(ComputeDevice* lpDevice);
		~Scan();

		size_t scanExclusiveShort(cl_mem d_Dst, cl_mem d_Src,	U32 batchSize, U32 arrayLength);
		size_t scanExclusiveLarge(cl_mem d_Dst, cl_mem d_Src,	U32 batchSize, U32 arrayLength);

		static U32 factorRadix2(U32& log2L, U32 L);
		static U32 iSnapUp(U32 dividend, U32 divisor);

	private:
		void init(ComputeDevice* lpDevice);
		size_t scanExclusiveLocal1(cl_mem d_Dst, cl_mem d_Src, U32 n,	U32 size);
		void scanExclusiveLocal2(cl_mem d_Buffer, cl_mem d_Dst, cl_mem d_Src,	U32 n, U32 size);
		size_t uniformUpdate(cl_mem d_Dst, cl_mem d_Buffer, U32 n);

	private:		
		//All three kernels run 512 threads per workgroup
		//Must be a power of two
		static const U32  WORKGROUP_SIZE = 256;

		////////////////////////////////////////////////////////////////////////////////
		// Common definitions
		////////////////////////////////////////////////////////////////////////////////
		static const U32 MAX_BATCH_ELEMENTS = 64 * 1048576;
		static const U32 MIN_SHORT_ARRAY_SIZE = 4;
		static const U32 MAX_SHORT_ARRAY_SIZE = 4 * WORKGROUP_SIZE;
		static const U32 MIN_LARGE_ARRAY_SIZE = 8 * WORKGROUP_SIZE;
		static const U32 MAX_LARGE_ARRAY_SIZE = 4 * WORKGROUP_SIZE * WORKGROUP_SIZE;

		//Keep compute device
		ComputeDevice* m_lpDevice;

		//OpenCL scan kernel handles
		ComputeKernel* m_lpKernelScanExclusiveLocal1;
		ComputeKernel* m_lpKernelScanExclusiveLocal2;
		ComputeKernel* m_lpKernelUniformUpdate;

		//Dest Buffer
		cl_mem m_dstBuffer;		
	};
}
}

#endif
