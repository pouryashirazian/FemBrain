#include "PS_OclScan.h"
#include "PS_FileDirectory.h"
#include "PS_ErrorManager.h"

using namespace PS;
using namespace PS::FILESTRINGUTILS;

namespace PS{
	namespace HPC{

		Scan::Scan(ComputeDevice* lpDevice)
		{
			init(lpDevice);
		}

		Scan::~Scan() {
			cl_int ciErrNum  = clReleaseMemObject(m_dstBuffer);
		}

		void Scan::init(ComputeDevice* lpDevice)
		{
			DAnsiStr strFP = ExtractFilePath(GetExePath());
			strFP = ExtractOneLevelUp(strFP);
			strFP += DAnsiStr("/AA_Shaders/Scan.cl");

			m_lpProgram = lpDevice->addProgramFromFile(strFP.cptr());
			assert(m_lpProgram != NULL);

			m_lpKernelScanExclusiveLocal1 = m_lpProgram->addKernel("scanExclusiveLocal1");
			m_lpKernelScanExclusiveLocal2 = m_lpProgram->addKernel("scanExclusiveLocal2");
			m_lpKernelUniformUpdate = m_lpProgram->addKernel("uniformUpdate");

			//shrLog( " ...checking minimum supported workgroup size\n");

			//Check for work group size
			cl_device_id device;
			size_t szScanExclusiveLocal1, szScanExclusiveLocal2, szUniformUpdate;
			cl_int ciErrNum;
			ciErrNum  = clGetCommandQueueInfo(lpDevice->getCommandQ(), CL_QUEUE_DEVICE, sizeof(cl_device_id), &device, NULL);
			ciErrNum |= clGetKernelWorkGroupInfo(m_lpKernelScanExclusiveLocal1->getKernel(), 
				lpDevice->getDevice(), 
				CL_KERNEL_WORK_GROUP_SIZE, 
				sizeof(size_t), 
				&szScanExclusiveLocal1, NULL);

			ciErrNum |= clGetKernelWorkGroupInfo(m_lpKernelScanExclusiveLocal2->getKernel(), 
				lpDevice->getDevice(), 
				CL_KERNEL_WORK_GROUP_SIZE, 
				sizeof(size_t), 
				&szScanExclusiveLocal2, NULL);

			ciErrNum |= clGetKernelWorkGroupInfo(m_lpKernelUniformUpdate->getKernel(), 
				lpDevice->getDevice(), 
				CL_KERNEL_WORK_GROUP_SIZE, 
				sizeof(size_t), 
				&szUniformUpdate, NULL);
			if(ciErrNum != CL_SUCCESS)		
				ReportErrorExt("ERROR: %s", lpDevice->oclErrorString(ciErrNum));
			//oclCheckError(ciErrNum, CL_SUCCESS);

			if( (szScanExclusiveLocal1 < WORKGROUP_SIZE) || (szScanExclusiveLocal2 < WORKGROUP_SIZE) || (szUniformUpdate < WORKGROUP_SIZE) ){
				ReportErrorExt("ERROR: Minimum work-group size %u required by this application is not supported on this device.\n", WORKGROUP_SIZE);
				exit(0);
			}

			//shrLog(" ...allocating internal buffers\n");
			m_dstBuffer = clCreateBuffer(lpDevice->getContext(), 
				CL_MEM_READ_WRITE, 
				(MAX_BATCH_ELEMENTS / (4 * WORKGROUP_SIZE)) * sizeof(U32), 
				NULL, &ciErrNum);
			ReportErrorExt("ERROR: %s", lpDevice->oclErrorString(ciErrNum));
			//oclCheckError(ciErrNum, CL_SUCCESS);

			m_clCommandQ = lpDevice->getCommandQ();
		}
		////////////////////////////////////////////////////////////////////////////////
		// Short scan launcher
		////////////////////////////////////////////////////////////////////////////////
		size_t Scan::scanExclusiveLocal1(cl_mem d_Dst, cl_mem d_Src, U32 n,	U32 size)
		{
			cl_int ciErrNum;
			size_t localWorkSize, globalWorkSize;

			ciErrNum  = clSetKernelArg(m_lpKernelScanExclusiveLocal1->getKernel(), 0, sizeof(cl_mem), (void *)&d_Dst);
			ciErrNum |= clSetKernelArg(m_lpKernelScanExclusiveLocal1->getKernel(), 1, sizeof(cl_mem), (void *)&d_Src);
			ciErrNum |= clSetKernelArg(m_lpKernelScanExclusiveLocal1->getKernel(), 2, 2 * WORKGROUP_SIZE * sizeof(U32), NULL);
			ciErrNum |= clSetKernelArg(m_lpKernelScanExclusiveLocal1->getKernel(), 3, sizeof(U32), (void *)&size);
			//oclCheckError(ciErrNum, CL_SUCCESS);

			localWorkSize = WORKGROUP_SIZE;
			globalWorkSize = (n * size) / 4;

			ciErrNum = clEnqueueNDRangeKernel(m_clCommandQ, 
				m_lpKernelScanExclusiveLocal1->getKernel(), 
				1, NULL, 
				&globalWorkSize, &localWorkSize, 0, NULL, NULL);
			//oclCheckError(ciErrNum, CL_SUCCESS);

			return localWorkSize;
		}

		size_t Scan::scanExclusiveShort(cl_mem d_Dst, cl_mem d_Src,	U32 batchSize, U32 arrayLength)
		{
			//Check power-of-two factorization
			U32 log2L;
			U32 factorizationRemainder = factorRadix2(log2L, arrayLength);
			assert( factorizationRemainder == 1);

			//Check supported size range
			assert( (arrayLength >= MIN_SHORT_ARRAY_SIZE) && (arrayLength <= MAX_SHORT_ARRAY_SIZE));

			//Check total batch size limit
			assert( (batchSize * arrayLength) <= MAX_BATCH_ELEMENTS);

			//Check all work-groups to be fully packed with data
			assert( (batchSize * arrayLength) % (4 * WORKGROUP_SIZE) == 0);

			return scanExclusiveLocal1(d_Dst, d_Src, batchSize,	arrayLength);
		}

		////////////////////////////////////////////////////////////////////////////////
		// Large scan launcher
		////////////////////////////////////////////////////////////////////////////////
		void Scan::scanExclusiveLocal2(cl_mem d_Buffer, cl_mem d_Dst, cl_mem d_Src,	U32 n, U32 size)
		{
				cl_int ciErrNum;
				size_t localWorkSize, globalWorkSize;

				U32 elements = n * size;
				ciErrNum  = clSetKernelArg(m_lpKernelScanExclusiveLocal2->getKernel(), 0, sizeof(cl_mem), (void *)&d_Buffer);
				ciErrNum |= clSetKernelArg(m_lpKernelScanExclusiveLocal2->getKernel(), 1, sizeof(cl_mem), (void *)&d_Dst);
				ciErrNum |= clSetKernelArg(m_lpKernelScanExclusiveLocal2->getKernel(), 2, sizeof(cl_mem), (void *)&d_Src);
				ciErrNum |= clSetKernelArg(m_lpKernelScanExclusiveLocal2->getKernel(), 3, 2 * WORKGROUP_SIZE * sizeof(U32), NULL);
				ciErrNum |= clSetKernelArg(m_lpKernelScanExclusiveLocal2->getKernel(), 4, sizeof(U32), (void *)&elements);
				ciErrNum |= clSetKernelArg(m_lpKernelScanExclusiveLocal2->getKernel(), 5, sizeof(U32), (void *)&size);
				assert(ciErrNum == CL_SUCCESS);

				localWorkSize = WORKGROUP_SIZE;
				globalWorkSize = iSnapUp(elements, WORKGROUP_SIZE);

				ciErrNum = clEnqueueNDRangeKernel(m_clCommandQ, m_lpKernelScanExclusiveLocal2->getKernel(), 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
				assert(ciErrNum == CL_SUCCESS);
		}

		size_t Scan::uniformUpdate(cl_mem d_Dst, cl_mem d_Buffer, U32 n)
		{
				cl_int ciErrNum;
				size_t localWorkSize, globalWorkSize;

				ciErrNum  = clSetKernelArg(m_lpKernelUniformUpdate->getKernel(), 0, sizeof(cl_mem), (void *)&d_Dst);
				ciErrNum |= clSetKernelArg(m_lpKernelUniformUpdate->getKernel(), 1, sizeof(cl_mem), (void *)&d_Buffer);
				assert(ciErrNum == CL_SUCCESS);

				localWorkSize = WORKGROUP_SIZE;
				globalWorkSize = n * WORKGROUP_SIZE;

				ciErrNum = clEnqueueNDRangeKernel(m_clCommandQ, m_lpKernelUniformUpdate->getKernel(), 1, NULL, &globalWorkSize, &localWorkSize, 0, NULL, NULL);
				assert(ciErrNum == CL_SUCCESS);

				return localWorkSize;
		}

		size_t Scan::scanExclusiveLarge(cl_mem d_Dst, cl_mem d_Src,	U32 batchSize, U32 arrayLength)
		{
			//Check power-of-two factorization
			U32 log2L;
			U32 factorizationRemainder = factorRadix2(log2L, arrayLength);
			assert(factorizationRemainder == 1);

			//Check supported size range
			assert( (arrayLength >= MIN_LARGE_ARRAY_SIZE) && (arrayLength <= MAX_LARGE_ARRAY_SIZE) == 1 );

			//Check total batch size limit
			assert((batchSize * arrayLength) <= MAX_BATCH_ELEMENTS);

			scanExclusiveLocal1(			
				d_Dst,
				d_Src,
				(batchSize * arrayLength) / (4 * WORKGROUP_SIZE),
				4 * WORKGROUP_SIZE
				);

			scanExclusiveLocal2(			
				m_dstBuffer,
				d_Dst,
				d_Src,
				batchSize,
				arrayLength / (4 * WORKGROUP_SIZE)
				);

			return uniformUpdate(			
				d_Dst,
				m_dstBuffer,
				(batchSize * arrayLength) / (4 * WORKGROUP_SIZE)
				);
		}

		U32 Scan::iSnapUp(U32 dividend, U32 divisor)
		{
			return ((dividend % divisor) == 0) ? dividend : (dividend - dividend % divisor + divisor);
		}

		U32 Scan::factorRadix2(U32& log2L, U32 L)
		{
			if(!L){
				log2L = 0;
				return 0;
			}else{
				for(log2L = 0; (L & 1) == 0; L >>= 1, log2L++);
				return L;
			}
		}

	}
}



