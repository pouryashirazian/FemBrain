//
//  OclHelperFuncs.cpp
//  hifem
//
//  Created by pshiraz on 1/28/14.
//  Copyright (c) 2014 pshiraz. All rights reserved.
//

#include "OclHelperFuncs.h"
#include "base/Logger.h"

using namespace PS;

namespace PS {
    namespace CL {
        
        cl_int oclPlatformID(cl_platform_id* clSelectedPlatformID, const char* lpStrProvider)
        {
            char chBuffer[1024];
            cl_uint num_platforms;
            cl_platform_id* clPlatformIDs;
            cl_int ciErrNum;
            *clSelectedPlatformID = NULL;
            
            // Get OpenCL platform count
            ciErrNum = clGetPlatformIDs (0, NULL, &num_platforms);
            if (ciErrNum != CL_SUCCESS)
            {
                LogErrorArg1("Error code: %i in clGetPlatformIDs Call!!", ciErrNum);
                return -1000;
            }
            else
            {
                if(num_platforms == 0)
                {
                    LogError("No OpenCL platform found!");
                    return -2000;
                }
                else
                {
                    // if there's a platform or more, make space for ID's
                    if ((clPlatformIDs = (cl_platform_id*)malloc(num_platforms * sizeof(cl_platform_id))) == NULL)
                    {
                        LogError("Failed to allocate memory for cl_platform ID's!");
                        return -3000;
                    }
                    
                    // get platform info for each platform and trap the NVIDIA platform if found
                    ciErrNum = clGetPlatformIDs (num_platforms, clPlatformIDs, NULL);
                    for(cl_uint i = 0; i < num_platforms; ++i)
                    {
                        ciErrNum = clGetPlatformInfo (clPlatformIDs[i], CL_PLATFORM_NAME, 1024, &chBuffer, NULL);
                        if(ciErrNum == CL_SUCCESS)
                        {
                            if(strstr(chBuffer, lpStrProvider) != NULL)
                            {
                                *clSelectedPlatformID = clPlatformIDs[i];
                                break;
                            }
                        }
                    }
                    
                    // default to zeroeth platform if NVIDIA not found
                    if(*clSelectedPlatformID == NULL)
                    {
                        LogWarningArg1("%s OpenCL platform not found - defaulting to first platform!", lpStrProvider);
                        *clSelectedPlatformID = clPlatformIDs[0];
                    }
                    
                    free(clPlatformIDs);
                }
            }
            
            return CL_SUCCESS;
        }
        
        //////////////////////////////////////////////////////////////////////////////////////////////
        int oclGetDevCap(cl_device_id device)
        {
            char cDevString[1024];
            bool bDevAttributeQuery = false;
            int iDevArch = -1;
            
            // Get device extensions, and if any then search for cl_nv_device_attribute_query
            clGetDeviceInfo(device, CL_DEVICE_EXTENSIONS, sizeof(cDevString), &cDevString, NULL);
            if (cDevString != 0)
            {
                std::string stdDevString;
                stdDevString = std::string(cDevString);
                size_t szOldPos = 0;
                size_t szSpacePos = stdDevString.find(' ', szOldPos); // extensions string is space delimited
                while (szSpacePos != stdDevString.npos)
                {
                    if( strcmp("cl_nv_device_attribute_query", stdDevString.substr(szOldPos, szSpacePos - szOldPos).c_str()) == 0 )
                    {
                        bDevAttributeQuery = true;
                    }
                    
                    do {
                        szOldPos = szSpacePos + 1;
                        szSpacePos = stdDevString.find(' ', szOldPos);
                    } while (szSpacePos == szOldPos);
                }
            }
            
            // if search succeeded, get device caps
            if(bDevAttributeQuery)
            {
                cl_int iComputeCapMajor = 0;
                cl_int iComputeCapMinor = 0;
#ifdef CL_DEVICE_COMPUTE_CAPABILITY_MAJOR_NV
                clGetDeviceInfo(device, CL_DEVICE_COMPUTE_CAPABILITY_MAJOR_NV, sizeof(cl_uint), (void*)&iComputeCapMajor, NULL);
#endif
#ifdef CL_DEVICE_COMPUTE_CAPABILITY_MINOR_NV
                clGetDeviceInfo(device, CL_DEVICE_COMPUTE_CAPABILITY_MINOR_NV, sizeof(cl_uint), (void*)&iComputeCapMinor, NULL);
#endif
                iDevArch = (10 * iComputeCapMajor) + iComputeCapMinor;
            }
            
            return iDevArch;
        }
        
        //////////////////////////////////////////////////////////////////////////////////////////////
        cl_device_id oclGetDev(cl_context cxGPUContext, unsigned int nr)
        {
            size_t szParmDataBytes;
            cl_device_id* cdDevices;
            
            // get the list of GPU devices associated with context
            clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);
            
            if( szParmDataBytes / sizeof(cl_device_id) <= nr ) {
                return (cl_device_id)-1;
            }
            
            cdDevices = (cl_device_id*) malloc(szParmDataBytes);
            
            clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);
            
            cl_device_id device = cdDevices[nr];
            free(cdDevices);
            
            return device;
        }
        
        //////////////////////////////////////////////////////////////////////////////////////////////
        cl_device_id oclGetMaxFlopsDev(cl_context cxGPUContext)
        {
            size_t szParmDataBytes;
            cl_device_id* cdDevices;
            
            // get the list of GPU devices associated with context
            clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);
            cdDevices = (cl_device_id*) malloc(szParmDataBytes);
            size_t device_count = szParmDataBytes / sizeof(cl_device_id);
            
            clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);
            
            cl_device_id max_flops_device = cdDevices[0];
            int max_flops = 0;
            
            size_t current_device = 0;
            
            // CL_DEVICE_MAX_COMPUTE_UNITS
            cl_uint compute_units;
            clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(compute_units), &compute_units, NULL);
            
            // CL_DEVICE_MAX_CLOCK_FREQUENCY
            cl_uint clock_frequency;
            clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(clock_frequency), &clock_frequency, NULL);
            
            max_flops = compute_units * clock_frequency;
            ++current_device;
            
            while( current_device < device_count )
            {
                // CL_DEVICE_MAX_COMPUTE_UNITS
                cl_uint compute_units;
                clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(compute_units), &compute_units, NULL);
                
                // CL_DEVICE_MAX_CLOCK_FREQUENCY
                cl_uint clock_frequency;
                clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(clock_frequency), &clock_frequency, NULL);
                
                int flops = compute_units * clock_frequency;
                if( flops > max_flops )
                {
                    max_flops        = flops;
                    max_flops_device = cdDevices[current_device];
                }
                ++current_device;
            }
            
            free(cdDevices);
            
            return max_flops_device;
        }
        
        
        void oclLogPtx(cl_program cpProgram, cl_device_id cdDevice, const char* cPtxFileName)
        {
            // Grab the number of devices associated with the program
            cl_uint num_devices;
            clGetProgramInfo(cpProgram, CL_PROGRAM_NUM_DEVICES, sizeof(cl_uint), &num_devices, NULL);
            
            // Grab the device ids
            cl_device_id* devices = (cl_device_id*) malloc(num_devices * sizeof(cl_device_id));
            clGetProgramInfo(cpProgram, CL_PROGRAM_DEVICES, num_devices * sizeof(cl_device_id), devices, 0);
            
            // Grab the sizes of the binaries
            size_t* binary_sizes = (size_t*)malloc(num_devices * sizeof(size_t));
            clGetProgramInfo(cpProgram, CL_PROGRAM_BINARY_SIZES, num_devices * sizeof(size_t), binary_sizes, NULL);
            
            // Now get the binaries
            char** ptx_code = (char**)malloc(num_devices * sizeof(char*));
            for( unsigned int i=0; i<num_devices; ++i)
            {
                ptx_code[i] = (char*)malloc(binary_sizes[i]);
            }
            clGetProgramInfo(cpProgram, CL_PROGRAM_BINARIES, 0, ptx_code, NULL);
            
            // Find the index of the device of interest
            unsigned int idx = 0;
            while((idx < num_devices) && (devices[idx] != cdDevice))
            {
                ++idx;
            }
            
            // If the index is associated, log the result
            if(idx < num_devices)
            {
                // if a separate filename is supplied, dump ptx there
                if (NULL != cPtxFileName)
                {
                    LogErrorArg1("Writing ptx to separate file: %s ...", cPtxFileName);
                    FILE* pFileStream = NULL;
#ifdef _WIN32
                    fopen_s(&pFileStream, cPtxFileName, "wb");
#else
                    pFileStream = fopen(cPtxFileName, "wb");
#endif
                    
                    fwrite(ptx_code[idx], binary_sizes[idx], 1, pFileStream);
                    fclose(pFileStream);
                }
                else
                    LogErrorArg1("Program Binary:%s ", ptx_code[idx]);
            }
            
            // Cleanup
            free(devices);
            free(binary_sizes);
            for(unsigned int i = 0; i < num_devices; ++i)
            {
                free(ptx_code[i]);
            }
            free( ptx_code );
        }
        
        //Error String for ocl
        const char* oclErrorString(cl_int error)
        {
            static const char* errorString[] = {
                "CL_SUCCESS",
                "CL_DEVICE_NOT_FOUND",
                "CL_DEVICE_NOT_AVAILABLE",
                "CL_COMPILER_NOT_AVAILABLE",
                "CL_MEM_OBJECT_ALLOCATION_FAILURE",
                "CL_OUT_OF_RESOURCES",
                "CL_OUT_OF_HOST_MEMORY",
                "CL_PROFILING_INFO_NOT_AVAILABLE",
                "CL_MEM_COPY_OVERLAP",
                "CL_IMAGE_FORMAT_MISMATCH",
                "CL_IMAGE_FORMAT_NOT_SUPPORTED",
                "CL_BUILD_PROGRAM_FAILURE",
                "CL_MAP_FAILURE",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "CL_INVALID_VALUE",
                "CL_INVALID_DEVICE_TYPE",
                "CL_INVALID_PLATFORM",
                "CL_INVALID_DEVICE",
                "CL_INVALID_CONTEXT",
                "CL_INVALID_QUEUE_PROPERTIES",
                "CL_INVALID_COMMAND_QUEUE",
                "CL_INVALID_HOST_PTR",
                "CL_INVALID_MEM_OBJECT",
                "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR",
                "CL_INVALID_IMAGE_SIZE",
                "CL_INVALID_SAMPLER",
                "CL_INVALID_BINARY",
                "CL_INVALID_BUILD_OPTIONS",
                "CL_INVALID_PROGRAM",
                "CL_INVALID_PROGRAM_EXECUTABLE",
                "CL_INVALID_KERNEL_NAME",
                "CL_INVALID_KERNEL_DEFINITION",
                "CL_INVALID_KERNEL",
                "CL_INVALID_ARG_INDEX",
                "CL_INVALID_ARG_VALUE",
                "CL_INVALID_ARG_SIZE",
                "CL_INVALID_KERNEL_ARGS",
                "CL_INVALID_WORK_DIMENSION",
                "CL_INVALID_WORK_GROUP_SIZE",
                "CL_INVALID_WORK_ITEM_SIZE",
                "CL_INVALID_GLOBAL_OFFSET",
                "CL_INVALID_EVENT_WAIT_LIST",
                "CL_INVALID_EVENT",
                "CL_INVALID_OPERATION",
                "CL_INVALID_GL_OBJECT",
                "CL_INVALID_BUFFER_SIZE",
                "CL_INVALID_MIP_LEVEL",
                "CL_INVALID_GLOBAL_WORK_SIZE",
            };
            
            const int errorCount = sizeof(errorString) / sizeof(errorString[0]);
            
            const int index = -error;
            
            return (index >= 0 && index < errorCount) ? errorString[index] : "Unspecified Error";
        }
        
        // Helper function to get OpenCL image format string (channel order and type) from constant
        // *********************************************************************
        const char* oclImageFormatString(cl_uint uiImageFormat)
        {
            // cl_channel_order
            if (uiImageFormat == CL_R)return "CL_R";
            if (uiImageFormat == CL_A)return "CL_A";
            if (uiImageFormat == CL_RG)return "CL_RG";
            if (uiImageFormat == CL_RA)return "CL_RA";
            if (uiImageFormat == CL_RGB)return "CL_RGB";
            if (uiImageFormat == CL_RGBA)return "CL_RGBA";
            if (uiImageFormat == CL_BGRA)return "CL_BGRA";
            if (uiImageFormat == CL_ARGB)return "CL_ARGB";
            if (uiImageFormat == CL_INTENSITY)return "CL_INTENSITY";
            if (uiImageFormat == CL_LUMINANCE)return "CL_LUMINANCE";
            
            // cl_channel_type
            if (uiImageFormat == CL_SNORM_INT8)return "CL_SNORM_INT8";
            if (uiImageFormat == CL_SNORM_INT16)return "CL_SNORM_INT16";
            if (uiImageFormat == CL_UNORM_INT8)return "CL_UNORM_INT8";
            if (uiImageFormat == CL_UNORM_INT16)return "CL_UNORM_INT16";
            if (uiImageFormat == CL_UNORM_SHORT_565)return "CL_UNORM_SHORT_565";
            if (uiImageFormat == CL_UNORM_SHORT_555)return "CL_UNORM_SHORT_555";
            if (uiImageFormat == CL_UNORM_INT_101010)return "CL_UNORM_INT_101010";
            if (uiImageFormat == CL_SIGNED_INT8)return "CL_SIGNED_INT8";
            if (uiImageFormat == CL_SIGNED_INT16)return "CL_SIGNED_INT16";
            if (uiImageFormat == CL_SIGNED_INT32)return "CL_SIGNED_INT32";
            if (uiImageFormat == CL_UNSIGNED_INT8)return "CL_UNSIGNED_INT8";
            if (uiImageFormat == CL_UNSIGNED_INT16)return "CL_UNSIGNED_INT16";
            if (uiImageFormat == CL_UNSIGNED_INT32)return "CL_UNSIGNED_INT32";
            if (uiImageFormat == CL_HALF_FLOAT)return "CL_HALF_FLOAT";
            if (uiImageFormat == CL_FLOAT)return "CL_FLOAT";
            
            // unknown constant
            return "Unknown";
        }


    }
}
