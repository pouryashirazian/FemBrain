//
//  OclHelperFuncs.h
//  hifem
//
//  Created by pshiraz on 1/28/14.
//  Copyright (c) 2014 pshiraz. All rights reserved.
//

#ifndef hifem_OclHelperFuncs_h
#define hifem_OclHelperFuncs_h

#include "selectcl.h"

namespace PS {

    namespace CL {
        cl_int oclPlatformID(cl_platform_id* clSelectedPlatformID, const char* lpStrProvider = "NVIDIA");
        //static void oclPrintDevInfo(int iLogMode, cl_device_id device);
        
        
        /*! Get and return device capability
         * @return the 2 digit integer representation of device Cap (major minor). return -1 if NA
         * @param device         OpenCL id of the device
         */
        int oclGetDevCap(cl_device_id device);
        
        /*! Gets the id of the nth device from the context
         * @return the id or -1 when out of range
         * @param cxGPUContext         OpenCL context
         * @param device_idx            index of the device of interest
         */
        cl_device_id oclGetDev(cl_context cxGPUContext, unsigned int device_idx);
        
        /*! Gets the id of device with maximal FLOPS from the context
         * @return the id
         * @param cxGPUContext         OpenCL context
         */
        cl_device_id oclGetMaxFlopsDev(cl_context cxGPUContext);
        
        /*! Get and log the binary (PTX) from the OpenCL compiler for the requested program & device
         * @param cpProgram    OpenCL program
         * @param cdDevice     device of interest
         * @param const char*  cPtxFileName   optional PTX file name
         */
        void oclLogPtx(cl_program cpProgram, cl_device_id cdDevice, const char* cPtxFileName);
        
        /*!
         * Returns a string representing the image format
         * @param uiImageFormat the code representing the image format
         */
        const char* oclImageFormatString(cl_uint uiImageFormat);
        
        /*!
         * Return an error string regarding a returned error.
         * @param error integer code of the error
         */
        const char* oclErrorString(cl_int error);

    }

}

#endif
