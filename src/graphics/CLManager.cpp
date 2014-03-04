//
//  CLManager.cpp
//  hifem
//
//  Created by pshiraz on 1/15/14.
//  Copyright (c) 2014 pshiraz. All rights reserved.
//

#include "CLManager.h"
#include "base/Logger.h"
#include "base/MathBase.h"

using namespace PS;

namespace PS {
    namespace CL {
        CLManager::CLManager() {
            init();
        }
        
        CLManager::~CLManager() {
            
        }
        
        void CLManager::init() {
            m_lpDevice = new ComputeDevice(ComputeDevice::dtGPU, true, false);
            if(m_lpDevice) {
                m_lpDevice->printInfo();
                
                if(m_lpDevice->isDoubleFPSupported())
                    LogInfo("Double FLT is supported.");
                else
                    LogError("Double is not available on this platform.");
            }
        }
        
        void CLManager::cleanup() {
            SAFE_DELETE(m_lpDevice);
        }
    }
}
