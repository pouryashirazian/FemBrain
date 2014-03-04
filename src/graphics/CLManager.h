//
//  CLManager.h
//  hifem
//
//  Created by pshiraz on 1/15/14.
//  Copyright (c) 2014 pshiraz. All rights reserved.
//

#ifndef hifem_CLManager_h
#define hifem_CLManager_h

#include "base/String.h"
#include "loki/Singleton.h"
#include "ComputeDevice.h"
#include "base/FastAccessToNamedResource.h"

using namespace Loki;
using namespace PS;

namespace PS {
    namespace CL {
        
        class CLManager {
        public:
            CLManager();
            virtual ~CLManager();
            
            ComputeDevice* device() const {return m_lpDevice;}
            
        protected:
            void init();
            void cleanup();
        protected:
            ComputeDevice* m_lpDevice;
        };
        
        //Singleton Access to scene graph
        typedef SingletonHolder<CLManager, CreateUsingNew, PhoenixSingleton> TheCLManager;

        
    }
}


#endif
