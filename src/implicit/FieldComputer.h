//
//  FieldComputer.h
//  hifem
//
//  Created by pshiraz on 1/15/14.
//  Copyright (c) 2014 pshiraz. All rights reserved.
//

#ifndef hifem_FieldComputer_h
#define hifem_FieldComputer_h

#include "LinearBlobTree.h"
#include "graphics/ComputeDevice.h"
#include "graphics/SGMesh.h"

using namespace PS;
using namespace PS::SG;
using namespace PS::CL;

namespace PS {
    namespace SKETCH {

    	//Voxel Grid Points
		struct VoxelGrid {
			U32 axialCount[3];
			U32 total;
			float cellsize;
		};

    
        class FieldComputer : public SGMesh {
        public:
            FieldComputer();
            FieldComputer(const LinearBlobTree& blob);
            virtual ~FieldComputer();
            
            //Draw
            void draw();

            //Fields for the voxel grid
            int fieldsForVoxelGrid(float cellsize, bool stackless = true);

            //Field at a point
            float field(const vec3f& p);
            float field_stackless(const vec3f& p);
            
            //Field for an array of points
            int field(U32 ctVertices, U32 step, vector<float>& vertices);
            int field_stackless(U32 ctVertices, U32 step, vector<float>& vertices);
        protected:
            void init();
            bool setBlob(const LinearBlobTree& blob);
            void cleanup();
            void clearBlobBuffers();
        private:
            LinearBlobTree m_blob;
            CL::ComputeDevice* m_lpGPU;
            CL::ComputeKernel* m_lpKernelFieldArrayStackLess;
            CL::ComputeKernel* m_lpKernelFieldArrayStackBased;
            CL::ComputeKernel* m_lpKernelFieldVoxelGridStackLess;
            CL::ComputeKernel* m_lpKernelFieldVoxelGridStackBased;

            //Memory Buffers
            CL::Buffer m_inMemHeader;
            CL::Buffer m_inMemOps;
            CL::Buffer m_inMemPrims;
            CL::Buffer m_inMemMtx;

            bool m_blobLoaded;
        };
        
        
        
    }
}


#endif
