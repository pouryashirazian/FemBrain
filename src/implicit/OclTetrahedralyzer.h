/*
 * OclTetrahedralyzer.h
 *
 *  Created on: Dec 26, 2013
 *      Author: pourya
 */

#ifndef OCLTETRAHEDRALYZER_H_
#define OCLTETRAHEDRALYZER_H_

#include "LinearBlobTree.h"
#include "base/Vec.h"
#include "graphics/ComputeDevice.h"
#include "graphics/SGMesh.h"
#include "graphics/OclScan.h"

using namespace PS::SIMDPOLY;
using namespace PS::CL;
using namespace PS::MATH;
using namespace PS::SKETCH;

namespace PS {
namespace SKETCH {

class GPUTetra : public SG::SGMesh {
public:
	GPUTetra();
    GPUTetra(const LinearBlobTree& blob);
	virtual ~GPUTetra();
    
    void run();
    void drawGrid();
    
protected:
    void init();
    bool setBlob(const LinearBlobTree& blob);
    

protected:
    LinearBlobTree m_blob;
    bool m_isLoaded;


};


}
}

#endif /* OCLTETRAHEDRALYZER_H_ */
