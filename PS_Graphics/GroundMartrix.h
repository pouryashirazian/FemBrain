/*
 * GroundMartrix.h
 *
 *  Created on: Mar 10, 2013
 *      Author: pourya
 */

#ifndef GROUNDMARTRIX_H_
#define GROUNDMARTRIX_H_

#include "PS_GLMeshBuffer.h"

class GroundMatrix : public GLMeshBuffer {

public:
	GroundMatrix(int rows = 32, int cols = 32, float step = 1.0f);
	virtual ~GroundMatrix();


};



#endif /* GROUNDMARTRIX_H_ */
