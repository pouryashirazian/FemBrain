/*
 * SGBulletCube.h
 *
 *  Created on: Sep 4, 2014
 *      Author: pourya
 */

#ifndef SGBULLETCUBE_H_
#define SGBULLETCUBE_H_

#include "SGBulletCDShape.h"

namespace PS {
namespace SG {

class SGBulletCube : public SGBulletShape {
public:
	SGBulletCube();
	virtual ~SGBulletCube();

	virtual void draw();
	virtual void timestep();
};

}
}




#endif /* SGBULLETCUBE_H_ */
