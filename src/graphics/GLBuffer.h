/*
 * GLBuffer.h
 *
 *  Created on: Sep 4, 2014
 *      Author: pourya
 */

#ifndef GLBUFFER_H_
#define GLBUFFER_H_

namespace PS {
namespace GL {


template <typename T, class UsagePolicy>
class GLBuffer {
public:
	GLBuffer();
	virtual ~GLBuffer();


protected:
	U32 m_handle;
};

}
}



#endif /* GLBUFFER_H_ */
