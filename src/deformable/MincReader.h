/*
 * MincReader.h
 *
 *  Created on: Mar 30, 2013
 *      Author: pourya
 */

#ifndef MINCREADER_H_
#define MINCREADER_H_


class MincReader {
public:
	MincReader(const char* chrFilePath);
	~MincReader();

	int read(const char* chrFilePath);

};



#endif /* MINCREADER_H_ */
