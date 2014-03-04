/*
 * TetGenExporter.h
 *
 *  Created on: Mar 7, 2013
 *      Author: pourya
 */

#ifndef TETGENEXPORTER_H_
#define TETGENEXPORTER_H_

#include <vector>
#include "base/MathBase.h"

using namespace std;

class TetGenExporter {
public:
	TetGenExporter();
	//TetGenExporter

	virtual ~TetGenExporter();

	static int tesselate(U32 ctVertices, float* arrVertices,
						    U32 ctTriangles, U32* arrElements,
						    const char* chrInputTitle = "IsoSurfMeshIn",
							const char* chrOutputTitle = "TetrahedraMeshOut");

	static int tesselate(const vector<float>& inTriVertices,
						 const vector<U32>& inTriElements,
						 vector<double>& outTetVertices,
						 vector<U32>& outTetElements);


};


#endif /* TETGENEXPORTER_H_ */
