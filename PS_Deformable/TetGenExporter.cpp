/*
 * TetGenExporter.cpp
 *
 *  Created on: Mar 7, 2013
 *      Author: pourya
 */

#include "TetGenExporter.h"
#include <AA_TetGen/tetgen.h>

TetGenExporter::TetGenExporter() {

}

TetGenExporter::~TetGenExporter() {

}

int TetGenExporter::tesselate(U32 ctVertices, float* arrVertices,
								  U32 ctTriangles, U32* arrElements,
								  const char* chrInputTitle,
								  const char* chrOutputTitle)
{
	tetgenio in, out;
	tetgenio::facet *f;
	tetgenio::polygon *p;

	// All indices start from 1.
	in.firstnumber = 0;

	in.numberofpoints = ctVertices;
	in.pointlist = new REAL[in.numberofpoints * 3];
	for(U32 i=0; i < ctVertices*3; i++)
		in.pointlist[i] = arrVertices[i];

	in.numberoffacets = ctTriangles;
	in.facetlist = new tetgenio::facet[in.numberoffacets];
	in.facetmarkerlist = new int[in.numberoffacets];

	//Add all triangles
	for(U32 i=0; i<ctTriangles; i++) {
		f = &in.facetlist[i];
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		f->numberofholes = 0;
		f->holelist = NULL;
		p = &f->polygonlist[0];
		p->numberofvertices = 3;
		p->vertexlist = new int[p->numberofvertices];
		p->vertexlist[0] = arrElements[i*3];
		p->vertexlist[1] = arrElements[i*3 + 1];
		p->vertexlist[2] = arrElements[i*3 + 2];


		// Set 'in.facetmarkerlist'
		in.facetmarkerlist[i] = 0;
	}

	// Output the PLC to files 'barin.node' and 'barin.poly'.
	in.save_nodes(const_cast<char*>(chrInputTitle));
	in.save_poly(const_cast<char*>(chrInputTitle));

	// Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
	//   do quality mesh generation (q) with a specified quality bound
	//   (1.414), and apply a maximum volume constraint (a0.1).

	//tetrahedralize("pq1.414a0.1", &in, &out);
	tetrahedralize("pq", &in, &out);

	// Output mesh to files 'barout.node', 'barout.ele' and 'barout.face'.
	out.save_nodes(const_cast<char*>(chrOutputTitle));
	out.save_elements(const_cast<char*>(chrOutputTitle));
	out.save_faces(const_cast<char*>(chrOutputTitle));

	return 1;
}

