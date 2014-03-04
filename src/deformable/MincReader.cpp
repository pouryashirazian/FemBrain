/*
 * MincReader.cpp
 *
 *  Created on: Mar 30, 2013
 *      Author: pourya
 */
#include "MincReader.h"
#include <minc2.h>
#include "base/Logger.h"
#include "base/FileDirectory.h"

using namespace PS;

MincReader::MincReader(const char* chrFilePath) {
	read(chrFilePath);
}

MincReader::~MincReader() {

}

int MincReader::read(const char* chrFilePath) {
	mihandle_t minc_volume;
	midimhandle_t dimentions[3];
	double voxel;

	unsigned long location[3];


	if(!PS::FILESTRINGUTILS::FileExists(AnsiStr(chrFilePath)))
		LogErrorArg1("Minc file does not exist! %s", chrFilePath);

	//Open the volume for reading
	int result = miopen_volume(chrFilePath, MI2_OPEN_READ, &minc_volume);
	if(result != MI_NOERROR) {
		LogErrorArg1("Error opening input file: %s", chrFilePath);
		return -1;
	}

	//Get dimensions
	result = miget_volume_dimensions(minc_volume, MI_DIMCLASS_SPATIAL, MI_DIMATTR_ALL, MI_DIMORDER_FILE, 3, dimentions);
	unsigned int sizes[3];

	//Read dimension sizes
	result = miget_dimension_sizes(dimentions, 3, sizes);
	LogInfoArg3("Volume Dimension sizes are: [%d, %d, %d]", sizes[0], sizes[1], sizes[2]);

	double* voxels = (double *)malloc(sizes[0] * sizes[1] * sizes[2] * sizeof(double));

	unsigned long slabcount[3];
	slabcount[0] = 1;
	slabcount[1] = sizes[1];
	slabcount[2] = sizes[2];

	unsigned long start[3];
	start[0] = start[1] = start[2] = 0;

	double* pSlab = NULL;
	for(int slice = 0; slice < sizes[0]; slice++) {
		start[0] = (unsigned long)slice;
		pSlab = &voxels[slice * sizes[1] * sizes[2]];
		if(miget_real_value_hyperslab(minc_volume, MI_TYPE_DOUBLE, start, slabcount, pSlab) != MI_NOERROR) {
			LogErrorArg1("Unable to fetch slab# %d", slice);
		}
	}

	//Compute Min and Max
	double vMin = voxels[0];
	double vMax = voxels[0];
	U32 xyz = sizes[0] * sizes[1] * sizes[2];
	for(U32 i=0; i<xyz; i++) {
		if(voxels[i] < vMin)
			vMin = voxels[i];
		if(voxels[i] > vMax)
			vMax = voxels[i];
	}
	LogInfoArg2("Min and Max Voxels are %.3f, %.3f", vMin, vMax);


	/*
	unsigned long voxel_location[3];

	U64 y = sizes[1];
	U64 z = sizes[2];
	U64 yz = y * z;
	for(U64 i=0; i<sizes[0]; i++) {
		for(U64 j=0; j<sizes[1]; j++) {
			for(U64 k=0; k<sizes[2]; k++) {
				voxel_location[0] = i;
				voxel_location[1] = j;
				voxel_location[2] = k;

				double* pVoxel = &voxels[i * yz + j * z + k];
				miget_real_value(minc_volume, voxel_location, 3, pVoxel);
			}
		}
	}
	*/

	//Create opencl 3D texture
	free(voxels);

	//Close the volume
	miclose_volume(minc_volume);

	return 1;
}



