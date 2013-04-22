/*
 * CLMeshBuffer.h
 *
 *  Created on: Apr 14, 2013
 *      Author: pourya
 */

#ifndef CLMESHBUFFER_H_
#define CLMESHBUFFER_H_

#include "PS_GLMeshBuffer.h"
#include "PS_ComputeDevice.h"

#define ERR_DEVICE_NOT_VALID -1


namespace PS{
namespace HPC {

/*!
 * To be able to read-back mesh and apply modifications using opencl kernel
 * This class provides transparent access to the underlying mesh buffer using
 * OpenCL, GL functionality to read-back values
 */
class CLMeshBuffer {
public:
	/*!
	 * Readback vertex attribs within lpBuffer from device
	 */
	static bool ReadbackMeshVertexAttribCL(ComputeDevice* lpDevice,
												const GLMeshBuffer* lpBuffer,
												VertexAttribType attrib,
												U32& count, U32& fstep,
												vector<float>& values);

	/*!
	 * Readback face attributes
	 */
	static bool ReadbackMeshFaceCL(ComputeDevice* lpDevice,
									   const GLMeshBuffer* lpBuffer,
									   U32& count, U32& istep,
									   vector<U32>& elements);

	/*!
	 * Copies the source mesh buffer to the destination buffer
	 */
	static bool CopyMeshBufferCL(ComputeDevice* lpDevice,
									 const GLMeshBuffer* lpSource,
									 GLMeshBuffer* lpDest);

	/*!
	 * Store as obj mesh
	 */
	static bool StoreAsObjMesh(const char* chrFilePath,
								ComputeDevice* lpDevice,
								const GLMeshBuffer* lpBuffer);

	/*!
	 * Prepares Mesh Buffer for drawing normals
	 */
	static GLMeshBuffer* PrepareMeshBufferNormals(ComputeDevice* lpDevice,
													  GLMeshBuffer* lpBuffer,
													  float len = 0.3f);
};



}
}
#endif /* CLMESHBUFFER_H_ */
