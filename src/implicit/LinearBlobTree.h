/*
 * LinearBlobTree.h
 *
 *  Created on: Dec 25, 2013
 *      Author: pourya
 */

#ifndef LINEARBLOBTREE_H_
#define LINEARBLOBTREE_H_

#include "base/DataArray.h"
#include "base/String.h"
#include "PolyMemManager.h"
#include "graphics/AABB.h"


//Defines an empty index to jump out of branch
#define NULL_BLOB 0xFFFF

//DATASIZES
#define DATASIZE_HEADER		12
#define DATASIZE_OPERATOR	16
#define DATASIZE_PRIMITIVE	20
#define DATASIZE_PRIMBOX	12
#define DATASIZE_OPBOX		 8
#define PRIM_MATRIX_STRIDE  12
#define BOX_MATRIX_STRIDE   16

#define DATASIZE_OPERATOR_F4	DATASIZE_OPERATOR/4
#define DATASIZE_PRIMITIVE_F4	DATASIZE_PRIMITIVE/4
#define PRIM_MATRIX_STRIDE_F4 	PRIM_MATRIX_STRIDE/4
#define BOX_MATRIX_STRIDE_F4 	BOX_MATRIX_STRIDE/4


//OFFSETS in HEADER
#define OFFSET4_HEADER_LOWER	0
#define OFFSET4_HEADER_UPPER	1
#define OFFSET4_HEADER_PARAMS	2

#define OFFSET_HEADER_BBOX_LO_X 	0
#define OFFSET_HEADER_BBOX_LO_Y 	1
#define OFFSET_HEADER_BBOX_LO_Z 	2
#define OFFSET_HEADER_BBOX_HI_X 	4
#define OFFSET_HEADER_BBOX_HI_Y 	5
#define OFFSET_HEADER_BBOX_HI_Z 	6
#define OFFSET_HEADER_COUNT_PRIMS 	8
#define OFFSET_HEADER_COUNT_OPS 	9
#define OFFSET_HEADER_COUNT_MTX		10
#define OFFSET_HEADER_START 		11

//OFFSETS in OPERATORS
#define OFFSET4_OP_TYPE			0
#define OFFSET4_OP_RES			1
#define OFFSET4_AABB_LO			2
#define OFFSET4_AABB_HI			3

#define OFFSET_OP_TYPE			0
#define OFFSET_OP_LC			1
#define OFFSET_OP_RC			2
#define OFFSET_OP_NEXT			3
#define OFFSET_OP_RES_X			4
#define OFFSET_OP_RES_Y			5
#define OFFSET_OP_RES_Z			6
#define OFFSET_OP_FLAGS			7
#define OFFSET_OP_AABB_LO		8
#define OFFSET_OP_AABB_HI		12

//OFFSETS in PRIMITIVES
#define OFFSET4_PRIM_TYPE		0
#define OFFSET4_PRIM_POS		1
#define OFFSET4_PRIM_DIR		2
#define OFFSET4_PRIM_RES		3
#define OFFSET4_PRIM_COLOR		4

#define OFFSET_PRIM_TYPE		0
#define OFFSET_PRIM_IDX_MATRIX 	1
#define OFFSET_PRIM_PARENT 		2
#define OFFSET_PRIM_SIBLING 	3
#define OFFSET_PRIM_POS		 	4
#define OFFSET_PRIM_DIR	 		8
#define OFFSET_PRIM_RES		 	12
#define OFFSET_PRIM_COLOR	 	16

#define OFFSET_PRIMBOX_LO		0
#define OFFSET_PRIMBOX_HI		4
#define OFFSET_PRIMBOX_SEED		8

#define OFFSET_OPBOX_LO			0
#define OFFSET_OPBOX_HI			4

//Error Codes
#define BLOB_SUCCESS 1
#define ERR_BLOB_INVALID_PARENT_INDEX  -1
#define ERR_BLOB_INVALID_PRIMITIVE_TYPE -2
#define ERR_BLOB_MATRIX_STORAGE_OVERFLOW -3
#define ERR_BLOB_PRIM_STORAGE_OVERFLOW	-4
#define ERR_BLOB_OP_STORAGE_OVERFLOW	-5
#define ERR_BLOB_NO_TRAVERSAL_ROUTE_AVAILABLE	-6
#define ERR_BLOB_INVALID_PRIM_INDEX -7
#define ERR_BLOB_INVALID_OP_INDEX -8


using namespace PS;
using namespace PS::SIMDPOLY;

namespace PS {
namespace SKETCH {

/*!
 * Linear BlobTree for fast Rendering on the GPU
 */
class LinearBlobTree {
public:
	LinearBlobTree();
	LinearBlobTree(const LinearBlobTree& rhs);
	virtual ~LinearBlobTree();

	//IO
	bool load(const AnsiStr& strFilePath);
	bool store(const AnsiStr& strFilePath);
	void copyFrom(const LinearBlobTree& rhs);

	//Stats
	U32 countPrimitives() const {return m_ctPrims;}
	U32 countOperators() const {return m_ctOps;}
	U32 countMtxNodes() const {return m_ctMtxNode;}
	U32 countMtxBoxes() const {return m_ctMtxBox;}
	AABB aabb() const {return m_aabb;}
	bool isPrimIndex(int index) const {return (index>=0 && index < (int)m_ctPrims);}
	bool isOpIndex(int index) const {return (index >=0 && index < (int)m_ctOps);}
	bool isMtxNodeIndex(int index) const {return (index>=0 && index < (int)m_ctMtxNode);}
	bool isMtxBoxIndex(int index) const {return (index >=0 && index < (int)m_ctMtxBox);}

	//Actions
	bool recomputeAABB();
	AABB computePrimAABB(int idxPrim);
	bool computeOpsAABB(int idxRoot, AABB& aabb);
	int  computeAllInstancedNodesAABB();
	AABB getPrimAABB(int idxPrim) const;
	AABB getOpAABB(int idxOp) const;


	//Updates Header by writing latest stats
	void updateHeader(U32 idxRoot = NULL_BLOB);

	/*!
	 * Resets BlobTree to zero ops and prims and 1 identity matrix for nodes
	 */
	void actReset();

	/*!
	 * Add a primitive matrix node to the blob
	 * @returns -1 if not successful or the index of the node otherwise
	 */
	int actAddMtxNode(const mat44f& mtx, int idxPrim = -1);

	/*!
	 * Adds an operator to the data-structure
	 * @return -1 if an error occured otherwise the index of the op
	 */
	int actAddOp(OperatorType opType, U32 idxLC, U32 idxRC, U32 idxNext,
				const vec3f& res, int flags);

	bool actDeleteOp(int idxOp);

	/*!
	 * Adds a primitive to the data-structure.
	 * @return -1 is an error occured otherwise the index of the primitive
	 */
	int actAddPrim(PrimitiveType primType, int idxParent,
					  const vec3f& pos, const vec3f& dir,
					  const vec3f& res, const vec4f& color);

	bool actDeletePrim(int idxPrim);

	//Determines the optimum traversal route on the GPU
	bool setTraversalRoute();

	//Prints the tree into a text file
	void print(const AnsiStr& strFilePath) const;

	LinearBlobTree& operator=(const LinearBlobTree& rhs);

private:

protected:
	AnsiStr m_strModelFilePath;

	//Count of primitives and ops
	U32 m_ctPrims;
	U32 m_ctOps;
	U32 m_ctMtxNode;
	U32 m_ctMtxBox;

	//Bounding Box
	AABB m_aabb;

public:
	//Header
	DataArray<float, DATASIZE_HEADER> arrHeader;

	//Prims Data
	DataArray<float, MAX_TREE_NODES * DATASIZE_PRIMITIVE> arrPrims;

	//Operators Data
	DataArray<float, MAX_TREE_NODES * DATASIZE_OPERATOR> arrOps;

	//Node Matrix
	DataArray<float, MAX_MTX_NODES * PRIM_MATRIX_STRIDE> arrMtxNodes;

	//Box Matrix
	DataArray<float, MAX_MTX_NODES * BOX_MATRIX_STRIDE> arrMtxBoxes;


	//Prim Boxes
	DataArray<float, MAX_TREE_NODES * DATASIZE_PRIMBOX> arrPrimBoxes;

	//Op Boxes
	DataArray<float, MAX_TREE_NODES * DATASIZE_OPBOX>	arrOpBoxes;

};


}
}

#endif /* LINEARBLOBTREE_H_ */
