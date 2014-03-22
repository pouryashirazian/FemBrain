/*
 * LinearBlobTree.cpp
 *
 *  Created on: Dec 25, 2013
 *      Author: pourya
 */
#include "LinearBlobTree.h"
#include "ReadSceneModel.h"
#include "base/Logger.h"

namespace PS {
namespace SKETCH {
	//Node class used for printing the tree
	struct NODE {
		U32 index;
		U32 depth;
		U8 isOp;

		NODE() :
				index(0), depth(0), isOp(1) {
		}
		NODE(U32 index_, U32 depth_, U8 isOp_) {
			index = index_;
			depth = depth_;
			isOp = isOp_;
		}
	};

	//LinearTree
	LinearBlobTree::LinearBlobTree() {
		this->actReset();
	}

	LinearBlobTree::LinearBlobTree(const LinearBlobTree& rhs) {
		m_ctOps = m_ctPrims = m_ctMtxBox = m_ctMtxNode = 0;
		this->copyFrom(rhs);
	}

	LinearBlobTree::~LinearBlobTree() {
		m_ctOps = m_ctPrims = 0;
	}

	bool LinearBlobTree::load(const AnsiStr& strFilePath)
	{
		if(!PS::FILESTRINGUTILS::FileExists(strFilePath)) {
			LogErrorArg1("Input model file doesnot exist! FILE:%s", strFilePath.cptr());
			return false;
		}
		//Read in SOA format
		SOABlobPrims tempPrims;
		SOABlobOps tempOps;
		SOABlobNodeMatrices tempMtxNode;
		SOABlobBoxMatrices tempMtxBox;

		ModelReader mr(&tempPrims, &tempOps, &tempMtxNode, &tempMtxBox);
		if(mr.read(strFilePath.cptr()) != MODELREAD_SUCCESS)
			return false;

		//Model File Path
		m_strModelFilePath = strFilePath;

		//Set Header
		m_aabb = AABB(vec3f(&tempPrims.bboxLo.x), vec3f(&tempPrims.bboxHi.x));
		arrHeader[0] = tempPrims.bboxLo.x;
		arrHeader[1] = tempPrims.bboxLo.y;
		arrHeader[2] = tempPrims.bboxLo.z;
		arrHeader[3] = 1.0f;
		arrHeader[4] = tempPrims.bboxHi.x;
		arrHeader[5] = tempPrims.bboxHi.y;
		arrHeader[6] = tempPrims.bboxHi.z;
		arrHeader[7] = 1.0f;

		arrHeader[8] = static_cast<float>(tempPrims.count);
		arrHeader[9] = static_cast<float>(tempOps.count);
		arrHeader[10] = static_cast<float>(tempMtxNode.count);

		//Root Op
		arrHeader[11] = NULL_BLOB;

		//Count
		m_ctPrims 	= tempPrims.count;
		m_ctOps 	= tempOps.count;
		m_ctMtxNode = tempMtxNode.count;
		m_ctMtxBox 	= tempMtxBox.count;

		//Copy Matrices
		memcpy(arrMtxNodes.ptr(), &tempMtxNode.matrix, arrMtxNodes.size());
		memcpy(arrMtxBoxes.ptr(), &tempMtxBox.matrix, arrMtxBoxes.size());

		//Prims
		for(U32 i=0; i < tempPrims.count; i++)
		{
			//Update Parent/Link and Link Properties
			arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE] = static_cast<float>(tempPrims.type[i]);
			arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_IDX_MATRIX] = static_cast<float>(tempPrims.idxMatrix[i]);
			arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT] = 0;
			arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_SIBLING] = 0;

			//Pos
			arrPrims[i * DATASIZE_PRIMITIVE + 4] = tempPrims.posX[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 5] = tempPrims.posY[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 6] = tempPrims.posZ[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 7] = 0.0f;

			//Dir
			arrPrims[i * DATASIZE_PRIMITIVE + 8] = tempPrims.dirX[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 9] = tempPrims.dirY[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 10] = tempPrims.dirZ[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 11] = 0.0f;

			//Res
			arrPrims[i * DATASIZE_PRIMITIVE + 12] = tempPrims.resX[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 13] = tempPrims.resY[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 14] = tempPrims.resZ[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 15] = 0.0f;

			//Color
			arrPrims[i * DATASIZE_PRIMITIVE + 16] = tempPrims.colorX[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 17] = tempPrims.colorY[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 18] = tempPrims.colorZ[i];
			arrPrims[i * DATASIZE_PRIMITIVE + 19] = 1.0f;
		}

		//Ops
		if(tempOps.count > 0)
		{
			for(U32 i=0; i < tempOps.count; i++)
			{
				//Update Parent/Link and Link Properties
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_TYPE] = static_cast<float>(tempOps.type[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_LC]   = static_cast<float>(tempOps.opLeftChild[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_RC]   = static_cast<float>(tempOps.opRightChild[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = NULL_BLOB;

				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_RES_X] = static_cast<float>(tempOps.resX[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_RES_Y] = static_cast<float>(tempOps.resY[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_RES_Z] = static_cast<float>(tempOps.resZ[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_FLAGS] = static_cast<float>(tempOps.opFlags[i]);

				//AABB
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO] = static_cast<float>(tempOps.bboxLoX[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 1] = static_cast<float>(tempOps.bboxLoY[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 2] = static_cast<float>(tempOps.bboxLoZ[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 3] = 1.0f;
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI] = static_cast<float>(tempOps.bboxHiX[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 1] = static_cast<float>(tempOps.bboxHiY[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 2] = static_cast<float>(tempOps.bboxHiZ[i]);
				arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 3] = 1.0f;
			}
		}
		else {
			arrOps[OFFSET_OP_NEXT] = NULL_BLOB;
		}

		//Print Tree to text file
		AnsiStr strPrintPath = strFilePath + ".print.txt";
		print(strPrintPath);

		//Set Traversal Route
		if(!setTraversalRoute())
		{
			LogError("Failed to find a tree traversal route in this BlobTree!");
			return false;
		}

		return true;
	}


	bool LinearBlobTree::store(const AnsiStr& strFilePath) {
		return false;
	}

	void LinearBlobTree::copyFrom(const LinearBlobTree& rhs) {
		m_strModelFilePath = rhs.m_strModelFilePath;
		m_aabb 		= rhs.m_aabb;
		m_ctMtxBox  = rhs.m_ctMtxBox;
		m_ctMtxNode = rhs.m_ctMtxNode;
		m_ctOps = rhs.m_ctOps;
		m_ctPrims = rhs.m_ctPrims;

		//Copy Main Arrays
		arrHeader = rhs.arrHeader;
		arrOps 	  = rhs.arrOps;
		arrPrims  = rhs.arrPrims;
		arrMtxBoxes = rhs.arrMtxBoxes;
		arrMtxNodes = rhs.arrMtxNodes;

		//Boxes
		arrPrimBoxes = rhs.arrPrimBoxes;
		arrOpBoxes = rhs.arrOpBoxes;
	}



	void LinearBlobTree::updateHeader(U32 idxRoot) {
		recomputeAABB();

		vec3f lo = m_aabb.lower();
		vec3f hi = m_aabb.upper();
		arrHeader[0] = lo.x;
		arrHeader[1] = lo.y;
		arrHeader[2] = lo.z;
		arrHeader[3] = 1.0f;
		arrHeader[4] = hi.x;
		arrHeader[5] = hi.y;
		arrHeader[6] = hi.z;
		arrHeader[7] = 1.0f;

		arrHeader[8] = static_cast<float>(m_ctPrims);
		arrHeader[9] = static_cast<float>(m_ctOps);
		arrHeader[10] = static_cast<float>(m_ctMtxNode);

		//Root Op
		arrHeader[11] = idxRoot;
	}

	//Actions
	int LinearBlobTree::actAddMtxNode(const mat44f& mtx, int idxPrim) {
		int idxMtx = m_ctMtxNode;
		m_ctMtxNode++;

		//Write matrix elements
		for(int row = 0; row < 3; row++)
			for(int col=0; col < 4; col++)
				arrMtxNodes[idxMtx * PRIM_MATRIX_STRIDE + row*4 + col] = mtx.element(row, col);

		//Update Associated Primitive
		if(isPrimIndex(idxPrim))
			arrPrims[idxPrim * DATASIZE_PRIMITIVE + OFFSET_PRIM_IDX_MATRIX] = idxMtx;
		return idxMtx;
	}

	int LinearBlobTree::actAddOp(OperatorType opType, U32 idxLC, U32 idxRC, U32 idxNext,
				  	  	  	  	  	 const vec3f& res, int flags) {
		int idxOp = m_ctOps;
		m_ctOps++;

		//Oparators
		arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_TYPE] = static_cast<float>(opType);
		arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_LC]   = static_cast<float>(idxLC);
		arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RC]   = static_cast<float>(idxRC);
		arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = static_cast<float>(idxNext);

		//Params
		arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RES_X] = res.x;
		arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RES_Y] = res.y;
		arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RES_Z] = res.z;
		arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_FLAGS] = static_cast<float>(flags);

		//Compute Bounding Boxes for all children in range
		AABB aabb;
		computeOpsAABB(idxOp, aabb);

		return idxOp;
	}

	bool LinearBlobTree::actDeleteOp(int idxOp) {
		return false;
	}

	int LinearBlobTree::actAddPrim(PrimitiveType primType, int idxParent,
					  	  	  	  	    const vec3f& pos, const vec3f& dir,
					  	  	  	  	    const vec3f& res, const vec4f& color) {
		if(isOpIndex(idxParent))
			return ERR_BLOB_INVALID_PARENT_INDEX;

		int idx = m_ctPrims;
		m_ctPrims++;

		//Update Parent/Link and Link Properties
		arrPrims[idx * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE] = static_cast<float>(primType);
		arrPrims[idx * DATASIZE_PRIMITIVE + OFFSET_PRIM_IDX_MATRIX] = 0;
		arrPrims[idx * DATASIZE_PRIMITIVE + OFFSET_PRIM_PARENT] = idxParent;
		arrPrims[idx * DATASIZE_PRIMITIVE + OFFSET_PRIM_SIBLING] = 0;

		//Pos
		arrPrims[idx * DATASIZE_PRIMITIVE + 4] = pos.x;
		arrPrims[idx * DATASIZE_PRIMITIVE + 5] = pos.y;
		arrPrims[idx * DATASIZE_PRIMITIVE + 6] = pos.z;
		arrPrims[idx * DATASIZE_PRIMITIVE + 7] = 0.0f;

		//Dir
		arrPrims[idx * DATASIZE_PRIMITIVE + 8] = dir.x;
		arrPrims[idx * DATASIZE_PRIMITIVE + 9] = dir.y;
		arrPrims[idx * DATASIZE_PRIMITIVE + 10] = dir.z;
		arrPrims[idx * DATASIZE_PRIMITIVE + 11] = 0.0f;

		//Res
		arrPrims[idx * DATASIZE_PRIMITIVE + 12] = res.x;
		arrPrims[idx * DATASIZE_PRIMITIVE + 13] = res.y;
		arrPrims[idx * DATASIZE_PRIMITIVE + 14] = res.z;
		arrPrims[idx * DATASIZE_PRIMITIVE + 15] = 0.0f;

		//Color
		arrPrims[idx * DATASIZE_PRIMITIVE + 16] = color.x;
		arrPrims[idx * DATASIZE_PRIMITIVE + 17] = color.y;
		arrPrims[idx * DATASIZE_PRIMITIVE + 18] = color.z;
		arrPrims[idx * DATASIZE_PRIMITIVE + 19] = color.w;

		//Compute BBox

		//Update Model BBOX
		vec3f offset(ISO_VALUE);
		m_aabb.set(pos - offset, pos + offset);

		//Updates Header which will recompute AABB
		updateHeader();

		return idx;
	}

	bool LinearBlobTree::actDeletePrim(int idxPrim) {
		if(!isPrimIndex(idxPrim))
			return false;
		arrPrims[idxPrim * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE] = primNULL;

		//Updates Header
		updateHeader();


		return true;
	}

	void LinearBlobTree::actReset() {
		m_ctOps = m_ctPrims = m_ctMtxBox = m_ctMtxNode = 0;
		mat44f mtx;
		mtx.identity();
		actAddMtxNode(mtx, -1);
	}


	bool LinearBlobTree::setTraversalRoute() {

		//Set OP next and Link Properties
		SIMPLESTACK<MAX_TREE_NODES> stkOps;
		SIMPLESTACK<MAX_TREE_NODES> stkLastBreak;

		stkOps.push(0);
		stkLastBreak.push(NULL_BLOB);

		//Process All Ops
		while(!stkOps.empty())
		{
			U16 idxOp = stkOps.top();
			U32 opFlags = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_FLAGS]);
			bool isBreak = (bool)((opFlags & ofBreak) >> 5);
			bool isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
			bool isRCOp  = (bool) (opFlags & ofRightChildIsOp);

			U32 idxLC = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_LC]);
			U32 idxRC = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RC]);

			//Process a break when we got two primitive kids
			if(isLCOp == false && isRCOp == false && stkLastBreak.empty() == false)
			{
				//Remove key op from stack
				stkOps.pop();
				U16 idxBreak = stkLastBreak.top();
				stkLastBreak.pop();

				//Set Root Op
				if(idxBreak == NULL_BLOB)
					arrHeader[OFFSET_HEADER_START] = idxOp;
				else
					arrOps[idxBreak * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = idxOp;

				if(isBreak)
					stkLastBreak.push(idxOp);

				//Pop while not at a break node
				while(!isBreak && !stkOps.empty())
				{
					idxOp = stkOps.top();
					stkOps.pop();

					opFlags = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_FLAGS]);
					isBreak = (bool)((opFlags & ofBreak) >> 5);
					//bool isRop   = (bool)((opFlags & ofIsRightOp) >> 4);
					//bool isUnary = (bool)((opFlags & ofIsUnaryOp) >> 3);
					//bool isRange = (bool)((opFlags & ofChildIndexIsRange) >> 2);
					isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
					isRCOp  = (bool) (opFlags & ofRightChildIsOp);

					//LC
					idxLC = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_LC]);

					//RC
					idxRC = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RC]);

					if(isBreak)
						stkLastBreak.push(idxOp);
					if(isLCOp)
						arrOps[idxLC * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = idxOp;
					if(isRCOp)
						arrOps[idxRC * DATASIZE_OPERATOR + OFFSET_OP_NEXT] = idxOp;
				}
			}
			else
			{
				if(isLCOp)
					stkOps.push(idxLC);

				if(isRCOp)
					stkOps.push(idxRC);
			}
		}

		//Test Route
		//Check next links and connections
		if(arrOps[OFFSET_OP_NEXT] != NULL_BLOB)
		{
			LogError("Root operator next pointer not set.");
			return false;
		}

		int ctErrors = 0;
		for(U32 i=1; i< m_ctOps; i++)
		{
			U16 next = arrOps[i * DATASIZE_OPERATOR + OFFSET_OP_NEXT];
			if(next < 0 || next >= m_ctOps)
			{
				ctErrors ++;
				LogErrorArg1("Next pointer not set properly at operator #%d", i);
			}
		}

		return (ctErrors == 0);
	}

	void LinearBlobTree::print(const AnsiStr& strFilePath) const
	{
		ofstream ofs;
		ofs.open(strFilePath.cptr(), ios::out);

		//Compute Tree Depth
		U32 maxDepth = 0;

		//Compute MAXDEPTH
		if(m_ctOps > 0)
		{
			GENERIC_PAIR_STACK<U32, MAX_TREE_NODES> stkDepth;
			stkDepth.push(0, 0);
			while(!stkDepth.empty())
			{
				U32 idxOp = stkDepth.topFirst();
				U32 depth = stkDepth.topSecond();
				maxDepth = MATHMAX(maxDepth, depth);
				stkDepth.pop();

				U32 opFlags = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_FLAGS]);
				bool isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
				bool isRCOp  = (bool) (opFlags & ofRightChildIsOp);

				U16 idxLC   = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_LC]);
				U16 idxRC   = static_cast<U32>(arrOps[idxOp * DATASIZE_OPERATOR + OFFSET_OP_RC]);

				if(isLCOp)
					stkDepth.push(idxLC, depth + 1);
				if(isRCOp)
					stkDepth.push(idxRC, depth + 1);
			}

			//Last Op has prims
			maxDepth ++;
		}


		if(m_ctOps > 0)
		{
			list<NODE> lstNodes;
			lstNodes.push_back(NODE(0, 0, 1));
			U32 currentDepth = -1;
			while(!lstNodes.empty())
			{
				NODE n = lstNodes.front();
				lstNodes.pop_front();

				if(n.depth != currentDepth)
				{
					ofs << endl;
					currentDepth = n.depth;
					for(U32 i=0; i<(maxDepth - currentDepth); i++)
						ofs << "\t\t\t";
				}

				if(n.isOp)
				{
					U32 opType  = static_cast<U32>(arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_TYPE]);
					U32 opFlags = static_cast<U32>(arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_FLAGS]);
					bool isUnary = (bool)((opFlags & ofIsUnaryOp) >> 3);
					bool isRange = (bool)((opFlags & ofIsUnaryOp) >> 2);
					bool isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
					bool isRCOp  = (bool) (opFlags & ofRightChildIsOp);

					U16 idxLC   = static_cast<U32>(arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_LC]);
					U16 idxRC   = static_cast<U32>(arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_RC]);

					//Visit Node
					AnsiStr strName = ModelReader::GetScriptOpName(opType);
					ofs << "OP#" << n.index << ":" << strName.substr(0, 5) << "\t";

					if(isRange)
					{
						for(int i = idxLC; i <= idxRC; i++)
							lstNodes.push_back(NODE(i, n.depth + 1, isLCOp));
					}
					else
					{
						lstNodes.push_back(NODE(idxLC, n.depth + 1, isLCOp));
						if(!isUnary)
							lstNodes.push_back(NODE(idxRC, n.depth + 1, isRCOp));
					}
				}
				else
				{
					U16 primType = static_cast<U16>(arrPrims[n.index * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE]);
					AnsiStr strName = ModelReader::GetScriptPrimName(primType);
					ofs << "PR#" << n.index << ":" << strName.substr(0, 5) << "\t";
				}
			}
		}
		else
		{
			U16 primType = static_cast<U16>(arrPrims[OFFSET_PRIM_TYPE]);
			AnsiStr strName = ModelReader::GetScriptPrimName(primType);
			ofs << "PR#0:" << strName.substr(0, 5) << "\t";
		}
		/*
		//Iterate over depths
		for(int i=0; i<=maxDepth; i++)
		{
			//Tabs
			for(int j=0; j<(maxDepth - i); j++)
				ofs << "\t\t\t";

			for(int j=0; j<arrNodes.size(); j++)
			{
				NODE n = arrNodes[j];
				if(n.depth == i)
				{
					if(n.isOp) {
						U32 opFlags = static_cast<U32>(m_arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_LINK_FLAGS]);
						bool isLCOp  = (bool)((opFlags & ofLeftChildIsOp) >> 1);
						bool isRCOp  = (bool) (opFlags & ofRightChildIsOp);
						U16 opType = static_cast<U16>(m_arrOps[n.index * DATASIZE_OPERATOR + OFFSET_OP_TYPE]);


						AnsiStr strName = ModelReader::GetScriptOpName(opType);
						ofs << "OP#" << n.index << ":" << strName.substr(0, 3) << "\t";
					}
					else {
						U16 primType = static_cast<U16>(m_arrPrims[n.index * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE]);
						AnsiStr strName = ModelReader::GetScriptPrimName(primType);
						ofs << "PR#" << n.index << ":" << strName.substr(0, 3) << "\t";
					}
				}
			}

			ofs << endl;
		}
		*/

		ofs.close();
	}

	LinearBlobTree& LinearBlobTree::operator=(const LinearBlobTree& rhs) {
		this->copyFrom(rhs);
		return (*this);
	}

	bool LinearBlobTree::recomputeAABB() {

		//Compute All Primitive BBoxes and entire AABB
		vec3f bboxLo = vec3f(FLT_MAX);
		vec3f bboxHi = vec3f(FLT_MIN);
		for(U32 i=0; i < m_ctPrims; i++) {
			AABB aabb = computePrimAABB(i);
			bboxLo = vec3f::minP(bboxLo, aabb.lower());
			bboxHi = vec3f::maxP(bboxHi, aabb.upper());
		}
		m_aabb.set(bboxLo, bboxHi);



		AABB aabbOps;
		if(!computeOpsAABB(0, aabbOps)) {
			LogError("Unable to compute AABB for ops!");
			return false;
		}


		int count = computeAllInstancedNodesAABB();
		if(count > 0)
			LogInfoArg1("Computed AABB for %d instanced nodes.", count);


		//Re-Compute operator bboxes for all ops
		if(!computeOpsAABB(0, aabbOps)) {
			LogError("Unable to re-compute AABB for ops!");
			return false;
		}

		return true;
	}

	AABB LinearBlobTree::computePrimAABB(int idxPrim) {
		vec3f curLo = vec3f(FLT_MAX, FLT_MAX, FLT_MAX);
		vec3f curHi = vec3f(FLT_MIN, FLT_MIN, FLT_MIN);

		if(!isPrimIndex(idxPrim))
			return AABB(curLo, curHi);

		int idxMtx = static_cast<int>(arrPrims[idxPrim * DATASIZE_PRIMITIVE + OFFSET_PRIM_IDX_MATRIX]);
		vec3f pos = vec3f(&arrPrims[idxPrim * DATASIZE_PRIMITIVE + OFFSET_PRIM_POS]);
		vec3f dir = vec3f(&arrPrims[idxPrim * DATASIZE_PRIMITIVE + OFFSET_PRIM_DIR]);
		vec4f res = vec4f(&arrPrims[idxPrim * DATASIZE_PRIMITIVE + OFFSET_PRIM_RES]);
		vec3f seed = pos;
		const float offset = ISO_VALUE;
		PrimitiveType primType = static_cast<PrimitiveType>(arrPrims[idxPrim * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE]);
		switch(primType)
		{
		case(primPoint): case(primNULL):
			{
				vec3f off = vec3f(offset);
				curLo = vec3f::sub(pos, off);
				curHi = vec3f::add(pos, off);
			}
			break;
		case(primLine):
			{
				vec3f s0 = pos;
				vec3f s1 = dir;
				vec3f expand = vec3f::add(vec3f::mul(offset, vec3f(1.0f, 1.0f, 1.0f)), vec3f::mul(3.0f*offset, vec3f::sub(s1, s0)));
				curLo = vec3f::sub(s0, expand);
				curHi = vec3f::add(s1, expand);
			}
			break;
		case(primRing):
			{
				float radius = res.x;

				vec3f dirComp = vec3f::sub(vec3f(1.0f, 1.0f, 1.0f), dir);
				radius += offset;

				vec3f expand = vec3f::add(vec3f::mul(radius + offset, dirComp), vec3f::mul(offset, dir));
				curLo = vec3f::sub(pos, expand);
				curHi = vec3f::add(pos, expand);
			}
			break;
		case(primDisc):
			{
				float radius = res.x;

				vec3f dirComp = vec3f::sub(vec3f(1.0f, 1.0f, 1.0f), dir);
				radius += offset;

				vec3f expand = vec3f::add(vec3f::mul(radius + offset, dirComp), vec3f::mul(offset, dir));
				curLo = vec3f::sub(pos, expand);
				curHi = vec3f::add(pos, expand);
			}
			break;
		case(primCylinder):
			{
				vec3f s0 = pos;
				float radius = res.x;
				float height = res.y;

				vec3f s1 = vec3f::add(s0, vec3f::mul(height, dir));
				vec3f expand = vec3f::add(vec3f::mul(offset + radius, vec3f(1.0f, 1.0f, 1.0f)), vec3f::mul(0.5f*offset, dir));
				curLo = vec3f::sub(s0, expand);
				curHi = vec3f::add(s1, expand);
			}
			break;
		case(primCube):
			{
				vec3f s0 = pos;
				float side = res.x + offset;

				curLo = vec3f::sub(s0, vec3f(side, side, side));
				curHi = vec3f::add(s0, vec3f(side, side, side));
			}
			break;
		case(primTriangle):
			{
				vec3f off = vec3f(offset, offset, offset);
				vec3f s0 = pos;
				vec3f s1 = dir;
				vec3f s2 = res.xyz();

				curLo = vec3f::sub(vec3f::minP(vec3f::minP(s0, s1), s2), off);
				curHi = vec3f::add(vec3f::maxP(vec3f::maxP(s0, s1), s2), off);
			}
			break;
		case(primQuadricPoint):
			{
				float w = dir.y + offset;
				vec3f w3 = vec3f(w, w, w);

				curLo = vec3f::sub(pos, w3);
				curHi = vec3f::add(pos, w3);
			}
			break;
		case(primInstance):
			{
				curLo = vec3f(0, 0, 0);
				curHi = vec3f(0, 0, 0);
			}
			break;

		}

		//Apply Transformation Matrix to BBox
		if(idxMtx != 0)
		{
			assert(idxMtx < (int)m_ctMtxBox);
			mat44f mat(&arrMtxBoxes[idxMtx * BOX_MATRIX_STRIDE]);

			//Set BBOX
			vec3f curLoT = mat.mapAffine(curLo);
			vec3f curHiT = mat.mapAffine(curHi);

			//Update Lo and Hi
			curLo = vec3f::minP(curLoT, curHiT);
			curHi = vec3f::maxP(curLoT, curHiT);

			//Set SEED
			seed = mat.mapAffine(seed);
		}

		//Set Primitive BBOX
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 0] = curLo.x;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 1] = curLo.y;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 2] = curLo.z;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 3] = 1.0f;

		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 4] = curHi.x;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 5] = curHi.y;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 6] = curHi.z;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 7] = 1.0f;

		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 8] = seed.x;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 9] = seed.y;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 10] = seed.z;
		arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + 11] = 1.0f;

		return AABB(curLo, curHi);
	}

	bool LinearBlobTree::computeOpsAABB(int idxRoot, AABB& aabb)
	{
		if(m_ctOps == 0)
			return false;
		if(!isOpIndex(idxRoot))
			return false;

		U32 idxLC, idxRC;
		bool bLeftChildOp, bRightChildOp, bRange, bUnary;

		int flags = arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_FLAGS];

		vec3f boxLo(FLT_MAX);
		vec3f boxHi(FLT_MIN);
		bUnary     = (( flags & ofIsUnaryOp) != 0);
		bRange 	   = (( flags & ofChildIndexIsRange) != 0);
		bLeftChildOp  = ((flags & ofLeftChildIsOp) != 0);
		bRightChildOp = ((flags & ofRightChildIsOp) != 0);
		idxLC = arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_LC];
		idxRC = arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_RC];

		//Range
		if(bRange)
		{
			vec3f primBoxLo;
			vec3f primBoxHi;


			for(U16 i=idxLC; i<=idxRC; i++)
			{
				primBoxLo = vec3f(&arrPrimBoxes[i * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_LO]);
				primBoxHi = vec3f(&arrPrimBoxes[i * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_HI]);

				if(i == idxLC)
				{
					boxLo = primBoxLo;
					boxHi = primBoxHi;
				}
				else
				{
					boxLo = vec3f::minP(boxLo, primBoxLo);
					boxHi = vec3f::maxP(boxHi, primBoxHi);
				}
			}
		}
		else
		{
			//Left Child Box
			vec3f boxLoLC;
			vec3f boxHiLC;
			if(bLeftChildOp) {
				AABB aabbLC;
				computeOpsAABB(idxLC, aabbLC);
				boxLoLC = aabbLC.lower();
				boxHiLC = aabbLC.upper();
			}
			else
			{
				boxLoLC = vec3f(&arrPrimBoxes[idxLC * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_LO]);
				boxHiLC = vec3f(&arrPrimBoxes[idxLC * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_HI]);
			}

			//Unary
			if(bUnary)
			{
				int idxMatrix = arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_RES_X];
				if(idxMatrix != 0)
				{
					assert(idxMatrix < (int)m_ctMtxBox);

					mat44f mat(&arrMtxBoxes[idxMatrix * BOX_MATRIX_STRIDE]);

					//Set BBOX
					vec3f curLoT = mat.mapAffine(boxLo);
					vec3f curHiT = mat.mapAffine(boxHi);

					boxLo = vec3f::minP(curLoT, curHiT);
					boxHi = vec3f::maxP(curLoT, curHiT);
				}
			}
			else
			{
				vec3f boxLoRC, boxHiRC;
				if(bRightChildOp) {
					AABB aabbRC;
					computeOpsAABB(idxRC, aabbRC);
					boxLoRC = aabbRC.lower();
					boxHiRC = aabbRC.upper();
				}
				else
				{
					boxLoRC = vec3f(&arrPrimBoxes[idxRC * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_LO]);
					boxHiRC = vec3f(&arrPrimBoxes[idxRC * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_HI]);
				}

				boxLo = vec3f::minP(boxLoLC, boxLoRC);
				boxHi = vec3f::maxP(boxHiLC, boxHiRC);
			}//bUnary
		}//bRange


		//Set Separate OP BOXES
		arrOpBoxes[idxRoot * DATASIZE_OPBOX + OFFSET_OPBOX_LO + 0] = boxLo.x;
		arrOpBoxes[idxRoot * DATASIZE_OPBOX + OFFSET_OPBOX_LO + 1] = boxLo.y;
		arrOpBoxes[idxRoot * DATASIZE_OPBOX + OFFSET_OPBOX_LO + 2] = boxLo.z;
		arrOpBoxes[idxRoot * DATASIZE_OPBOX + OFFSET_OPBOX_LO + 3] = 1.0f;

		arrOpBoxes[idxRoot * DATASIZE_OPBOX + OFFSET_OPBOX_HI + 0] = boxHi.x;
		arrOpBoxes[idxRoot * DATASIZE_OPBOX + OFFSET_OPBOX_HI + 1] = boxHi.y;
		arrOpBoxes[idxRoot * DATASIZE_OPBOX + OFFSET_OPBOX_HI + 2] = boxHi.z;
		arrOpBoxes[idxRoot * DATASIZE_OPBOX + OFFSET_OPBOX_HI + 3] = 1.0f;

		//Set Inside OP BOXES
		arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 0] = boxLo.x;
		arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 1] = boxLo.y;
		arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 2] = boxLo.z;
		arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_AABB_LO + 3] = 1.0f;

		arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 0] = boxHi.x;
		arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 1] = boxHi.y;
		arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 2] = boxHi.z;
		arrOps[idxRoot * DATASIZE_OPERATOR + OFFSET_OP_AABB_HI + 3] = 1.0f;

		aabb.set(boxLo, boxHi);

		return true;
	}


	int LinearBlobTree::computeAllInstancedNodesAABB()
	{
		if(m_ctPrims == 0)
			return 0;

		int count = 0;
		for(U32 i=0; i<m_ctPrims; i++)
		{
			PrimitiveType primType = static_cast<PrimitiveType>(arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE]);
			if(primType == primInstance)
			{
				vec3f res = vec3f(&arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_RES]);
				vec3f dir = vec3f(&arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_DIR]);

				//original node index
				U32 idxOrigin = static_cast<U32>(res.x);
				U32 isOriginOp = static_cast<U32>(res.z);
				U8 originNodeType = static_cast<U8>(dir.x);
				vec3f curLo, curHi;

				if(isOriginOp)
				{
					assert(static_cast<U8>(arrOps[idxOrigin * DATASIZE_OPERATOR + OFFSET_OP_TYPE])  == originNodeType);
					curLo = vec3f(&arrOpBoxes[idxOrigin * DATASIZE_OPBOX + OFFSET_OPBOX_LO]);
					curHi = vec3f(&arrOpBoxes[idxOrigin * DATASIZE_OPBOX + OFFSET_OPBOX_HI]);
				}
				else
				{
					assert(static_cast<U8>(arrPrims[idxOrigin * DATASIZE_PRIMITIVE + OFFSET_PRIM_TYPE]) == originNodeType);
					curLo = vec3f(&arrPrimBoxes[idxOrigin * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_LO]);
					curHi = vec3f(&arrPrimBoxes[idxOrigin * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_HI]);
				}

				//Apply Transformation Matrix to BBox
				int idxMatrix = arrPrims[i * DATASIZE_PRIMITIVE + OFFSET_PRIM_IDX_MATRIX];
				assert(idxMatrix < (int)m_ctMtxBox);

				mat44f mat(&arrMtxBoxes[idxMatrix* BOX_MATRIX_STRIDE]);

				//Set BBOX
				vec3f curLoT = mat.mapAffine(curLo);
				vec3f curHiT = mat.mapAffine(curHi);


				curLo = vec3f::minP(curLoT, curHiT);
				curHi = vec3f::maxP(curLoT, curHiT);
				vec3f seed = vec3f::mul(0.5f, vec3f::add(curLo, curHi));

				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_LO + 0] = curLo.x;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_LO + 1] = curLo.y;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_LO + 2] = curLo.z;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_LO + 3] = 1.0;

				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_HI + 0] = curHi.x;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_HI + 1] = curHi.y;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_HI + 2] = curHi.z;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_HI + 3] = 1.0;

				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_SEED + 0] = seed.x;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_SEED + 1] = seed.y;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_SEED + 2] = seed.z;
				arrPrimBoxes[i * BOX_MATRIX_STRIDE + OFFSET_PRIMBOX_SEED + 3] = 1.0;

				count++;
			}
		}

		//Return number of instanced nodes found
		return count;
	}

	AABB LinearBlobTree::getPrimAABB(int idxPrim) const {
		assert(isPrimIndex(idxPrim));

		vec3f lo = vec3f(&arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_LO]);
		vec3f hi = vec3f(&arrPrimBoxes[idxPrim * DATASIZE_PRIMBOX + OFFSET_PRIMBOX_HI]);
		return AABB(lo, hi);
	}

	AABB LinearBlobTree::getOpAABB(int idxOp) const {
		assert(isOpIndex(idxOp));

		vec3f lo = vec3f(&arrOpBoxes[idxOp * DATASIZE_OPBOX + OFFSET_OPBOX_LO]);
		vec3f hi = vec3f(&arrOpBoxes[idxOp * DATASIZE_OPBOX + OFFSET_OPBOX_HI]);
		return AABB(lo, hi);

	}


}
}
