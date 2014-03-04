/*
 * PS_ReadSceneMode.cpp
 *
 *  Created on: 2011-10-19
 *      Author: pourya
 */
#include "ReadSceneModel.h"
#include "base/String.h"
#include "base/Logger.h"
#include "base/Matrix.h"
#include <list>

using namespace PS;
using namespace PS::MATH;
using namespace PS::FILESTRINGUTILS;
using namespace std;

//Holds nodeinfo
struct NODEINFO{
	enum CHILDTREELOCATION{ctlLeft, ctlRight, ctlRange};
	int idxScript;
	CHILDTREELOCATION childTreeLocation;
	int idxParentSOA;
};

U8 ModelReader::GetScriptOpType(const char* chrOpName)
{
	if(strcmp(chrOpName, "FASTQUADRICPOINTSET") == 0)
		return opFastQuadricPointSet;
	else if(strcmp(chrOpName, "UNION") == 0)
		return opUnion;
	else if(strcmp(chrOpName, "BLEND") == 0)
		return opBlend;
	else if(strcmp(chrOpName, "RICCI BLEND") == 0)
		return opRicciBlend;
	else if(strcmp(chrOpName, "INTERSECTION") == 0)
		return opIntersect;
	else if(strcmp(chrOpName, "DIFFERENCE") == 0)
		return opDif;
	else if(strcmp(chrOpName, "SMOOTH DIFFERENCE") == 0)
		return opSmoothDif;
	else if(strcmp(chrOpName, "CACHE") == 0)
		return opCache;
	else if(strcmp(chrOpName, "TWIST") == 0)
		return opWarpTwist;
	else if(strcmp(chrOpName, "TAPER") == 0)
		return opWarpTaper;
	else if(strcmp(chrOpName, "BEND") == 0)
		return opWarpBend;
	else if(strcmp(chrOpName, "SHEAR") == 0)
		return opWarpShear;
	else
		return opUnion;
}

AnsiStr ModelReader::GetScriptOpName(U8 opCode)
{
	AnsiStr strName = "UNKNOWN";
	switch(opCode)
	{
	case(opFastQuadricPointSet): {
		strName = "FASTQUADRICPOINTSET";
		break;
	}
	case(opUnion): {
		strName = "UNION";
		break;
	}

	case(opBlend): {
		strName = "BLEND";
		break;
	}
	case(opRicciBlend): {
		strName = "RICCI BLEND";
		break;
	}
	case(opIntersect): {
		strName = "INTERSECTION";
		break;
	}
	case(opDif): {
		strName = "DIFFERENCE";
		break;
	}
	case(opSmoothDif): {
		strName = "SMOOTH DIFFERENCE";
		break;
	}

	case(opCache): {
		strName = "CACHE";
		break;
	}
	case(opWarpTwist): {
		strName = "TWIST";
		break;
	}
	case(opWarpTaper): {
		strName = "TAPER";
		break;
	}
	case(opWarpBend): {
		strName = "BEND";
		break;
	}
	case(opWarpShear): {
		strName = "SHEAR";
		break;
	}
	}

	return strName;
}

U8 ModelReader::GetScriptPrimType(const char* chrPrimName)
{
	if(strcmp(chrPrimName, "QUADRICPOINT") == 0)
		return primQuadricPoint;
	else if(strcmp(chrPrimName, "POINT") == 0)
		return primPoint;
	else if(strcmp(chrPrimName, "LINE") == 0)
		return primLine;
	else if(strcmp(chrPrimName, "CYLINDER") == 0)
		return primCylinder;
	else if(strcmp(chrPrimName, "DISC") == 0)
		return primDisc;
	else if(strcmp(chrPrimName, "RING") == 0)
		return primRing;
	else if(strcmp(chrPrimName, "CUBE") == 0)
		return primCube;
	else if(strcmp(chrPrimName, "TRIANGLE") == 0)
		return primTriangle;
	else if(strcmp(chrPrimName, "INSTANCE") == 0)
		return primInstance;
	else if(strcmp(chrPrimName, "NULL") == 0)
		return primNULL;
	else
		return primNULL;
}

AnsiStr ModelReader::GetScriptPrimName(U8 primCode)
{
	AnsiStr strName = "UNKNOWN";
	switch(primCode)
	{
	case(primQuadricPoint): {
		strName = "QUADRICPOINT";
		break;
	}
	case(primPoint): {
		strName = "POINT";
		break;
	}
	case(primLine): {
		strName = "LINE";
		break;
	}
	case(primCylinder): {
		strName = "CYLINDER";
		break;
	}
	case(primDisc): {
		strName = "DISC";
		break;
	}
	case(primRing): {
		strName = "RING";
		break;
	}

	case(primCube): {
		strName = "CUBE";
		break;
	}
	case(primTriangle): {
		strName = "TRIANGLE";
		break;
	}
	case(primInstance): {
		strName = "INSTANCE";
		break;
	}
	case(primNULL): {
		strName = "NULL";
		break;
	}
	}

	return strName;
}

ModelReader::ModelReader(SOABlobPrims* lpPrims,
						 SOABlobOps* lpOps,
						 SOABlobNodeMatrices* lpMtxNodes,
						 SOABlobBoxMatrices* lpMtxBox):m_lpBlobPrims(lpPrims), m_lpBlobOps(lpOps),
						 m_lpMtxNode(lpMtxNodes), m_lpMtxBox(lpMtxBox)
{
}

ModelReader::ModelReader(SimdPoly& simdPoly)
{
	m_lpBlobPrims = simdPoly.getBlobPrims();
	m_lpBlobOps = simdPoly.getBlobOps();
	m_lpMtxNode = simdPoly.getMtxNode();
	m_lpMtxBox = simdPoly.getMtxBox();
}

ModelReader::~ModelReader()
{
	m_lstReadNodes.resize(0);
}

int ModelReader::setAllInstancedNodes()
{
	int ctFixed = 0;
	for(U32 i=0;i < m_lpBlobPrims->count; i++)
	{
		if(m_lpBlobPrims->type[i] == primInstance)
		{
			U32 idxScript = m_lpBlobPrims->resY[i];
			for(U32 j=0; j < m_lstReadNodes.size(); j++)
			{
				if(m_lstReadNodes[j].idxScript == idxScript)
				{
					m_lpBlobPrims->resX[i] = m_lstReadNodes[j].idxArray;
					ctFixed++;
					break;
				}
			}

		}
	}

	return ctFixed;
}

int ModelReader::read(const char* lpFilePath)
{
	if(!FileExists(AnsiStr(lpFilePath)))
		return ERR_INVALID_MODEL_FILE;

	//Reset Internal Data Structures;
	m_ctInstancedNodes = 0;
	m_lstReadNodes.resize(0);

	//Reset BlobTree
	m_lpBlobPrims->count = 0;
	m_lpBlobOps->count = 0;

	//Set identity matrix
	float identity[] = {1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f};
	m_lpMtxNode->count = 1;
	m_lpMtxBox->count = 1;
	for(U32 i=0; i<PRIM_MATRIX_STRIDE; i++)
		m_lpMtxNode->matrix[i] = identity[i];
	for(U32 i=0; i<BOX_MATRIX_STRIDE; i++)
		m_lpMtxBox->matrix[i] = identity[i];


	//FileVersion
	SettingsScript* lpScript = new SettingsScript(AnsiStr(lpFilePath), SettingsScript::fmRead);
	int nVersion = lpScript->readInt("Global", "FileVersion", 0);
	if(nVersion < MIN_FILE_VERSION)
	{
		LogErrorArg1("Invalid file version. Minimum file version is %d.", MIN_FILE_VERSION);
		SAFE_DELETE(lpScript);
		return ERR_INVALID_MODEL_FILE;
	}

	int ctLayers = lpScript->readInt("Global", "NumLayers", 0);
	if(ctLayers == 0 || ctLayers > 1)
	{
		LogError("Invalid scene file. There should be at least 1 layer.");
		SAFE_DELETE(lpScript);
		return ERR_INVALID_LAYERS_COUNT;
	}

	int ctPrims = lpScript->readInt("Global", "CountPrimitives", 0);
	int ctOps = lpScript->readInt("Global", "CountOperators", 0);
	if((ctPrims > MAX_TREE_NODES)||(ctOps > MAX_TREE_NODES))
	{
		LogErrorArg3("MAX TREE NODES set at %d. Model has %d primitive %d operators.", MAX_TREE_NODES, ctPrims, ctOps);
		SAFE_DELETE(lpScript);
		return ERR_NODE_OVERFLOW;
	}

	std::vector<int> ids;
	ids.reserve(4);
	lpScript->readIntArray("Global", "RootIDs", 1, ids);
	if(ids.size() == 0)
	{
		SAFE_DELETE(lpScript);
		return ERR_INVALID_ROOT_ID;
	}

	//start by reading root
	int ctErrors = 0;

	//Recursively read all nodes
	this->readNode(lpScript, ids[0]);
	if(setAllInstancedNodes() == m_ctInstancedNodes)
	{
		LogInfoArg1("MODELREADER: %d instanced nodes are updated.", m_ctInstancedNodes);
	}

	int total = m_lpBlobOps->count + m_lpBlobPrims->count;
	if((ctPrims + ctOps > 0)&&(total != (ctPrims + ctOps)))
	{
		LogErrorArg2("Not read all nodes [%d of %d].", total, ctPrims + ctOps);
		ctErrors++;
	}

	SAFE_DELETE(lpScript);

	//Prepare Boxes
	int res = PrepareAllBoxes(*m_lpBlobOps, *m_lpBlobPrims, *m_lpMtxBox);
	if(res != RET_SUCCESS)
	{
		LogError("Unable to compute bboxes.");
		return ERR_READ_MODEL_HAS_PROBLEMS;
	}

	ctErrors += CheckForBlobTreeErrors(*m_lpBlobOps, *m_lpBlobPrims, true);
	if(ctErrors > 0)
	{
		LogErrorArg1("%d errors found in the generated SOABlobTree.", ctErrors);
		return ERR_READ_MODEL_HAS_PROBLEMS;
	}
	else
	{
		LogInfoArg1("MODELREADER: Model read successfully. FileVersion = %d", nVersion);
		return MODELREAD_SUCCESS;
	}
}

//Recursive read
int ModelReader::readNode(SettingsScript* lpScript, int id, int* lpOutIsOp)
{
	AnsiStr strNodeName = printToAStr("BLOBNODE %d", id);

	bool isOp = lpScript->readBool(strNodeName, "IsOperator");
	if(lpOutIsOp)
		*lpOutIsOp = isOp;

	int idxNode = -1;
	if(isOp)
	{
		//Offset in SOA
		idxNode = m_lpBlobOps->count;

		//Left Child and Right Child are indices to BlobTree Nodes and Ops
		m_lpBlobOps->count++;

		//Add Op
		AnsiStr strOpType = lpScript->readString(strNodeName, "OperatorType");
		m_lpBlobOps->type[idxNode]   = GetScriptOpType(strOpType.ptr());
		m_lpBlobOps->opFlags[idxNode]  = 0;


		switch(m_lpBlobOps->type[idxNode])
		{
		case(opRicciBlend):
		{
			float power = lpScript->readFloat(strNodeName, "power", 1.0f);
			m_lpBlobOps->resX[idxNode] = power;
			m_lpBlobOps->resY[idxNode] = 1.0f / power;
		}
		break;
		case(opWarpTwist):
		{
			float factor = lpScript->readFloat(strNodeName, "factor", 1.0f);
			float axis = static_cast<float>(lpScript->readInt(strNodeName, "axis", 2));

			m_lpBlobOps->resX[idxNode] = factor;
			m_lpBlobOps->resY[idxNode] = axis;
			m_lpBlobOps->opFlags[idxNode]  |= ofIsUnaryOp;
		}
		break;

		case(opWarpTaper):
		{
			float factor = lpScript->readFloat(strNodeName, "factor", 1.0f);
			float axisBase = static_cast<float>(lpScript->readInt(strNodeName, "base axis", 0));
			float axisTaper = static_cast<float>(lpScript->readInt(strNodeName, "taper axis", 2));
			m_lpBlobOps->resX[idxNode] = factor;
			m_lpBlobOps->resY[idxNode] = axisBase;
			m_lpBlobOps->resZ[idxNode] = axisTaper;
			m_lpBlobOps->opFlags[idxNode]  |= ofIsUnaryOp;
		}
		break;

		case(opWarpBend):
		{
			float rate	 = lpScript->readFloat(strNodeName, "rate", 1.0f);
			float center = lpScript->readFloat(strNodeName, "center", 0.5f);
			float lbound = lpScript->readFloat(strNodeName, "left bound", 0.0f);
			float rbound = lpScript->readFloat(strNodeName, "right bound", 1.0f);

			m_lpBlobOps->resX[idxNode] = rate;
			m_lpBlobOps->resY[idxNode] = center;
			m_lpBlobOps->resZ[idxNode] = lbound;
			m_lpBlobOps->resW[idxNode] = rbound;
			m_lpBlobOps->opFlags[idxNode]  |= ofIsUnaryOp;
		}
		break;
		case(opWarpShear):
		{
			float factor = lpScript->readFloat(strNodeName, "factor", 1.0f);
			float axisBase  = static_cast<float>(lpScript->readInt(strNodeName, "base axis", 0));
			float axisShear = static_cast<float>(lpScript->readInt(strNodeName, "shear axis", 2));

			m_lpBlobOps->resX[idxNode] = factor;
			m_lpBlobOps->resY[idxNode] = axisBase;
			m_lpBlobOps->resZ[idxNode] = axisShear;
			m_lpBlobOps->opFlags[idxNode]  |= ofIsUnaryOp;
		}
		break;
		}

		//Read Child nodes
		{
			//Read Children
			vector<int> ids;
			int ctExpectedChildren = lpScript->readInt(strNodeName, "ChildrenCount");
			bool bUseRange = lpScript->readBool(strNodeName, "ChildrenIDsUseRange", false);
			if(bUseRange)
			{
				ids.resize(0);
				lpScript->readIntArray(strNodeName, "ChildrenIDsRange", 2, ids);
				if(ids.size() != 2)
				{
					printf("Error: Processing SOABlobOp %d, Range is not defined [%d].\n", idxNode, (int)ids.size());
					SAFE_DELETE(lpScript);
					return false;
				}

				int idxStart = ids[0];
				int idxEnd = ids[1];
				ids.resize(0);
				for(int i=idxStart; i<=idxEnd; i++)
				{
					ids.push_back(readNode(lpScript, i));
				}

				//Child Kind is range
				m_lpBlobOps->opLeftChild[idxNode]  = ids[0];
				m_lpBlobOps->opRightChild[idxNode] = ids[ids.size() - 1];
				m_lpBlobOps->opFlags[idxNode]  |= ofChildIndexIsRange;
			}
			else
			{
				ids.resize(0);
				lpScript->readIntArray(strNodeName, "ChildrenIDs", ctExpectedChildren, ids);
				bool isBinaryNode = ((m_lpBlobOps->opFlags[idxNode] & ofIsUnaryOp) == 0);
				if(isBinaryNode && (ids.size() != 2))
				{
					printf("Error: Processing Binary node: SOABlobOp %d, Found %d children.\n", idxNode, (int)ids.size());
					SAFE_DELETE(lpScript);
					return false;
				}

				int isLCOp = 0;
				int isRCOp = 0;
				int idxLC;
				int idxRC;

				//Read Left Child
				idxLC = readNode(lpScript, ids[0], &isLCOp);
				m_lpBlobOps->opLeftChild[idxNode] = idxLC;
				if(isLCOp)
					m_lpBlobOps->opFlags[idxNode] |= ofLeftChildIsOp;

				if(isBinaryNode)
				{
					idxRC = readNode(lpScript, ids[1], &isRCOp);
					m_lpBlobOps->opRightChild[idxNode] = idxRC;
					if(isRCOp)
					{
						m_lpBlobOps->opFlags[idxNode] |= ofRightChildIsOp;

						//Is Right Op
						m_lpBlobOps->opFlags[idxRC] |= ofIsRightOp;
					}

					//Set Break Flag for both child ops
					if(isLCOp && isRCOp)
					{
						m_lpBlobOps->opFlags[idxLC] |= ofBreak;
						m_lpBlobOps->opFlags[idxRC] |= ofBreak;
					}

				}
			}
		}
	}
	else
	{
		//Increment
		idxNode = m_lpBlobPrims->count;
		m_lpBlobPrims->count++;

		//Set SkeletonType
		AnsiStr strPrimType = lpScript->readString(strNodeName, "PrimitiveType");
		m_lpBlobPrims->type[idxNode] = GetScriptPrimType(strPrimType.ptr());

		//Set Color
		vec4f color = lpScript->readVec4f(strNodeName, "MtrlDiffused");
		m_lpBlobPrims->colorX[idxNode] 	= color.x;
		m_lpBlobPrims->colorY[idxNode] 	= color.y;
		m_lpBlobPrims->colorZ[idxNode] 	= color.z;

		//Set other params
		switch(m_lpBlobPrims->type[idxNode])
		{
		case(primQuadricPoint):
		{
			vec3f pos   = lpScript->readVec3f(strNodeName, "position");
			float scale  = lpScript->readFloat(strNodeName, "scale");
			float radius = lpScript->readFloat(strNodeName, "radius");
			m_lpBlobPrims->posX[idxNode] = pos.x;
			m_lpBlobPrims->posY[idxNode] = pos.y;
			m_lpBlobPrims->posZ[idxNode] = pos.z;

			m_lpBlobPrims->dirX[idxNode] = scale;
			m_lpBlobPrims->dirY[idxNode] = radius;
			m_lpBlobPrims->dirZ[idxNode] = radius * radius;

			//Coeffs
			m_lpBlobPrims->resX[idxNode] = scale / (radius*radius*radius*radius);
			m_lpBlobPrims->resY[idxNode] = (-2.0f * scale) / (radius * radius);
			m_lpBlobPrims->resZ[idxNode] = scale;
		}
		break;
		case(primPoint):
		{
			vec3f pos = lpScript->readVec3f(strNodeName, "position");
			m_lpBlobPrims->posX[idxNode] = pos.x;
			m_lpBlobPrims->posY[idxNode] = pos.y;
			m_lpBlobPrims->posZ[idxNode] = pos.z;
		}
		break;
		case(primLine):
		{
			vec3f s = lpScript->readVec3f(strNodeName, "start");
			vec3f e = lpScript->readVec3f(strNodeName, "end");
			m_lpBlobPrims->posX[idxNode] = s.x;
			m_lpBlobPrims->posY[idxNode] = s.y;
			m_lpBlobPrims->posZ[idxNode] = s.z;

			m_lpBlobPrims->dirX[idxNode] = e.x;
			m_lpBlobPrims->dirY[idxNode] = e.y;
			m_lpBlobPrims->dirZ[idxNode] = e.z;
		}
		break;
		case(primRing):
		{
			vec3f c = lpScript->readVec3f(strNodeName, "position");
			vec3f d = lpScript->readVec3f(strNodeName, "direction");
			float r = lpScript->readFloat(strNodeName, "radius");
			m_lpBlobPrims->posX[idxNode] = c.x;
			m_lpBlobPrims->posY[idxNode] = c.y;
			m_lpBlobPrims->posZ[idxNode] = c.z;

			m_lpBlobPrims->dirX[idxNode] = d.x;
			m_lpBlobPrims->dirY[idxNode] = d.y;
			m_lpBlobPrims->dirZ[idxNode] = d.z;

			m_lpBlobPrims->resX[idxNode] = r;
		}
		break;
		case(primDisc):
		{
			vec3f c = lpScript->readVec3f(strNodeName, "position");
			vec3f d = lpScript->readVec3f(strNodeName, "direction");
			float r = lpScript->readFloat(strNodeName, "radius");

			m_lpBlobPrims->posX[idxNode] = c.x;
			m_lpBlobPrims->posY[idxNode] = c.y;
			m_lpBlobPrims->posZ[idxNode] = c.z;

			m_lpBlobPrims->dirX[idxNode] = d.x;
			m_lpBlobPrims->dirY[idxNode] = d.y;
			m_lpBlobPrims->dirZ[idxNode] = d.z;

			m_lpBlobPrims->resX[idxNode] = r;

		}
		break;
		case(primCylinder):
		{
			vec3f p = lpScript->readVec3f(strNodeName, "position");
			vec3f d = lpScript->readVec3f(strNodeName, "direction");
			float r = lpScript->readFloat(strNodeName, "radius");
			float h = lpScript->readFloat(strNodeName, "height");

			m_lpBlobPrims->posX[idxNode] = p.x;
			m_lpBlobPrims->posY[idxNode] = p.y;
			m_lpBlobPrims->posZ[idxNode] = p.z;

			m_lpBlobPrims->dirX[idxNode] = d.x;
			m_lpBlobPrims->dirY[idxNode] = d.y;
			m_lpBlobPrims->dirZ[idxNode] = d.z;

			m_lpBlobPrims->resX[idxNode] = r;
			m_lpBlobPrims->resY[idxNode] = h;
		}
		break;
		case(primCube):
		{
			vec3f p = lpScript->readVec3f(strNodeName, "position");
			float s = lpScript->readFloat(strNodeName, "side");

			m_lpBlobPrims->posX[idxNode] = p.x;
			m_lpBlobPrims->posY[idxNode] = p.y;
			m_lpBlobPrims->posZ[idxNode] = p.z;

			m_lpBlobPrims->resX[idxNode] = s;
		}
		break;
		case(primTriangle):
		{
			vec3f a = lpScript->readVec3f(strNodeName, "corner0");
			vec3f b = lpScript->readVec3f(strNodeName, "corner1");
			vec3f c = lpScript->readVec3f(strNodeName, "corner2");
			m_lpBlobPrims->posX[idxNode] = a.x;
			m_lpBlobPrims->posY[idxNode] = a.y;
			m_lpBlobPrims->posZ[idxNode] = a.z;

			m_lpBlobPrims->dirX[idxNode] = b.x;
			m_lpBlobPrims->dirY[idxNode] = b.y;
			m_lpBlobPrims->dirZ[idxNode] = b.z;

			m_lpBlobPrims->resX[idxNode] = c.x;
			m_lpBlobPrims->resY[idxNode] = c.y;
			m_lpBlobPrims->resZ[idxNode] = c.z;
		}
		break;
		case(primNULL):
		{
			m_lpBlobPrims->posX[idxNode] = 0.0f;
			m_lpBlobPrims->posY[idxNode] = 0.0f;
			m_lpBlobPrims->posZ[idxNode] = 0.0f;
		}
		break;
		case(primInstance):
		{
			//x: idxArray
			//y: idxScript
			//z: isOperator
			//w: NodeType
			int idxScript = lpScript->readInt(strNodeName, "OriginalNodeIndex");
			int isOriginOp = lpScript->readInt(strNodeName, "OriginalNodeIsOp");
			int idxOriginNodeType = 0;
			if(isOriginOp)
				idxOriginNodeType = GetScriptOpType(lpScript->readString(strNodeName, "OriginalNodeType").cptr());
			else
				idxOriginNodeType = GetScriptPrimType(lpScript->readString(strNodeName, "OriginalNodeType").cptr());
			m_lpBlobPrims->resX[idxNode] = 0;
			m_lpBlobPrims->resY[idxNode] = idxScript;
			m_lpBlobPrims->resZ[idxNode] = isOriginOp;

			//Origin NodeType
			m_lpBlobPrims->dirX[idxNode] = idxOriginNodeType;
			m_ctInstancedNodes++;
		}
		break;
		default:
		{
			printf("[BlobTree Script] Unknown primitive type: [%s]", strPrimType.ptr());
			SAFE_DELETE(lpScript);
			return ERR_UNKNOWN_PRIM;
		}
		}

		m_lpBlobPrims->idxMatrix[idxNode] = this->readTransformation(lpScript, id);
	}

	//Save the index script and array index pair
	READNODES rd;
	rd.idxArray = idxNode;
	rd.idxScript = id;
	m_lstReadNodes.push_back(rd);

	//Return idxNode
	return idxNode;
}

//Return the index of the transformation
int ModelReader::readTransformation(PS::SettingsScript* lpScript, int id)
{
	AnsiStr strNodeName = printToAStr("BLOBNODE %d", id);

	//Matrix Computation
	vec3f s = lpScript->readVec3f(strNodeName, "AffineScale");
	vec4f r = lpScript->readVec4f(strNodeName, "AffineRotate");
	vec3f t = lpScript->readVec3f(strNodeName, "AffineTranslate");

	mat44f matForward, matBackward;
	matForward.identity();
	matBackward.identity();

	//Compute Forward and Backward Transformation as it is done with Affine
	//The correct order is TRSp = (T(R(Sp)))
	//Set Translate
	matForward.translate(t);

	//Rotation
	matForward.rotate(quat(r));

	//Scale
	matForward.scale(s);


	//Compute Backward Matrix
	matBackward = matForward.inverted();

	if(matForward.isIdentity())
		return 0;
	else
	{
		//Make sure we have enough memory for Matrices
		assert(m_lpMtxNode->count < MAX_MTX_NODES);

		//Copy Backward Matrix for Primitive Distance Computation
		int idxMatrix = m_lpMtxNode->count;
		int idxMtxElement = idxMatrix * PRIM_MATRIX_STRIDE;
		for(int row = 0; row < 3; row++) {
			for(int col = 0; col < 4; col++) {
				m_lpMtxNode->matrix[idxMtxElement + row*4 + col] = matBackward.e[col][row];
			}
		}
		m_lpMtxNode->count++;

		//Copy Forward Matrix for Box Manipulation
		idxMatrix = m_lpMtxBox->count;
		idxMtxElement = idxMatrix * BOX_MATRIX_STRIDE;
		for(int row = 0; row < 4; row++) {
			for(int col = 0; col < 4; col++) {
				m_lpMtxBox->matrix[idxMtxElement + row*4 + col] = matForward.e[col][row];
			}
		}
		m_lpMtxBox->count++;

		return idxMatrix;
	}
}


/*!
 * Exports polygonizer output as an obj file
 */
bool ExportMeshAsObj(const char* lpFilePath, const PolyMPUs& poly)
{
	std::vector<float> arrVertices;
	std::vector<float> arrNormals;
	std::vector<U32> arrTriangles;

	U32 ctVertices = 0;
	U32 ctTriangles = 0;
	//Count all vertices and triangles
	for(U32 i=0; i<poly.countWorkUnits(); i++)
	{
		ctTriangles += poly.lpMPUs[i].ctTriangles;
		ctVertices += poly.lpMPUs[i].ctVertices;
	}

	arrVertices.resize(ctVertices*3);
	arrNormals.resize(ctVertices*3);
	arrTriangles.resize(ctTriangles*3);

	ctTriangles = 0;
	ctVertices = 0;

	for(U32 i=0; i<poly.countWorkUnits(); i++)
	{
		//ctTriangles += poly.lpMPUs[i].ctTriangles;
		if(poly.lpMPUs[i].ctTriangles > 0)
		{
			U32 triOffset = ctVertices;
			for(U32 iTri = 0; iTri < poly.lpMPUs[i].ctTriangles; iTri++)
			{
				U32 idxMeshTriPart = i * MPU_MESHPART_TRIANGLE_STRIDE + iTri * 3;
				arrTriangles[ctTriangles*3] 	= poly.globalMesh.vTriangles[idxMeshTriPart] + triOffset;
				arrTriangles[ctTriangles*3 + 1] = poly.globalMesh.vTriangles[idxMeshTriPart + 1] + triOffset;
				arrTriangles[ctTriangles*3 + 2] = poly.globalMesh.vTriangles[idxMeshTriPart + 2] + triOffset;
				ctTriangles++;
			}


			for(U32 iVertex = 0; iVertex < poly.lpMPUs[i].ctVertices; iVertex++)
			{
				U32 idxMeshVtxPart = i * MPU_MESHPART_VERTEX_STRIDE + iVertex * 3;
				arrVertices[ctVertices*3] = poly.globalMesh.vPos[idxMeshVtxPart];
				arrVertices[ctVertices*3 + 1] = poly.globalMesh.vPos[idxMeshVtxPart + 1];
				arrVertices[ctVertices*3 + 2] = poly.globalMesh.vPos[idxMeshVtxPart + 2];

				arrNormals[ctVertices*3] = poly.globalMesh.vNorm[idxMeshVtxPart];
				arrNormals[ctVertices*3 + 1] = poly.globalMesh.vNorm[idxMeshVtxPart + 1];
				arrNormals[ctVertices*3 + 2] = poly.globalMesh.vNorm[idxMeshVtxPart + 2];
				ctVertices++;
			}
		}

	}

	// find the file
	std::ofstream ofs( lpFilePath );

	AnsiStr strOut = AnsiStr("#Exported from ParsipCmd\n");
	ofs << strOut;
	strOut = AnsiStr("o cube1\n");
	ofs << strOut;

	strOut = printToAStr("#%d vertices, %d faces\n", ctVertices, ctTriangles);
	ofs << strOut;

	vec3f v;
	for(U32 i=0; i<ctVertices; i++)
	{
		v = vec3f(arrVertices[i * 3], arrVertices[i * 3 + 1], arrVertices[i * 3 + 2]);
		strOut = printToAStr("v %f %f %f\n",  v.x, v.y, v.z);
		ofs << strOut;
	}

	vec3f n;
	for(U32 i=0; i<ctVertices; i++)
	{
		n = vec3f(arrNormals[i * 3], arrNormals[i * 3 + 1], arrNormals[i * 3 + 2]);
		strOut = printToAStr("vn %f %f %f\n",  n.x, n.y, n.z);
		ofs << strOut;
	}


	strOut = printToAStr("g cube1_default\n");
	ofs << strOut;

	strOut = printToAStr("usemtl default\n");
	ofs << strOut;

	U32 t[3];
	for(U32 i=0; i<ctTriangles; i++)
	{
		t[0] = arrTriangles[i * 3] + 1;
		t[1] = arrTriangles[i * 3 + 1] + 1;
		t[2] = arrTriangles[i * 3 + 2] + 1;
		strOut = printToAStr("f %d//%d %d//%d %d//%d\n",  t[0], t[0], t[1], t[1], t[2], t[2]);
		ofs << strOut;
	}

	ofs.close();
	return (ctTriangles > 0);
}


