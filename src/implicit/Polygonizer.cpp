#include <assert.h>
#include "Polygonizer.h"
#include "_CellConfigTable.h"
#include "base/Matrix.h"
#include "base/SIMDVecN.h"

using namespace PS::MATH;
//#include "PS_KDTree.h"
//#include "stdio.h"

namespace PS{
namespace SIMDPOLY{


typedef tbb::enumerable_thread_specific< std::pair<int, int> > CounterTotalCrossedMPU;
typedef tbb::enumerable_thread_specific< FieldComputer > ThreadFieldComputer;

CounterTotalCrossedMPU g_ctTotalCrossed(std::make_pair(0, 0));
ThreadFieldComputer g_threadFieldComputer;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
ThreadStartSetup::ThreadStartSetup(const SOABlobOps* lpOps,
								   const SOABlobPrims* lpPrims,
		  	  	 	 	 	 	   const SOABlobNodeMatrices* lpMatrices)
{
	m_lpBlobPrims = const_cast<SOABlobPrims*>(lpPrims);
	m_lpBlobMatrices = const_cast<SOABlobNodeMatrices*>(lpMatrices);
	m_lpBlobOps = const_cast<SOABlobOps*>(lpOps);
	observe(true);
}


void ThreadStartSetup::on_scheduler_entry(bool is_worker)
{
	ThreadFieldComputer::reference m_localFieldComputer = g_threadFieldComputer.local();


	//Create this copy for the thread
	memcpy(&m_localFieldComputer.m_blobOps, m_lpBlobOps, sizeof(SOABlobOps));
	memcpy(&m_localFieldComputer.m_blobPrims, m_lpBlobPrims, sizeof(SOABlobPrims));
	memcpy(&m_localFieldComputer.m_blobPrimMatrices, m_lpBlobMatrices, sizeof(SOABlobNodeMatrices));
//	m_localFieldComputer.setup(m_lpBlobPrims, m_lpBlobMatrices, m_lpBlobOps);

	//__builtin_prefetch(&m_localFieldComputer.m_blobPrims);
	//__builtin_prefetch(&m_localFieldComputer.m_blobPrimMatrices);
	//__builtin_prefetch(&m_localFieldComputer.m_blobOps);
	PS_PREFETCH(&m_localFieldComputer.m_blobOps);
	PS_PREFETCH(&m_localFieldComputer.m_blobPrims);
	PS_PREFETCH(&m_localFieldComputer.m_blobPrimMatrices);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Returns the count of needed MPUs
vec3i CountMPUNeeded(float cellsize, const vec3f& lo, const vec3f& hi)
{
	const int cellsPerMPU = GRID_DIM - 1;

	vec3f allSides = vec3f::sub(hi, lo);
	vec3i ctCellsNeeded;
	vec3i ctMPUNeeded;

	ctCellsNeeded.x = static_cast<int>(ceil(allSides.x / cellsize));
	ctCellsNeeded.y = static_cast<int>(ceil(allSides.y / cellsize));
	ctCellsNeeded.z = static_cast<int>(ceil(allSides.z / cellsize));

	ctMPUNeeded.x = ctCellsNeeded.x / cellsPerMPU;
	ctMPUNeeded.y = ctCellsNeeded.y / cellsPerMPU;
	ctMPUNeeded.z = ctCellsNeeded.z / cellsPerMPU;
	if (ctCellsNeeded.x % cellsPerMPU != 0)
		ctMPUNeeded.x++;
	if (ctCellsNeeded.y % cellsPerMPU != 0)
		ctMPUNeeded.y++;
	if (ctCellsNeeded.z % cellsPerMPU != 0)
		ctMPUNeeded.z++;

	return ctMPUNeeded;
}

bool IsBoxInside(const vec3f& aLo, const vec3f& aHi, const vec3f& bLo, const vec3f& bHi)
{
	if((aLo.x < bLo.x)||(aLo.y < bLo.y)||(aLo.z < bLo.z))
		return false;
	if((aHi.x > bHi.x)||(aHi.y > bHi.y)||(aHi.z > bHi.z))
		return false;
	return true;
}

//Check Blobtree Errors
int CheckForBlobTreeErrors(SOABlobOps& blobOps, SOABlobPrims& blobPrims, bool bCheckBoxes)
{
	//Check each operator referenced once
	int ctErrors = 0;
	for(U32 i=0; i < blobOps.count; i++)
	{
		int ctUsed = 0;
		for(U32 j=0; j < blobOps.count; j++)
		{
			if(i != j)
			{
				if((blobOps.opLeftChild[j] == i)&&((blobOps.opFlags[j] & 2) != 0))
					ctUsed++;

				if((blobOps.opRightChild[j] == i)&&((blobOps.opFlags[j] & 1) != 0))
					ctUsed++;
			}
		}

		if(ctUsed > 1)
			ctErrors++;
	}

	//
	if(bCheckBoxes)
	{
		//Check if bounding box of a primitive has error
		for(U32 i=0; i < blobPrims.count; i++)
		{
			vec3f boxLo;
			vec3f boxHi;

			boxLo.x = blobPrims.bboxLoX[i];
			boxLo.y = blobPrims.bboxLoY[i];
			boxLo.z = blobPrims.bboxLoZ[i];

			boxHi.x = blobPrims.bboxHiX[i];
			boxHi.y = blobPrims.bboxHiY[i];
			boxHi.z = blobPrims.bboxHiZ[i];

			if((boxHi.x < boxLo.x)||(boxHi.y < boxLo.y)||(boxHi.z < boxLo.z))
			{
				printf("BLOBTREE ERROR: Bounding Box Error at Prim %d.\n", i);
				ctErrors++;
			}
		}

		//Check if BBox of an operator has error
		for(U32 i=0; i < blobOps.count; i++)
		{
			vec3f boxLo;
			vec3f boxHi;

			boxLo.x = blobOps.bboxLoX[i];
			boxLo.y = blobOps.bboxLoY[i];
			boxLo.z = blobOps.bboxLoZ[i];

			boxHi.x = blobOps.bboxHiX[i];
			boxHi.y = blobOps.bboxHiY[i];
			boxHi.z = blobOps.bboxHiZ[i];

			bool bRange = (blobOps.opFlags[i] & 4) >> 2;
			bool bLeftChildIsOp = (blobOps.opFlags[i] & 2) >> 1;
			bool bRightChildIsOp = (blobOps.opFlags[i] & 1);

			if(!bRange)
			{
				U32 idxLC = blobOps.opLeftChild[i];
				U32 idxRC = blobOps.opRightChild[i];
				vec3f child1BoxLo;
				vec3f child1BoxHi;
				vec3f child2BoxLo;
				vec3f child2BoxHi;

				if(bLeftChildIsOp)
				{
					child1BoxLo = vec3f(blobOps.bboxLoX[idxLC], blobOps.bboxLoY[idxLC], blobOps.bboxLoZ[idxLC]);
					child1BoxHi = vec3f(blobOps.bboxHiX[idxLC], blobOps.bboxHiY[idxLC], blobOps.bboxHiZ[idxLC]);
				}
				else
				{
					child1BoxLo = vec3f(blobPrims.bboxLoX[idxRC], blobPrims.bboxLoY[idxRC], blobPrims.bboxLoZ[idxRC]);
					child1BoxHi = vec3f(blobPrims.bboxHiX[idxRC], blobPrims.bboxHiY[idxRC], blobPrims.bboxHiZ[idxRC]);
				}

				if(bRightChildIsOp)
				{
					child2BoxLo = vec3f(blobOps.bboxLoX[idxLC], blobOps.bboxLoY[idxLC], blobOps.bboxLoZ[idxLC]);
					child2BoxHi = vec3f(blobOps.bboxHiX[idxLC], blobOps.bboxHiY[idxLC], blobOps.bboxHiZ[idxLC]);
				}
				else
				{
					child2BoxLo = vec3f(blobPrims.bboxLoX[idxRC], blobPrims.bboxLoY[idxRC], blobPrims.bboxLoZ[idxRC]);
					child2BoxHi = vec3f(blobPrims.bboxHiX[idxRC], blobPrims.bboxHiY[idxRC], blobPrims.bboxHiZ[idxRC]);
				}

				if(!IsBoxInside(child1BoxLo, child1BoxHi, boxLo, boxHi))
				{
					printf("BLOBTREE ERROR: Left child of op %d box is not inside parent!\n", i);
					ctErrors++;
				}

				if(!IsBoxInside(child2BoxLo, child2BoxHi, boxLo, boxHi))
				{
					printf("BLOBTREE ERROR: Right child of op %d box is not inside parent!\n", i);
					ctErrors++;
				}
			}


			if((boxHi.x < boxLo.x)||(boxHi.y < boxLo.y)||(boxHi.z < boxLo.z))
			{
				printf("BLOBTREE ERROR: Bounding Box Error at Op %d.\n", i);
				ctErrors++;
			}
		}

	}

	return ctErrors;
}

int PrepareAllBoxes(SOABlobOps& blobOps,
					SOABlobPrims& blobPrims,
					SOABlobBoxMatrices& boxMatrices)
{
	int res = PrepareAllPrimBBoxes(blobPrims, boxMatrices);
	if(res != RET_SUCCESS)
	{
		printf("ERROR: Failed to compute primitive bbox. \n");
		return res;
	}

	//Compute correct operator bboxes for Non-Instanced SubTrees
	vec3f bboxLo, bboxHi;
	res = PrepareOpBBox_Recursive(0, blobOps, blobPrims, boxMatrices, bboxLo, bboxHi);
	if(res != RET_SUCCESS)
	{
		printf("Failed to compute operator bboxes for non-instanced nodes. \n");
		return res;
	}

	//Compute bboxes for instanced nodes
	res = PrepareAllInstancedNodesBBoxes(blobOps, blobPrims, boxMatrices);
	if(res != RET_SUCCESS)
	{
		printf("Failed to compute instanced node bbox. \n");
		return res;
	}

	//Re-Compute operator bboxes for all ops
	res = PrepareOpBBox_Recursive(0, blobOps, blobPrims, boxMatrices, bboxLo, bboxHi);
	if(res != RET_SUCCESS)
	{
		printf("Failed to compute operator bboxes for all nodes. \n");
		return res;
	}

	//Compute Entire Model Bounding Box
	bboxLo = vec3f(FLT_MAX, FLT_MAX, FLT_MAX);
	bboxHi = vec3f(FLT_MIN, FLT_MIN, FLT_MIN);
	for(U32 i=0; i < blobPrims.count; i++)
	{
		vec3f curLo = vec3f(blobPrims.bboxLoX[i], blobPrims.bboxLoY[i], blobPrims.bboxLoZ[i]);
		vec3f curHi = vec3f(blobPrims.bboxHiX[i], blobPrims.bboxHiY[i], blobPrims.bboxHiZ[i]);

		bboxLo = vec3f::minP(bboxLo, curLo);
		bboxHi = vec3f::maxP(bboxHi, curHi);
	}

	//Model BBOX
	blobPrims.bboxLo = bboxLo;
	blobPrims.bboxHi = bboxHi;

	return res;
}


//KDTree and Blob Primitive Bounding Volumes are not created every time
//Just whenever there is a change in the Blobtree
int PrepareAllPrimBBoxes(SOABlobPrims& blobPrims,
				  		 SOABlobBoxMatrices& boxMatrices)
{
	if(blobPrims.count == 0)
		return RET_PARAM_ERROR;

	//U8 cost[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	//Compute all primitive BBoxes
	//Boxes are provided with input structure in future
	{
		float offset = ISO_VALUE;

		//Computing bounding box per each primitive using its skeletal info
		for(U32 i=0; i<blobPrims.count; i++)
		{
			vec3f curLo = vec3f(FLT_MAX, FLT_MAX, FLT_MAX);
			vec3f curHi = vec3f(FLT_MIN, FLT_MIN, FLT_MIN);
			vec3f seed = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);

			switch(blobPrims.type[i])
			{
			case(primPoint): case(primNULL):
				{
					vec3f off = vec3f(offset, offset, offset);
					vec3f pos = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);
					curLo = vec3f::sub(pos, off);
					curHi = vec3f::add(pos, off);
				}
				break;
			case(primLine):
				{
					vec3f s0 = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);
					vec3f s1 = vec3f(blobPrims.dirX[i], blobPrims.dirY[i], blobPrims.dirZ[i]);
					vec3f expand = vec3f::add(vec3f::mul(offset, vec3f(1.0f, 1.0f, 1.0f)), vec3f::mul(3.0f*offset, vec3f::sub(s1, s0)));
					curLo = vec3f::sub(s0, expand);
					curHi = vec3f::add(s1, expand);
				}
				break;
			case(primRing):
				{
					vec3f pos = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);
					vec3f dir = vec3f(blobPrims.dirX[i], blobPrims.dirY[i], blobPrims.dirZ[i]);
					float radius = blobPrims.resX[i];

					vec3f dirComp = vec3f::sub(vec3f(1.0f, 1.0f, 1.0f), dir);
					radius += offset;

					vec3f expand = vec3f::add(vec3f::mul(radius + offset, dirComp), vec3f::mul(offset, dir));
					curLo = vec3f::sub(pos, expand);
					curHi = vec3f::add(pos, expand);
				}
				break;
			case(primDisc):
				{
					vec3f pos = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);
					vec3f dir = vec3f(blobPrims.dirX[i], blobPrims.dirY[i], blobPrims.dirZ[i]);
					float radius = blobPrims.resX[i];

					vec3f dirComp = vec3f::sub(vec3f(1.0f, 1.0f, 1.0f), dir);
					radius += offset;

					vec3f expand = vec3f::add(vec3f::mul(radius + offset, dirComp), vec3f::mul(offset, dir));
					curLo = vec3f::sub(pos, expand);
					curHi = vec3f::add(pos, expand);
				}
				break;
			case(primCylinder):
				{
					vec3f s0 = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);
					vec3f dir = vec3f(blobPrims.dirX[i], blobPrims.dirY[i], blobPrims.dirZ[i]);
					float radius = blobPrims.resX[i];
					float height = blobPrims.resY[i];

					vec3f s1 = vec3f::add(s0, vec3f::mul(height, dir));
					vec3f expand = vec3f::add(vec3f::mul(offset + radius, vec3f(1.0f, 1.0f, 1.0f)), vec3f::mul(0.5f*offset, dir));
					curLo = vec3f::sub(s0, expand);
					curHi = vec3f::add(s1, expand);
				}
				break;
			case(primCube):
				{
					vec3f s0 = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);
					float side = blobPrims.resX[i] + offset;

					curLo = vec3f::sub(s0, vec3f(side, side, side));
					curHi = vec3f::add(s0, vec3f(side, side, side));
				}
				break;
			case(primTriangle):
				{
					vec3f off = vec3f(offset, offset, offset);
					vec3f s0 = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);
					vec3f s1 = vec3f(blobPrims.dirX[i], blobPrims.dirY[i], blobPrims.dirZ[i]);
					vec3f s2 = vec3f(blobPrims.resX[i], blobPrims.resY[i], blobPrims.resZ[i]);

					curLo = vec3f::sub(vec3f::minP(vec3f::minP(s0, s1), s2), off);
					curHi = vec3f::add(vec3f::maxP(vec3f::maxP(s0, s1), s2), off);
				}
				break;
			case(primQuadricPoint):
				{
					vec3f pos = vec3f(blobPrims.posX[i], blobPrims.posY[i], blobPrims.posZ[i]);
					float w = blobPrims.dirY[i] + offset;
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
			int idxMatrix = blobPrims.idxMatrix[i];
			if(idxMatrix != 0)
			{
				assert(idxMatrix < (int)boxMatrices.count);
				idxMatrix *= BOX_MATRIX_STRIDE;
				//MAT44 mat(&boxMatrices.matrix[idxMatrix]);
				mat44f mat(&boxMatrices.matrix[idxMatrix]);

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
			blobPrims.bboxLoX[i] = curLo.x;
			blobPrims.bboxLoY[i] = curLo.y;
			blobPrims.bboxLoZ[i] = curLo.z;
			blobPrims.bboxHiX[i] = curHi.x;
			blobPrims.bboxHiY[i] = curHi.y;
			blobPrims.bboxHiZ[i] = curHi.z;

			blobPrims.seedX[i] = seed.x;
			blobPrims.seedY[i] = seed.y;
			blobPrims.seedZ[i] = seed.z;
		}
	}

	return RET_SUCCESS;
}

int PrepareOpBBox_Recursive(U32 idxNode,
							SOABlobOps& blobOps,
							SOABlobPrims& blobPrims,
						  	SOABlobBoxMatrices& boxMatrices,
						  	vec3f& boxLo, vec3f& boxHi)
{
	if(blobOps.count == 0)
		return RET_SUCCESS;
	if(idxNode >= blobOps.count)
		return RET_PARAM_ERROR;

	U32 idxLC, idxRC;
	bool bLeftChildOp, bRightChildOp, bRange, bUnary;

	bUnary     = (blobOps.opFlags[idxNode] & ofIsUnaryOp) >> 3;
	bRange 	   = (blobOps.opFlags[idxNode] & ofChildIndexIsRange) >> 2;
	bLeftChildOp  = (blobOps.opFlags[idxNode] & ofLeftChildIsOp) >> 1;
	bRightChildOp = (blobOps.opFlags[idxNode] & ofRightChildIsOp);
	idxLC = blobOps.opLeftChild[idxNode];
	idxRC = blobOps.opRightChild[idxNode];

	//Range
	if(bRange)
	{
		vec3f primBoxLo;
		vec3f primBoxHi;

		for(U16 i=idxLC; i<=idxRC; i++)
		{
			primBoxLo = vec3f(blobPrims.bboxLoX[i], blobPrims.bboxLoY[i], blobPrims.bboxLoZ[i]);
			primBoxHi = vec3f(blobPrims.bboxHiX[i], blobPrims.bboxHiY[i], blobPrims.bboxHiZ[i]);

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

		blobOps.bboxLoX[idxNode] = boxLo.x;
		blobOps.bboxLoY[idxNode] = boxLo.y;
		blobOps.bboxLoZ[idxNode] = boxLo.z;

		blobOps.bboxHiX[idxNode] = boxHi.x;
		blobOps.bboxHiY[idxNode] = boxHi.y;
		blobOps.bboxHiZ[idxNode] = boxHi.z;
	}
	else
	{
		//Left Child Box
		vec3f boxLoLC, boxHiLC;
		if(bLeftChildOp)
			PrepareOpBBox_Recursive(idxLC, blobOps, blobPrims, boxMatrices, boxLoLC, boxHiLC);
		else
		{
			boxLoLC = vec3f(blobPrims.bboxLoX[idxLC], blobPrims.bboxLoY[idxLC], blobPrims.bboxLoZ[idxLC]);
			boxHiLC = vec3f(blobPrims.bboxHiX[idxLC], blobPrims.bboxHiY[idxLC], blobPrims.bboxHiZ[idxLC]);
		}

		//Unary
		if(bUnary)
		{
			int idxMatrix = static_cast<int>(blobOps.resX[idxNode]);
			if(idxMatrix != 0)
			{
				assert(idxMatrix < (int)boxMatrices.count);
				idxMatrix *= BOX_MATRIX_STRIDE;
				//MAT44 mat(&boxMatrices.matrix[idxMatrix]);
				mat44f mat(&boxMatrices.matrix[idxMatrix]);

				//Set BBOX
				vec3f curLoT = mat.mapAffine(boxLo);
				vec3f curHiT = mat.mapAffine(boxHi);

				boxLo = vec3f::minP(curLoT, curHiT);
				boxHi = vec3f::maxP(curLoT, curHiT);

				blobOps.bboxLoX[idxNode] = boxLo.x;
				blobOps.bboxLoY[idxNode] = boxLo.y;
				blobOps.bboxLoZ[idxNode] = boxLo.z;

				blobOps.bboxHiX[idxNode] = boxHi.x;
				blobOps.bboxHiY[idxNode] = boxHi.y;
				blobOps.bboxHiZ[idxNode] = boxHi.z;
			}
		}
		else
		{
			vec3f boxLoRC, boxHiRC;
			if(bRightChildOp)
				PrepareOpBBox_Recursive(idxRC, blobOps, blobPrims, boxMatrices, boxLoRC, boxHiRC);
			else
			{
				boxLoRC = vec3f(blobPrims.bboxLoX[idxRC], blobPrims.bboxLoY[idxRC], blobPrims.bboxLoZ[idxRC]);
				boxHiRC = vec3f(blobPrims.bboxHiX[idxRC], blobPrims.bboxHiY[idxRC], blobPrims.bboxHiZ[idxRC]);
			}

			boxLo = vec3f::minP(boxLoLC, boxLoRC);
			boxHi = vec3f::maxP(boxHiLC, boxHiRC);

			blobOps.bboxLoX[idxNode] = boxLo.x;
			blobOps.bboxLoY[idxNode] = boxLo.y;
			blobOps.bboxLoZ[idxNode] = boxLo.z;

			blobOps.bboxHiX[idxNode] = boxHi.x;
			blobOps.bboxHiY[idxNode] = boxHi.y;
			blobOps.bboxHiZ[idxNode] = boxHi.z;
		}//bUnary
	}//bRange

	return RET_SUCCESS;
}

int PrepareAllInstancedNodesBBoxes(SOABlobOps& blobOps,
								   SOABlobPrims& blobPrims,
								   SOABlobBoxMatrices& boxMatrices)
{
	for(U32 i=0; i<blobPrims.count; i++)
	{
		if(blobPrims.type[i] == primInstance)
		{
			//original node index
			U32 idxOrigin = blobPrims.resX[i];
			U32 isOriginOp = blobPrims.resZ[i];
			U8 originNodeType = blobPrims.dirX[i];
			vec3f curLo, curHi;

			if(isOriginOp)
			{
				assert(blobOps.type[idxOrigin] == originNodeType);
				curLo = vec3f(blobOps.bboxLoX[idxOrigin], blobOps.bboxLoY[idxOrigin], blobOps.bboxLoZ[idxOrigin]);
				curHi = vec3f(blobOps.bboxHiX[idxOrigin], blobOps.bboxHiY[idxOrigin], blobOps.bboxHiZ[idxOrigin]);
			}
			else
			{
				assert(blobPrims.type[idxOrigin] == originNodeType);
				curLo = vec3f(blobPrims.bboxLoX[idxOrigin], blobPrims.bboxLoY[idxOrigin], blobPrims.bboxLoZ[idxOrigin]);
				curHi = vec3f(blobPrims.bboxHiX[idxOrigin], blobPrims.bboxHiY[idxOrigin], blobPrims.bboxHiZ[idxOrigin]);
			}

			//Apply Transformation Matrix to BBox
			int idxMatrix = blobPrims.idxMatrix[i];
			assert(idxMatrix < (int)boxMatrices.count);

			idxMatrix *= BOX_MATRIX_STRIDE;
			//MAT44 mat(&boxMatrices.matrix[idxMatrix]);
			mat44f mat(&boxMatrices.matrix[idxMatrix]);

			//Set BBOX
			//vec3f curLoT = mat4Transform(mat, curLo);
			//vec3f curHiT = mat4Transform(mat, curHi);
			vec3f curLoT = mat.mapAffine(curLo);
			vec3f curHiT = mat.mapAffine(curHi);


			curLo = vec3f::minP(curLoT, curHiT);
			curHi = vec3f::maxP(curLoT, curHiT);

			blobPrims.bboxLoX[i] = curLo.x;
			blobPrims.bboxLoY[i] = curLo.y;
			blobPrims.bboxLoZ[i] = curLo.z;

			blobPrims.bboxHiX[i] = curHi.x;
			blobPrims.bboxHiY[i] = curHi.y;
			blobPrims.bboxHiZ[i] = curHi.z;

			//Set SEED
			vec3f seed = vec3f::mul(0.5f, vec3f::add(curLo, curHi));
			blobPrims.seedX[i] = seed.x;
			blobPrims.seedY[i] = seed.y;
			blobPrims.seedZ[i] = seed.z;
		}
	}

	return RET_SUCCESS;
}


/*!
 * Main polygonizer function. Creates MPUs and setup the runnable threads using
 * TBB
 */
int Polygonize(float cellsize,
			   bool bScalarRun,
			   const SOABlobOps& blobOps,
			   const SOABlobPrims& blobPrims,
			   const SOABlobNodeMatrices& blobPrimMatrices,
			   U32 	ctMPUs,
			   MPU* lpMPUs,
			   MPUGLOBALMESH* lpGlobalMesh,
			   MPUSTATS* lpProcessStats)
{
	if(blobPrims.count == 0)
		return RET_PARAM_ERROR;
	if((lpMPUs == NULL)||(ctMPUs == 0))
		return RET_PARAM_ERROR;

	//Now Run Threads
	ThreadStartSetup startSetup(&blobOps, &blobPrims, &blobPrimMatrices);
	CMPUProcessor body(cellsize, bScalarRun, ctMPUs, lpMPUs, lpGlobalMesh, lpProcessStats);
	tbb::parallel_for(blocked_range<size_t>(0, ctMPUs), body, tbb::auto_partitioner());
	g_threadFieldComputer.clear();

	return RET_SUCCESS;
}

void PrintThreadResults(int ctAttempts, U32* lpThreadProcessed, U32* lpThreadCrossed)
{
	//Print Results
	int idxThread = 0;
	for(CounterTotalCrossedMPU::const_iterator i = g_ctTotalCrossed.begin(); i != g_ctTotalCrossed.end(); i++)
	{
		if(lpThreadProcessed)
			lpThreadProcessed[idxThread] = i->first / ctAttempts;
		if(lpThreadCrossed)
			lpThreadCrossed[idxThread] = i->second / ctAttempts;
		idxThread++;
		printf("Thread#  %d, Processed MPUs %d, Crossed MPUs %d \n", idxThread, i->first / ctAttempts, i->second / ctAttempts);
	}
	g_ctTotalCrossed.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
CMPUProcessor::CMPUProcessor(float cellsize, bool bScalarRun,
							 size_t ctMPUs, MPU* lpMPU, MPUGLOBALMESH* lpGlobalMesh, MPUSTATS* lpStats)

{
	m_cellsize = cellsize;
	m_ctMPUs = ctMPUs;	
	m_lpMPU = lpMPU;
	m_lpGlobalMesh = lpGlobalMesh;
	m_lpStats = lpStats;
	m_bScalarRun = bScalarRun;
}

void CMPUProcessor::operator()(const blocked_range<size_t>& range) const
{
	CounterTotalCrossedMPU::reference m_localCounter = g_ctTotalCrossed.local();
	ThreadFieldComputer::reference m_localFieldComputer = g_threadFieldComputer.local();
	tbb_thread::id threadID = this_tbb_thread::get_id();

	tbb::tick_count tickFieldEvals;
	for(size_t i=range.begin(); i!= range.end(); i++)
	{
		if(m_lpStats)
		{
			m_lpStats[i].threadID = threadID;
			m_lpStats[i].tickStart = tick_count::now();
			tickFieldEvals = m_lpStats[i].tickStart;
		}

		//Call MPU processor
		if(m_bScalarRun)
			process_cells_scalar(m_localFieldComputer, m_lpMPU[i], *m_lpGlobalMesh, tickFieldEvals);
		else
			process_cells_simd(m_localFieldComputer, m_lpMPU[i], *m_lpGlobalMesh, tickFieldEvals);

		if(m_lpStats)
		{
			m_lpStats[i].tickEnd = tick_count::now();
			m_lpStats[i].tickFieldEvals = tickFieldEvals;
		}

		//Increment Total MPU Counter
		++m_localCounter.first;

		//Increment intersected MPUs Counter
		if(m_lpMPU[i].ctTriangles > 0)
			++m_localCounter.second;
	}
}

/**
 * Discard an MPU early in the algorithm
 */
bool CMPUProcessor::discard(const FieldComputer& fc, MPU& mpu) const
{
	const float mpuSide = (GRID_DIM - 1) * m_cellsize;
	if(mpuSide > 1.0f)
		return false;

	//const float halfSide = 0.5f * mpuSide;

	//Early Discard Check
	bool bDiscard = true;

	float PS_SIMD_ALIGN(arrX[8]);
	float PS_SIMD_ALIGN(arrY[8]);
	float PS_SIMD_ALIGN(arrZ[8]);

	for(int i=0; i<8; i++)
	{
		arrX[i] = (float)((0xAA >> i) & 1);
		arrY[i] = (float)((0xCC >> i) & 1);
		arrZ[i] = (float)((0xF0 >> i) & 1);
	}
#ifdef SIMD_USE_M128
	//My SIMD VARS
	Float_ X(&arrX[0]);
	Float_ Y(&arrY[0]);
	Float_ Z1(&arrZ[0]);
	Float_ Z2(&arrZ[4]);
	Float_ zero(0.0f);
	Float_ one(1.0f);

	//Center
	Float_ side(mpuSide);
	X = (X * side) + Float_(mpu.bboxLo.x);
	Y = (Y * side) + Float_(mpu.bboxLo.y);
	Z1 = (Z1 * side) + Float_(mpu.bboxLo.z);
	Z2 = (Z2 * side) + Float_(mpu.bboxLo.z);

	Float_ F1;
	Float_ F2;
	fc.fieldValue(X, Y, Z1, F1);
	fc.fieldValue(X, Y, Z2, F2);

	//Update Stats
	mpu.ctFieldEvals += 2;

	F1 = SimdAnd(SimdGT(F1, zero), one);
	F2 = SimdAnd(SimdGT(F2, zero), one);
	VecNMask mask1(F1.v);
	VecNMask mask2(F2.v);
	if((mask1 != PS_SIMD_ALLZERO)||(mask2 != PS_SIMD_ALLZERO))
	{
		bDiscard = false;
	}
#elif defined(SIMD_USE_M256)
	Float_ X(&arrX[0]);
	Float_ Y(&arrY[0]);
	Float_ Z(&arrZ[0]);
	Float_ zero(0.0f);
	Float_ one(1.0f);

	//Center
	Float_ side(mpuSide);
	X = (X * side) + Float_(mpu.bboxLo.x);
	Y = (Y * side) + Float_(mpu.bboxLo.y);
	Z = (Z * side) + Float_(mpu.bboxLo.z);

	Float_ F1;
	fc.fieldValue(X, Y, Z, F1);

	//Update Stats
	mpu.ctFieldEvals++;

	F1 = SimdAnd(SimdGT(F1, zero), one);
	VecNMask mask1(F1.v);

	if(mask1 != PS_SIMD_ALLZERO)
	{
		bDiscard = false;
	}
#endif


	/*
#ifdef SIMD_USE_M128
	//My SIMD VARS
	Float_ X(&arrX[0]);
	Float_ Y(&arrY[0]);
	Float_ Z1(&arrZ[0]);
	Float_ Z2(&arrZ[4]);
	Float_ zero(0.0f);
	Float_ one(1.0f);

	float s = MATHMIN(0.5f, halfSide);

	//Center
	float cx = mpu.bboxLo.x + halfSide;
	float cy = mpu.bboxLo.y + halfSide;
	float cz = mpu.bboxLo.z + halfSide;

	while(s <= halfSide)
	{
		Float_ side(2*s);
		X = (X * side) + Float_(cx - s);
		Y = (Y * side) + Float_(cy - s);
		Z1 = (Z1 * side) + Float_(cz - s);
		Z2 = (Z2 * side) + Float_(cz - s);

		Float_ F1;
		Float_ F2;
		fc.fieldValue(X, Y, Z1, F1);
		fc.fieldValue(X, Y, Z2, F2);

		//Update Stats
		mpu.ctFieldEvals += 2;

		F1 = SimdAnd(SimdGT(F1, zero), one);
		F2 = SimdAnd(SimdGT(F2, zero), one);
		VecNMask mask1(F1.v);
		VecNMask mask2(F2.v);
		if((mask1 != PS_SIMD_ALLZERO)||(mask2 != PS_SIMD_ALLZERO))
		{
			bDiscard = false;
			break;
		}

		if(s < halfSide)
			s = MATHMIN(s + 0.5f, halfSide);
		else
			break;
	}

#elif defined(SIMD_USE_M256)
	Float_ X(&arrX[0]);
	Float_ Y(&arrY[0]);
	Float_ Z(&arrZ[0]);
	Float_ zero(0.0f);
	Float_ one(1.0f);
	Float_ side(mpuSide);

	float s = MATHMIN(0.5f, halfSide);

	//Center
	float cx = mpu.bboxLo.x + halfSide;
	float cy = mpu.bboxLo.y + halfSide;
	float cz = mpu.bboxLo.z + halfSide;

	while(s <= halfSide)
	{
		Float_ side(2*s);
		X = (X * side) + Float_(cx - s);
		Y = (Y * side) + Float_(cy - s);
		Z = (Z * side) + Float_(cz - s);


		Float_ F1;
		fc.fieldValue(X, Y, Z, F1);

		//Update Stats
		mpu.ctFieldEvals++;


		F1 = SimdAnd(SimdGT(F1, zero), one);
		VecNMask mask1(F1.v);

		if(mask1 != PS_SIMD_ALLZERO)
		{
			bDiscard = false;
			break;
		}

		if(s < halfSide)
			s = MATHMIN(s + 0.5f, halfSide);
		else
			break;
	}

#endif
*/

	return bDiscard;
}



/*!
 * Processes cells in SIMD
 */
void CMPUProcessor::process_cells_simd(const FieldComputer& fc, MPU& mpu, MPUGLOBALMESH& globalMesh, tbb::tick_count& tickFieldEvals) const
{
	//Zero EveryThing
	//mpu.error = 0.0f;
	mpu.ctFieldEvals = 0;
	mpu.ctVertices = 0;
	mpu.ctTriangles = 0;

	//Discard MPU is doesnot contain field
	if(this->discard(fc, mpu))
		return;

	//Create Field-Value Cache and init it
	const U32 m = GRID_DIM * GRID_DIM * GRID_DIM;
	float PS_SIMD_ALIGN(fvCache[m]);

	//Init
	EDGETABLE edgeTable;
	memset(&edgeTable, 0, sizeof(EDGETABLE));


	//Compute steps
	const int kMax = PS_SIMD_BLOCKS(GRID_DIM);
	const int szOneNeedle = PS_SIMD_FLEN * kMax;
	const int szOneSlab = szOneNeedle * GRID_DIM;

	//Temporary goodies
	Float_ simdIsoVal(ISO_VALUE);
	Float_ simdOne(1.0f);
	float PS_SIMD_ALIGN(arrScaleZ[8]);
	for(int iSimd=0; iSimd<8; iSimd++)
		arrScaleZ[iSimd] = float(iSimd);

	//Compute field at grid positions
	{
		//Temp variables
		Float_ simdIn(0.0f);
		Float_ simdOut(0.0f);

		Float_ simdCellSize(m_cellsize);
		Float_ simdScaleZ(arrScaleZ);

		//Cache all field-values with in this MPU
		for(int i=0; i<GRID_DIM; i++)
		{
			for(int j=0; j<GRID_DIM; j++)
			{
				for(int k=0; k<kMax; k++)
				{
					Float_ simdPX(mpu.bboxLo.x + i*m_cellsize);
					Float_ simdPY(mpu.bboxLo.y + j*m_cellsize);
					Float_ simdPZ(mpu.bboxLo.z);
					Float_ simdK(k * PS_SIMD_FLEN);

					simdPZ = simdPZ + (simdK + simdScaleZ) * simdCellSize;

					Float_ arrField;

					//FieldValue computed using SIMD
					fc.fieldValue(simdPX, simdPY, simdPZ, arrField);

					//Store Fields Row By Row
					arrField.store(&fvCache[ i * szOneNeedle + j * szOneSlab + k * PS_SIMD_FLEN]);

					//Keep stats
					mpu.ctFieldEvals++;
					simdPX = SimdAnd(SimdGTE(arrField, simdIsoVal), simdOne);
					simdIn = simdIn + simdPX;
					simdOut = simdOut + simdOne - simdPX;
				}
			}
		}

		//Early Discard Shaved 2 seconds off
		VecNMask mask1(simdIn.v);
		if(mask1 == PS_SIMD_ALLZERO)
			return;

		mask1 = VecNMask(simdOut.v);
		if(mask1 == PS_SIMD_ALLZERO)
			return;
	}

	//Get Tick Count for FieldEvals
	tickFieldEvals = tbb::tick_count::now();

	//Triangulation: First compute edge intersection points for all cells
	//then compute color and normals for all vertices
	/////////////////////////////////////////////////////////
	//For the 8 vertices of each cell
	U16* lpMeshTriangles = &globalMesh.vTriangles[mpu.idxGlobalID * MPU_MESHPART_TRIANGLE_STRIDE];

	U32 szVertexPartOffset = mpu.idxGlobalID * MPU_MESHPART_VERTEX_STRIDE;
	float* lpMeshVertices = &globalMesh.vPos[szVertexPartOffset];
	float* lpMeshNormals = &globalMesh.vNorm[szVertexPartOffset];
	float* lpMeshColors = &globalMesh.vColor[szVertexPartOffset];


	float PS_SIMD_ALIGN(arrPowerMask[8]);
	float PS_SIMD_ALIGN(arrFields[8]);

	arrPowerMask[0] = 1.0f;
	for(int iSimd=1; iSimd<8; iSimd++)
		arrPowerMask[iSimd] = 2.0f * arrPowerMask[iSimd - 1];

	int arrConfig[16];
	int idxCellConfig = 0;

#ifdef SIMD_USE_M128
	Float_ simdMaskPowerLeft = Float_(&arrPowerMask[0]);
	Float_ simdMaskPowerRight = Float_(&arrPowerMask[4]);

	Float_ simdRootIntervals1 = Float_(&arrScaleZ[0]);
	Float_ simdRootIntervals2 = Float_(&arrScaleZ[4]);
	{
		Float_ simdOneSeventh = Float_(1.0f / 7.0f);
		simdRootIntervals1 = simdRootIntervals1 * simdOneSeventh;
		simdRootIntervals2 = simdRootIntervals2 * simdOneSeventh;
	}

#elif defined(SIMD_USE_M256)
	Float_ maskPower = Float_(&arrPowerMask[0]);
	Float_ simdRootIntervals = Float_(&arrScaleZ[0]);
	{
		Float_ simdOneSeventh = Float_(1.0f / 7.0f);
		simdRootIntervals = simdRootIntervals * simdOneSeventh;
	}
#endif

	for(int i=0; i<GRID_DIM-1; i++)
	{
		//Compute once use multiple of i
		U32 ip0 = i * szOneNeedle;
		U32 ip1 = ip0 + szOneNeedle;

		for(int j=0; j<GRID_DIM-1; j++)
		{
			//Compute once use multiple of j
			U32 jp0 = j * szOneSlab;
			U32 jp1 = jp0 + szOneSlab;

			for(int k=0; k<GRID_DIM-1; k++)
			{
				//GATHER FIELDS
				U32 idx = ip0 + jp0 + k;
				arrFields[0] = fvCache[ idx];
				arrFields[1] = fvCache[ idx + 1];

				idx = ip0 + jp1 + k;
				arrFields[2] = fvCache[ idx];
				arrFields[3] = fvCache[ idx + 1];

				idx = ip1 + jp0 + k;
				arrFields[4] = fvCache[ idx];
				arrFields[5] = fvCache[ idx + 1];

				idx = ip1 + jp1 + k;
				arrFields[6] = fvCache[ idx];
				arrFields[7] = fvCache[ idx + 1];
#ifdef SIMD_USE_M128
				//Compute configuration
				Float_ Left = SimdAnd(SimdGTE(Float_(&arrFields[0]), simdIsoVal), simdOne) * simdMaskPowerLeft;
				Float_ Right = SimdAnd(SimdGTE(Float_(&arrFields[4]), simdIsoVal), simdOne) * simdMaskPowerRight;

				//3 Levels of HADDS Needed
				Left = SimdhAdd(Left, Right);
				Left = SimdhAdd(Left, Left);
				Left = SimdhAdd(Left, Left);
				idxCellConfig = static_cast<int>(Left[0]);
#elif defined(SIMD_USE_M256)
				Float_ Left = SimdAnd(SimdGTE(Float_(&arrFields[0]), simdIsoVal), simdOne) * maskPower;
				Left = SimdhAdd(Left, Left);
				Left = SimdhAdd(Left, Left);
				idxCellConfig = static_cast<int>(Left[0] + Left[4]);
#endif
				if(idxCellConfig == 0 || idxCellConfig == 255)
					continue;

				//Fetch cell configuration
				memcpy(arrConfig, &g_triTableCache[idxCellConfig][0], 16 * sizeof(int));
				//Increment number of processed cells
				//Compute surface vertex, normal and material on each crossing edge
				int idxMeshVertex[16];
				Left = SimdhAdd(Left, Left);
				//Read case
				int ctTotalPolygons = 0;
				int ctEdges = 0;

				for(int icase=0; icase<16; icase++)
				{
					int candidate = arrConfig[icase];
					if(candidate == -1)
						break;

					//New Edge
					ctEdges++;
					int idxEdgeStart = corner1[candidate];
					int idxEdgeAxis  = edgeaxis[candidate];

					//Compute indices
					int sx = i + ((idxEdgeStart & 4) >> 2);
					int sy = j + ((idxEdgeStart & 2) >> 1);
					int sz = k + (idxEdgeStart & 1);

					idxMeshVertex[icase] = getEdge(edgeTable, sx, sy, sz, idxEdgeAxis);

					//See if the vertex exist in edge table. If it doesn't exist compute and add it to edge table
					if(idxMeshVertex[icase] == -1)
					{
						//Reduced operations for edge processing
						vec3f e1 = vec3f::add(mpu.bboxLo, vec3f(m_cellsize * sx, m_cellsize * sy, m_cellsize * sz));
						vec3f e2 = e1;

						e2.setElement(idxEdgeAxis, e2.element(idxEdgeAxis) + m_cellsize);

						//int idxEdgeStart = corner2[candidate];
						Float_ simdDisplaceX(e2.x - e1.x);
						Float_ simdDisplaceY(e2.y - e1.y);
						Float_ simdDisplaceZ(e2.z - e1.z);
						float PS_SIMD_ALIGN(arrPosX[8]);
						float PS_SIMD_ALIGN(arrPosY[8]);
						float PS_SIMD_ALIGN(arrPosZ[8]);

						int interval = 0;
						//Compute Interval
						{
#if defined(SIMD_USE_M128)
							Float_ rootFields1;
							Float_ rootFields2;

							//Compute Field for the first 4 vertices along the edge
							Float_ posX = Float_(e1.x) + simdDisplaceX * simdRootIntervals1;
							Float_ posY = Float_(e1.y) + simdDisplaceY * simdRootIntervals1;
							Float_ posZ = Float_(e1.z) + simdDisplaceZ * simdRootIntervals1;
							fc.fieldValue(posX, posY, posZ, rootFields1);
							posX.store(&arrPosX[0]);
							posY.store(&arrPosY[0]);
							posZ.store(&arrPosZ[0]);
							rootFields1.store(&arrFields[0]);

							//Compute Field for the second 4 vertices along the edge
							posX = Float_(e1.x) + simdDisplaceX * simdRootIntervals2;
							posY = Float_(e1.y) + simdDisplaceY * simdRootIntervals2;
							posZ = Float_(e1.z) + simdDisplaceZ * simdRootIntervals2;
							fc.fieldValue(posX, posY, posZ, rootFields2);
							posX.store(&arrPosX[4]);
							posY.store(&arrPosY[4]);
							posZ.store(&arrPosZ[4]);
							rootFields2.store(&arrFields[4]);
							mpu.ctFieldEvals += 2;

							Float_ inside1 = SimdAnd(SimdGTE(rootFields1, simdIsoVal), simdOne);
							Float_ inside2 = SimdAnd(SimdGTE(rootFields2, simdIsoVal), simdOne);

							bool bRight = (static_cast<int>(inside1[0]) == 0);
							inside1 = SimdhAdd(inside1, inside2);
							inside1 = SimdhAdd(inside1, inside1);
							inside1 = SimdhAdd(inside1, inside1);

							int ii = static_cast<int>(inside1[0]);
							interval = bRight * (8 - ii) + !bRight * ii;
#elif defined(SIMD_USE_M256)
							Float_ rootFields;
							Float_ posX = Float_(e1.x) + simdDisplaceX * simdRootIntervals;
							Float_ posY = Float_(e1.y) + simdDisplaceY * simdRootIntervals;
							Float_ posZ = Float_(e1.z) + simdDisplaceZ * simdRootIntervals;
							fc.fieldValue(posX, posY, posZ, rootFields);
							posX.store(&arrPosX[0]);
							posY.store(&arrPosY[0]);
							posZ.store(&arrPosZ[0]);
							rootFields.store(&arrFields[0]);
							mpu.ctFieldEvals++;

							Float_ inside = SimdAnd(SimdGTE(rootFields, simdIsoVal), simdOne);

							bool bRight = (static_cast<int>(inside[0]) == 0);
							inside = SimdhAdd(inside, inside);
							inside = SimdhAdd(inside, inside);
							int ii = static_cast<int>(inside[0] + inside[4]);
							interval = bRight * (8 - ii) + !bRight * ii;
#endif
						}

						//Update e1 and e2 for the high resolution field
						e1 = vec3f(arrPosX[interval - 1], arrPosY[interval - 1], arrPosZ[interval - 1]);
						e2 = vec3f(arrPosX[interval], arrPosY[interval], arrPosZ[interval]);

						float scale = (ISO_VALUE - arrFields[interval - 1])/(arrFields[interval] - arrFields[interval-1]);
						vec3f p = vec3f::add(e1, vec3f::mul(scale, vec3f::sub(e2, e1)));

						U16 idxVertex = mpu.ctVertices;
						U16 idxVertex_X = idxVertex * 3;
						U16 idxVertex_Y = idxVertex_X + 1;
						U16 idxVertex_Z = idxVertex_X + 2;

						lpMeshVertices[idxVertex_X]   = p.x;
						lpMeshVertices[idxVertex_Y]   = p.y;
						lpMeshVertices[idxVertex_Z]   = p.z;
						mpu.ctVertices++;

						//Get vertex v index from list. It is the last one
						idxMeshVertex[icase] = idxVertex;
						setEdge(edgeTable, sx, sy, sz, idxEdgeAxis, idxVertex);

						//Test Pourya
						/*
						Float_ simdPX = Float_(p.x);
						Float_ simdPY = Float_(p.y);
						Float_ simdPZ = Float_(p.z);
						Float_ simdOutField;
						Float_ simdOutColorX;
						Float_ simdOutColorY;
						Float_ simdOutColorZ;

						Float_ simdOutNormalX;
						Float_ simdOutNormalY;
						Float_ simdOutNormalZ;

						//Compute Field and Color
						fc.fieldValueAndColor(simdPX, simdPY, simdPZ,
											  simdOutField, simdOutColorX, simdOutColorY, simdOutColorZ);

						//Use computed field to get the normal
						fc.normal(simdPX, simdPY, simdPZ, simdOutField,
								simdOutNormalX, simdOutNormalY, simdOutNormalZ, NORMAL_DELTA);

						//Test Pourya
						lpMeshNormals[idxVertex_X]   = simdOutNormalX[0];
						lpMeshNormals[idxVertex_Y]   = simdOutNormalY[0];
						lpMeshNormals[idxVertex_Z]   = simdOutNormalZ[0];

						lpMeshColors[idxVertex_X]   = simdOutColorX[0];
						lpMeshColors[idxVertex_Y]   = simdOutColorY[0];
						lpMeshColors[idxVertex_Z]   = simdOutColorZ[0];
						*/

					}
				}//End icase

				//Number of polygons
				ctTotalPolygons = ctEdges / 3;
				for(int icase = 0; icase < ctTotalPolygons; icase++)
				{
					int idxTriangle = mpu.ctTriangles * 3;
					lpMeshTriangles[idxTriangle] = idxMeshVertex[icase*3];
					lpMeshTriangles[idxTriangle + 1] = idxMeshVertex[icase*3 + 1];
					lpMeshTriangles[idxTriangle + 2] = idxMeshVertex[icase*3 + 2];
					mpu.ctTriangles++;
				}
			}//Processed One Cell
		}
	}



	//Color and Normals are computed Last
	{
		Float_ simdPX;
		Float_ simdPY;
		Float_ simdPZ;
		Float_ simdOutField;
		Float_ simdOutColorX;
		Float_ simdOutColorY;
		Float_ simdOutColorZ;

		Float_ simdOutNormalX;
		Float_ simdOutNormalY;
		Float_ simdOutNormalZ;

		U32 szBlockStride = PS_SIMD_FLEN * 3;
		//Compute Color and Normals
		U16 ctVertexBlocks = PS_SIMD_BLOCKS(mpu.ctVertices);
		mpu.ctFieldEvals += ctVertexBlocks;
/*
		for(U32 iVertex=0; iVertex<mpu.ctVertices; iVertex++)
		{
			simdPX = Float_(lpMeshVertices[iVertex*3]);
			simdPY = Float_(lpMeshVertices[iVertex*3 + 1]);
			simdPZ = Float_(lpMeshVertices[iVertex*3 + 2]);

			//Compute Field and Color
			fc.fieldValueAndColor(simdPX, simdPY, simdPZ,
								  simdOutField, simdOutColorX, simdOutColorY, simdOutColorZ);
			//Use computed field to get the normal
			fc.normal(simdPX, simdPY, simdPZ, simdOutField,
					  simdOutNormalX, simdOutNormalY, simdOutNormalZ, NORMAL_DELTA);

			U16 idxVertex_X = iVertex * 3;
			U16 idxVertex_Y = idxVertex_X + 1;
			U16 idxVertex_Z = idxVertex_X + 2;

			//Test Pourya
			lpMeshNormals[idxVertex_X]   = simdOutNormalX[0];
			lpMeshNormals[idxVertex_Y]   = simdOutNormalY[0];
			lpMeshNormals[idxVertex_Z]   = simdOutNormalZ[0];

			lpMeshColors[idxVertex_X]   = simdOutColorX[0];
			lpMeshColors[idxVertex_Y]   = simdOutColorY[0];
			lpMeshColors[idxVertex_Z]   = simdOutColorZ[0];
		}
*/
		for(U16 iBlock = 0; iBlock < ctVertexBlocks; iBlock++)
		{
			U32 szBlockOffset = iBlock * szBlockStride;

			//Load ISO-Surface Vertices from Aligned Memory (AOS to SOA Conversion)
			SimdLoadVector(&lpMeshVertices[szBlockOffset], simdPX, simdPY, simdPZ);

			//Compute Field and Color
			fc.fieldValueAndColor(simdPX, simdPY, simdPZ, simdOutField,
								  simdOutColorX, simdOutColorY, simdOutColorZ);

			//Use computed field to get the normal
			fc.normal(simdPX, simdPY, simdPZ, simdOutField,
					  simdOutNormalX, simdOutNormalY, simdOutNormalZ, NORMAL_DELTA);

			//Store color. Convert from AOS to SOA
			SimdStoreVector(&lpMeshColors[szBlockOffset], simdOutColorX, simdOutColorY, simdOutColorZ);

			//Store normal. Convert from AOS to SOA
			SimdStoreVector(&lpMeshNormals[szBlockOffset], simdOutNormalX, simdOutNormalY, simdOutNormalZ);
		}

	}


}


int CMPUProcessor::getEdge(const EDGETABLE& edgeTable, int i, int j, int k, int edgeAxis) const
{
	//Hash Value of the low corner
	int hashval = CELLID_FROM_IDX(i, j, k);

	//Check how many edges are registered at that vertex
	if(edgeTable.ctEdges[hashval] > 0)
	{
		//The following line is simply x = 0, y = 1 or z = 2 since the difference is one at one direction only.
		if(edgeTable.hasEdgeWithHiNeighbor[hashval*3 + edgeAxis])
			return edgeTable.idxVertices[hashval*3 + edgeAxis];
	}
	return -1;
}

int CMPUProcessor::getEdge(const EDGETABLE& edgeTable , vec3i& start, vec3i& end) const
{
	int i1 = start.x;
	int j1 = start.y;
	int k1 = start.z;
	int i2 = end.x;
	int j2 = end.y;
	int k2 = end.z;

	if (i1>i2 || (i1==i2 && (j1>j2 || (j1==j2 && k1>k2)))) 
	{
		int t;
		t=i1; i1=i2; i2=t; 
		t=j1; j1=j2; j2=t; 
		t=k1; k1=k2; k2=t;
		start = vec3i(i1, j1, k1);
		end = vec3i(i2, j2, k2);
	}

	//Hash Value of the low corner
	int hashval = CELLID_FROM_IDX(i1, j1, k1);

	//Check how many edges are registered at that vertex
	if(edgeTable.ctEdges[hashval] > 0)
	{
		//The following line is simply x = 0, y = 1 or z = 2 since the difference is one at one direction only.
		int idxEdge = (((i2 - i1) + (j2 - j1)*2 + (k2 - k1)*4) >> 1);
		if(edgeTable.hasEdgeWithHiNeighbor[hashval*3 + idxEdge])
			return edgeTable.idxVertices[hashval*3 + idxEdge];
	}
	return -1;
}


void CMPUProcessor::setEdge(EDGETABLE& edgeTable, int i, int j, int k, int edgeAxis, int vid) const
{
	int hashval = CELLID_FROM_IDX(i, j, k);

	edgeTable.ctEdges[hashval]++;
	edgeTable.hasEdgeWithHiNeighbor[hashval*3 + edgeAxis] = 1;
	edgeTable.idxVertices[hashval*3 + edgeAxis] = vid;
}

void CMPUProcessor::setEdge(EDGETABLE& edgeTable, vec3i& start, vec3i& end, int vid) const
{
	int i1 = start.x;
	int j1 = start.y;
	int k1 = start.z;
	int i2 = end.x;
	int j2 = end.y;
	int k2 = end.z;

	if (i1>i2 || (i1==i2 && (j1>j2 || (j1==j2 && k1>k2)))) 
	{
		int t;
		t=i1; i1=i2; i2=t; 
		t=j1; j1=j2; j2=t; 
		t=k1; k1=k2; k2=t;
		start = vec3i(i1, j1, k1);
		end = vec3i(i2, j2, k2);
	}

	int hashval = CELLID_FROM_IDX(i1, j1, k1);

	edgeTable.ctEdges[hashval]++;

	int idxEdge = (((i2 - i1) + (j2 - j1)*2 + (k2 - k1)*4) >> 1);
	edgeTable.hasEdgeWithHiNeighbor[hashval*3 + idxEdge] = 1;
	edgeTable.idxVertices[hashval*3 + idxEdge] = vid;
}

/*
int CMPUProcessor::computeRootNewtonRaphson(const vec3f& p1, const vec3f& p2,
											float fp1, float fp2, 
											vec3f& output, float& outputField,
											float target_field, int iterations)
{
	vec3f grad, x, d;
	float f, g;

	if(iterations <= 0) return -1;	

	if(fabsf(fp1 - target_field) < fabsf(fp2 - target_field))
		x = p1;
	else
		x = p2;

	int i=0;
	for(i=0; i<iterations; i++)
	{
		//Get gradient for direction of displacement
		//grad = m_root->gradient(x, FIELD_VALUE_EPSILON);			
		//f = m_root->fieldValue(x);
		//Use faster method to compute fieldvalue and gradient at once
		grad = cptblobPrims->fieldValueAndGradient(x, FIELDVALUE_EPSILON);

		d.set(target_field - grad[3]);

		//Uses shrink-wrap method to converge to surface	
		g = 1.0f / (grad.dot(grad));
		x = x + (d*grad) * g;

		outputField = cptblobPrims->fieldvalue(x, lpStoreFVOps, lpStoreFVPrims);
		output = x;		
		if(fabsf(outputField - target_field) < FIELDVALUE_EPSILON)				
			break;				
	}

	output[3] = 0.0f;
	return (i+1)*4;
}
*/


////////////////////////////////////////////////////////////////////////////////////////////////
FieldComputer::FieldComputer(SOABlobPrims* lpPrims, SOABlobBoxMatrices* lpPrimMatrices, SOABlobOps* lpOps)
{
	setup(lpPrims, lpPrimMatrices, lpOps);
}

void FieldComputer::setup(SOABlobPrims* lpPrims,
		   	   	 	 	  SOABlobBoxMatrices* lpPrimMatrices,
		   	   	 	 	  SOABlobOps* lpOps)
{
	memcpy(&m_blobPrims, lpPrims, sizeof(SOABlobPrims));
	memcpy(&m_blobPrimMatrices, lpPrimMatrices, sizeof(SOABlobNodeMatrices));
	memcpy(&m_blobOps, lpOps, sizeof(SOABlobOps));
}

bool FieldComputer::isOutsideOp(int idxNode, const Float_& pX, const Float_& pY, const Float_& pZ) const
{
	Float_ one(1.0f);
	Float_ opBoxLoX = Float_(m_blobOps.bboxLoX[idxNode]);
	Float_ opBoxLoY = Float_(m_blobOps.bboxLoY[idxNode]);
	Float_ opBoxLoZ = Float_(m_blobOps.bboxLoZ[idxNode]);

	Float_ opBoxHiX = Float_(m_blobOps.bboxHiX[idxNode]);
	Float_ opBoxHiY = Float_(m_blobOps.bboxHiY[idxNode]);
	Float_ opBoxHiZ = Float_(m_blobOps.bboxHiZ[idxNode]);

	Float_ inside = SimdAnd(one, SimdAnd(SimdGTE(pX, opBoxLoX), SimdGTE(opBoxHiX, pX)));
	inside = SimdAnd(inside, SimdAnd(one, SimdAnd(SimdGTE(pY, opBoxLoY), SimdGTE(opBoxHiY, pY))));
	inside = SimdAnd(inside, SimdAnd(one, SimdAnd(SimdGTE(pZ, opBoxLoZ), SimdGTE(opBoxHiZ, pZ))));

	if(VecNMask(inside.v) == PS_SIMD_ALLZERO)
		return true;
	else
		return false;
}

bool FieldComputer::isOutsidePrim(int idxNode, const Float_& pX, const Float_& pY, const Float_& pZ) const
{
	Float_ one(1.0f);
	Float_ primBoxLoX = Float_(m_blobPrims.bboxLoX[idxNode]);
	Float_ primBoxLoY = Float_(m_blobPrims.bboxLoY[idxNode]);
	Float_ primBoxLoZ = Float_(m_blobPrims.bboxLoZ[idxNode]);

	Float_ primBoxHiX = Float_(m_blobPrims.bboxHiX[idxNode]);
	Float_ primBoxHiY = Float_(m_blobPrims.bboxHiY[idxNode]);
	Float_ primBoxHiZ = Float_(m_blobPrims.bboxHiZ[idxNode]);

	Float_ inside = SimdAnd(one, SimdAnd(SimdGTE(pX, primBoxLoX), SimdGTE(primBoxHiX, pX)));
	inside = SimdAnd(inside, SimdAnd(one, SimdAnd(SimdGTE(pY, primBoxLoY), SimdGTE(primBoxHiY, pY))));
	inside = SimdAnd(inside, SimdAnd(one, SimdAnd(SimdGTE(pZ, primBoxLoZ), SimdGTE(primBoxHiZ, pZ))));

	if(VecNMask(inside.v) == PS_SIMD_ALLZERO)
		return true;
	else
		return false;
}

int FieldComputer::getCrossedFaces(const float arrFields[8], int outFaces[6]) const
{
	//Left Face
	int state = (arrFields[0] >= ISO_VALUE) + (arrFields[1] >= ISO_VALUE) +
			(arrFields[2] >= ISO_VALUE) + (arrFields[3] >= ISO_VALUE);
	outFaces[L] = ((state != 4)&&(state != 0));

	//Right Face
	state = (arrFields[4] >= ISO_VALUE) + (arrFields[5] >= ISO_VALUE) +
			(arrFields[6] >= ISO_VALUE) + (arrFields[7] >= ISO_VALUE);
	outFaces[R] = ((state != 4)&&(state != 0));

	//Bottom Face
	state = (arrFields[0] >= ISO_VALUE) + (arrFields[1] >= ISO_VALUE) +
			(arrFields[4] >= ISO_VALUE) + (arrFields[5] >= ISO_VALUE);
	outFaces[B] = ((state != 4)&&(state != 0));

	//Top Face
	state = (arrFields[2] >= ISO_VALUE) + (arrFields[3] >= ISO_VALUE) +
			(arrFields[6] >= ISO_VALUE) + (arrFields[7] >= ISO_VALUE);
	outFaces[T] = ((state != 4)&&(state != 0));

	//Near Face
	state = (arrFields[0] >= ISO_VALUE) + (arrFields[2] >= ISO_VALUE) +
			(arrFields[4] >= ISO_VALUE) + (arrFields[6] >= ISO_VALUE);
	outFaces[N] = ((state != 4)&&(state != 0));

	//Far Face
	state = (arrFields[1] >= ISO_VALUE) + (arrFields[3] >= ISO_VALUE) +
			(arrFields[5] >= ISO_VALUE) + (arrFields[7] >= ISO_VALUE);
	outFaces[F] = ((state != 4)&&(state != 0));

	int sum = 0;
	for(int i=0; i<6;i++)
		sum += outFaces[i];
	return sum;
}

void FieldComputer::computePrimitiveField(const Float_& pX, const Float_& pY, const Float_& pZ,
										  Float_& primField, U32 idxPrimitive) const
{
	//If outside primitive then return
	if(isOutsidePrim(idxPrimitive, pX, pY, pZ))
	{
		primField.setZero();
		return;
	}

	//Transform point first
	Float_ dist2;
	Float_ ptX, ptY, ptZ;
	dist2.setZero();

	U32 idxMatrix = m_blobPrims.idxMatrix[idxPrimitive];
	if(idxMatrix == 0)
	{
		ptX = pX;
		ptY = pY;
		ptZ = pZ;
	}
	else
	{
		assert(idxMatrix < m_blobPrimMatrices.count);
		idxMatrix *= PRIM_MATRIX_STRIDE;

		Float_ matc1 = m_blobPrimMatrices.matrix[idxMatrix];
		Float_ matc2 = m_blobPrimMatrices.matrix[idxMatrix + 1];
		Float_ matc3 = m_blobPrimMatrices.matrix[idxMatrix + 2];
		Float_ matc4 = m_blobPrimMatrices.matrix[idxMatrix + 3];
		ptX = matc1 * pX + matc2 * pY + matc3 * pZ + matc4;

		matc1 = m_blobPrimMatrices.matrix[idxMatrix + 4];
		matc2 = m_blobPrimMatrices.matrix[idxMatrix + 5];
		matc3 = m_blobPrimMatrices.matrix[idxMatrix + 6];
		matc4 = m_blobPrimMatrices.matrix[idxMatrix + 7];
		ptY = matc1 * pX + matc2 * pY + matc3 * pZ + matc4;

		matc1 = m_blobPrimMatrices.matrix[idxMatrix + 8];
		matc2 = m_blobPrimMatrices.matrix[idxMatrix + 9];
		matc3 = m_blobPrimMatrices.matrix[idxMatrix + 10];
		matc4 = m_blobPrimMatrices.matrix[idxMatrix + 11];
		ptZ = matc1 * pX + matc2 * pY + matc3 * pZ + matc4;
	}


	switch(m_blobPrims.type[idxPrimitive])
	{
	case(primPoint):
	{
		//this is just for a sphere
		Float_ distX = Float_(m_blobPrims.posX[idxPrimitive]) - ptX;
		Float_ distY = Float_(m_blobPrims.posY[idxPrimitive]) - ptY;
		Float_ distZ = Float_(m_blobPrims.posZ[idxPrimitive]) - ptZ;
		dist2 = (distX * distX) + (distY * distY) + (distZ * distZ);
	}
	break;
	case(primLine):
	{
		Float_ line0X = Float_(m_blobPrims.posX[idxPrimitive]);
		Float_ line0Y = Float_(m_blobPrims.posY[idxPrimitive]);
		Float_ line0Z = Float_(m_blobPrims.posZ[idxPrimitive]);

		Float_ lineDeltaX = Float_(m_blobPrims.dirX[idxPrimitive]) - line0X;
		Float_ lineDeltaY = Float_(m_blobPrims.dirY[idxPrimitive]) - line0Y;
		Float_ lineDeltaZ = Float_(m_blobPrims.dirZ[idxPrimitive]) - line0Z;
		Float_ lineDeltaDot = lineDeltaX * lineDeltaX + lineDeltaY * lineDeltaY + lineDeltaZ * lineDeltaZ;

		Float_ distX = ptX - line0X;
		Float_ distY = ptY - line0Y;
		Float_ distZ = ptZ - line0Z;


		Float_ delta = distX * lineDeltaX + distY * lineDeltaY + distZ * lineDeltaZ;
		delta = delta / lineDeltaDot;

		//Nearest Point
		distX = ptX - (line0X + delta * lineDeltaX);
		distY = ptY - (line0Y + delta * lineDeltaY);
		distZ = ptZ - (line0Z + delta * lineDeltaZ);

		//Distance to nearest Point
		dist2 = (distX * distX) + (distY * distY) + (distZ * distZ);
	}
	break;
	case(primCylinder):
	{
		Float_ posX = ptX - Float_(m_blobPrims.posX[idxPrimitive]);
		Float_ posY = ptY - Float_(m_blobPrims.posY[idxPrimitive]);
		Float_ posZ = ptZ - Float_(m_blobPrims.posZ[idxPrimitive]);


		Float_ dirX = Float_(m_blobPrims.dirX[idxPrimitive]);
		Float_ dirY = Float_(m_blobPrims.dirY[idxPrimitive]);
		Float_ dirZ = Float_(m_blobPrims.dirZ[idxPrimitive]);

		Float_ radius = Float_(m_blobPrims.resX[idxPrimitive]);
		Float_ height = Float_(m_blobPrims.resY[idxPrimitive]);

		Float_ zero(0.0f);
		Float_ one(1.0f);
		Float_ y = posX * dirX + posY * dirY + posZ * dirZ;
		//Float_ x = SimdMax(zero, one / (SimdAccurateRSqrt(posX * posX + posY * posY + posZ * posZ - y*y)) - radius);
		Float_ x = SimdMax(zero, SimdSqrt(posX * posX + posY * posY + posZ * posZ - y*y) - radius);

		Float_ mask = SimdGT(y, zero);
		mask = SimdAnd(mask, one);

		y = mask * SimdMax(zero, y - height) + (one - mask) * y;

		dist2 = x*x + y*y;
	}
	break;
	case(primTriangle):
	{
		Float_ tri0X = Float_(m_blobPrims.posX[idxPrimitive]);
		Float_ tri0Y = Float_(m_blobPrims.posY[idxPrimitive]);
		Float_ tri0Z = Float_(m_blobPrims.posZ[idxPrimitive]);

		Float_ tri1X = Float_(m_blobPrims.dirX[idxPrimitive]);
		Float_ tri1Y = Float_(m_blobPrims.dirY[idxPrimitive]);
		Float_ tri1Z = Float_(m_blobPrims.dirZ[idxPrimitive]);

		Float_ tri2X = Float_(m_blobPrims.resX[idxPrimitive]);
		Float_ tri2Y = Float_(m_blobPrims.resY[idxPrimitive]);
		Float_ tri2Z = Float_(m_blobPrims.resZ[idxPrimitive]);

		//Compute Triangle distance
		dist2 = FLT_MAX;
	}
	break;

	case(primCube):
	{
		Float_ posX = Float_(m_blobPrims.posX[idxPrimitive]);
		Float_ posY = Float_(m_blobPrims.posY[idxPrimitive]);
		Float_ posZ = Float_(m_blobPrims.posZ[idxPrimitive]);
		Float_ side = Float_(m_blobPrims.resX[idxPrimitive]);
		Float_ minusSide = Float_(-1.0f * m_blobPrims.resX[idxPrimitive]);

		Float_ difX = ptX - posX;
		Float_ difY = ptY - posY;
		Float_ difZ = ptZ - posZ;

		Float_ one(1.0f);

		//ALONG X
		Float_ maskMinus = SimdGT(minusSide, difX);
		Float_ maskPositive = SimdGT(difX, side);
		maskMinus = SimdAnd(maskMinus, one);
		maskPositive = SimdAnd(maskPositive, one);
		Float_ delta = (difX + side) * maskMinus + (difX - side) * maskPositive;
		dist2 = delta * delta;

		//ALONG Y
		maskMinus = SimdGT(minusSide, difY);
		maskPositive = SimdGT(difY, side);
		maskMinus = SimdAnd(maskMinus, one);
		maskPositive = SimdAnd(maskPositive, one);
		delta = (difY + side) * maskMinus + (difY - side) * maskPositive;
		dist2 += delta * delta;

		//ALONG Z
		maskMinus = SimdGT(minusSide, difZ);
		maskPositive = SimdGT(difZ, side);
		maskMinus = SimdAnd(maskMinus, one);
		maskPositive = SimdAnd(maskPositive, one);
		delta = (difZ + side) * maskMinus + (difZ - side) * maskPositive;
		dist2 += delta * delta;
	}
	break;

	case(primDisc):
	{
		/*
        vec3f n = prims[id].dir.xyz();
        vec3f c = prims[id].pos.xyz();
        float r = prims[id].res1[0];
        vec3f dir = pn - c - (n.dot(pn - c))*n;

        //Check if Q lies on center or p is just above center
        if(dir.length() <= r)
        {
            dd = Absolutef((pn - c).length2() - dir.length2());
        }
        else
        {
            dir.normalize();
            vec3f x = c + r * dir;
            dd = (x - pn).length2();
        }
        */


		Float_ dX = ptX - Float_(m_blobPrims.posX[idxPrimitive]);
		Float_ dY = ptY - Float_(m_blobPrims.posY[idxPrimitive]);
		Float_ dZ = ptZ - Float_(m_blobPrims.posZ[idxPrimitive]);

		Float_ nX = Float_(m_blobPrims.dirX[idxPrimitive]);
		Float_ nY = Float_(m_blobPrims.dirY[idxPrimitive]);
		Float_ nZ = Float_(m_blobPrims.dirZ[idxPrimitive]);
		Float_ radius = Float_(m_blobPrims.resX[idxPrimitive]);

		Float_ dot = nX * dX + nY * dY + nZ * dZ;
		Float_ dirX = dX - nX * dot;
		Float_ dirY = dY - nY * dot;
		Float_ dirZ = dZ - nZ * dot;
		dot = dirX * dirX + dirY * dirY + dirZ * dirZ;

		//Need Accurate Square Root
		Float_ rsdot = SimdAccurateRSqrt(dot);
		dirX = dirX * rsdot;
		dirY = dirY * rsdot;
		dirZ = dirZ * rsdot;

		nX = radius * dirX - dX;
		nY = radius * dirY - dY;
		nZ = radius * dirZ - dZ;

		//Float_ mask = SimdGTE(radius, SimdSqrt(dot));
		Float_ mask = SimdGTE(radius * radius, dot);
		Float_ one(1.0f);
		mask = SimdAnd(mask, one);

		dist2 =  mask * SimdAbs(dX * dX + dY * dY + dZ * dZ - dot) +
				(one - mask) * (nX * nX + nY * nY + nZ * nZ);

	}
	break;

	case(primRing):
	{
		/*
		vec3f c = vec3f(m_blobPrims.posX[idxPrimitive],
						m_blobPrims.posY[idxPrimitive],
						m_blobPrims.posZ[idxPrimitive]);

		vec3f n = vec3f(m_blobPrims.dirX[idxPrimitive],
						m_blobPrims.dirY[idxPrimitive],
						m_blobPrims.dirZ[idxPrimitive]);

		float r = m_blobPrims.resX[idxPrimitive];

		for(int i=0;i<PS_SIMD_FLEN; i++)
		{
			float dd = 0.0f;

			vec3f pn = vec3f(ptX[i], ptY[i], ptZ[i]);
			vec3f pnc = vsub3f(pn, c);

			vec3f dir =  vsub3f(pnc, vscale3f(vdot3f(n, pnc), n));

			//Check if Q lies on center or p is just above center
			if(vlengthSquared3f(dir) == 0.0f)
			{
				//r^2 + |p-c|^2
				dd = r*r + vlengthSquared3f(pnc);
			}
			else
			{
				vnormalize3f(dir);

				vec3f x = vadd3f(c, vscale3f(r, dir));
				dd = vlengthSquared3f(vsub3f(x, pn));
			}
			dist2[i] = dd;
		}
		*/


		Float_ dX = ptX - Float_(m_blobPrims.posX[idxPrimitive]);
		Float_ dY = ptY - Float_(m_blobPrims.posY[idxPrimitive]);
		Float_ dZ = ptZ - Float_(m_blobPrims.posZ[idxPrimitive]);

		Float_ nX = Float_(m_blobPrims.dirX[idxPrimitive]);
		Float_ nY = Float_(m_blobPrims.dirY[idxPrimitive]);
		Float_ nZ = Float_(m_blobPrims.dirZ[idxPrimitive]);
		Float_ radius = Float_(m_blobPrims.resX[idxPrimitive]);

		Float_ dot = nX * dX + nY * dY + nZ * dZ;
		Float_ dirX = dX - nX * dot;
		Float_ dirY = dY - nY * dot;
		Float_ dirZ = dZ - nZ * dot;
		dot = dirX * dirX + dirY * dirY + dirZ * dirZ;

		Float_ zero(0.0f);
		Float_ one(1.0f);

		Float_ mask = SimdEQ(dot, zero);
		mask = SimdAnd(mask, one);

		//Normalize Dir for (one - mask) case
		//Need Accurate Square Root
		dot = SimdAccurateRSqrt(dot);
		dirX = dirX * dot;
		dirY = dirY * dot;
		dirZ = dirZ * dot;


		nX = radius * dirX - dX;
		nY = radius * dirY - dY;
		nZ = radius * dirZ - dZ;

		//dist2 = mask * (radius * radius + dX * dX + dY * dY + dZ * dZ);

		dist2 = mask * (radius * radius + dX * dX + dY * dY + dZ * dZ) +
				(one - mask) * (nX * nX + nY * nY + nZ * nZ);
	}
	break;
	case(primQuadricPoint):
	{
		Float_ dX = ptX - Float_(m_blobPrims.posX[idxPrimitive]);
		Float_ dY = ptY - Float_(m_blobPrims.posY[idxPrimitive]);
		Float_ dZ = ptZ - Float_(m_blobPrims.posZ[idxPrimitive]);

		Float_ rs = Float_(m_blobPrims.dirZ[idxPrimitive]);
		Float_ coeff1 = Float_(m_blobPrims.resX[idxPrimitive]);
		Float_ coeff2 = Float_(m_blobPrims.resY[idxPrimitive]);
		Float_ coeff3 = Float_(m_blobPrims.resZ[idxPrimitive]);

		dist2 = dX * dX + dY * dY + dZ * dZ;
		Float_ one(1.0f);
		Float_ mask = SimdAnd(SimdGT(rs, dist2), one);
		primField = mask * (dist2*dist2* coeff1 + dist2 * coeff2 + coeff3);
		return;
		//dist2 = mask * (dist2*dist2* coeff1 + dist2 * coeff2 + coeff3);
	}
	break;

	case(primNULL):
	{
		dist2 = Float_(10.0f);
	}
	break;
	case(primInstance):
	{
		//NodeIndex in the array
		// X = idxArray, Y = idxScript, Z = isOriginOp, W = dirX = NodeType
		U32 idxOrigin  = static_cast<U32>(m_blobPrims.resX[idxPrimitive]);
		U32 isOriginOp = static_cast<U32>(m_blobPrims.resZ[idxPrimitive]);
		U8 originType  = static_cast<U8>(m_blobPrims.dirX[idxPrimitive]);

		if(isOriginOp)
		{
			//If all points are outside the instanced node then return
			if(isOutsideOp(idxOrigin, ptX, ptY, ptZ))
				primField.setZero();
			else
			{
				assert(originType == m_blobOps.type[idxOrigin]);
				fieldValue(ptX, ptY, ptZ, primField, idxOrigin);
			}
		}
		else
			computePrimitiveField(ptX, ptY, ptZ, primField, idxOrigin);
		return;
	}
	break;

	}

	//Compute Output Field for primitive
	ComputeWyvillFieldValueSquare_(dist2, primField);
}


//For primitives check if the query point is inside BBOX
//Pass an array to get all fields due to all primitives for field and color function
int FieldComputer::fieldValue(const Float_& pX, const Float_& pY, const Float_& pZ,
							  Float_& outField, U32 idxRootNode,
							  float* lpOutFieldPrims, float* lpOutFieldOps) const
{
	outField.setZero();
	float PS_SIMD_ALIGN(arrPrimFields[PS_SIMD_PADSIZE(m_szConstFieldPadded)]);
	float PS_SIMD_ALIGN(arrOpFields[PS_SIMD_PADSIZE(m_szConstFieldPadded)]);

	//Evaluate Operators
	if(m_blobOps.count > 0)
	{
		outField.setZero();

		//NO VLA
		//U8 arrOpsFieldComputed[m_blobOps.ctOps];
		//memset(arrOpsFieldComputed, 0, m_blobOps.ctOps);
		U8 arrOpsFieldComputed[MAX_TREE_NODES];
		memset(arrOpsFieldComputed, 0, MAX_TREE_NODES);

		SIMPLESTACK<MAX_TREE_NODES> stkOps;
		stkOps.push(idxRootNode);

		U16 idxLC, idxRC;
		U8 childKind, opType;
		bool bLeftChildOp, bRightChildOp, bReady, bRange, bUnary;

		//Process All
		while(!stkOps.empty())
		{
			idxRootNode = stkOps.top();
			U16 opType =

			idxLC = m_blobOps.opLeftChild[idxRootNode];
			idxRC = m_blobOps.opRightChild[idxRootNode];
			childKind = m_blobOps.opFlags[idxRootNode];
			bUnary	   = (childKind & ofIsUnaryOp) >> 3;
			bRange 	   = (childKind & ofChildIndexIsRange) >> 2;
			bLeftChildOp = (childKind & ofLeftChildIsOp) >> 1;
			bRightChildOp = (childKind & ofRightChildIsOp);

			//Here process if we need to go down the tree
			/*
			if((bRange)||(depth > 3))
			{
				if(isOutsideOp(idxNode, pX, pY, pZ))
				{
					stkOps.pop();
					outField.store(&arrOpFields[idxNode * PS_SIMD_FLEN]);
					arrOpsFieldComputed[idxNode] = 1;
					continue;
				}
			}
			*/

			//Range
			if(bRange)
			{
				stkOps.pop();

				Float_ curField;
				for(U16 i=idxLC; i<=idxRC; i++)
				{
					computePrimitiveField(pX, pY, pZ, curField, i);
					curField.store(&arrPrimFields[i * PS_SIMD_FLEN]);
					outField = outField + curField;
				}

				//Store Op Field
				outField.store(&arrOpFields[idxRootNode * PS_SIMD_FLEN]);
				arrOpsFieldComputed[idxRootNode] = 1;
			}
			else
			{
				if(bUnary)
					bReady = !(bLeftChildOp && (arrOpsFieldComputed[idxLC] == 0));
				else
					bReady = !((bLeftChildOp && (arrOpsFieldComputed[idxLC] == 0))||
							   (bRightChildOp && (arrOpsFieldComputed[idxRC] == 0)));
				if(bReady)
				{
					stkOps.pop();

					Float_ leftChildField;
					Float_ rightChildField;

					if(bLeftChildOp)
						leftChildField = Float_(&arrOpFields[idxLC * PS_SIMD_FLEN]);
					else
					{
						computePrimitiveField(pX, pY, pZ, leftChildField, idxLC);
						leftChildField.store(&arrPrimFields[idxLC * PS_SIMD_FLEN]);
					}

					if(!bUnary)
					{
						if(bRightChildOp)
							rightChildField = Float_(&arrOpFields[idxRC * PS_SIMD_FLEN]);
						else
						{
							computePrimitiveField(pX, pY, pZ, rightChildField, idxRC);
							rightChildField.store(&arrPrimFields[idxRC * PS_SIMD_FLEN]);
						}
					}

					opType = m_blobOps.type[idxRootNode];
					switch (opType) {
					case (opBlend): {
						outField = leftChildField + rightChildField;
					}
					break;
					case (opRicciBlend):
					{
						Float_ rp = Float_(m_blobOps.resX[idxRootNode]);
						Float_ rpRcp = Float_(m_blobOps.resY[idxRootNode]);
						outField = SimdPower(SimdPower(leftChildField, rp) + SimdPower(rightChildField, rp), rpRcp);
					}
					break;
					case (opUnion): {
						outField = SimdMax(leftChildField, rightChildField);
					}
					break;
					case (opIntersect): {
						outField = SimdMin(leftChildField, rightChildField);
					}
					break;
					case (opDif): {
						Float_ maxField(1.0f);
						outField = SimdMin(leftChildField, maxField - rightChildField);
					}
					break;
					case (opSmoothDif): {
						Float_ maxField(1.0f);
						outField = leftChildField * (maxField - rightChildField);
					}
					break;
					case (opWarpBend): {
						outField = leftChildField;
					}
					break;
					case (opWarpTwist): {
						outField = leftChildField;
					}
						break;
					case (opWarpTaper): {
						outField = leftChildField;
					}
						break;
					case (opWarpShear): {
						outField = leftChildField;
					}
						break;

					}

					//Store Op Field
					outField.store(&arrOpFields[idxRootNode * PS_SIMD_FLEN]);
					arrOpsFieldComputed[idxRootNode] = 1;
				}
				else
				{
					//If lchild is op and not processed
					if(bLeftChildOp && (arrOpsFieldComputed[idxLC] == 0))
						stkOps.push(idxLC);
						//stkOps.push(depth + 1, idxLC);

					//If rchild is op and not processed
					if(bRightChildOp && (arrOpsFieldComputed[idxRC] == 0))
						stkOps.push(idxRC);
						//stkOps.push(depth + 1, idxRC);
				}//End if bReady
			}//End if bRange
		}
	}
	else
	{
		//NO Op - Just compute all primitive fields and Blend them
		Float_ curPrimField;
		for(U32 i=0; i<m_blobPrims.count; i++)
		{
			computePrimitiveField(pX, pY, pZ, curPrimField, i);
			outField = outField + curPrimField;

			//Store Fields
			curPrimField.store(&arrPrimFields[i * PS_SIMD_FLEN]);
		}
	}

	if(lpOutFieldPrims)
		memcpy(lpOutFieldPrims, arrPrimFields, m_szConstFieldPadded * sizeof(float));
		//memcpy(lpOutFieldPrims, arrPrimFields, szPrimFieldArrSize * sizeof(float));
	if(lpOutFieldOps)
		memcpy(lpOutFieldOps, arrOpFields, m_szConstFieldPadded * sizeof(float));
		//memcpy(lpOutFieldOps, arrOpFields, szOpFieldArrSize * sizeof(float));

	return 1;
}

int FieldComputer::fieldValueAndColor(const Float_& pX, const Float_& pY, const Float_& pZ,
									  Float_& outField,	Float_& outColorX,
									  Float_& outColorY, Float_& outColorZ, U32 idxRootNode) const
{
	//const U32 szPrimFieldArrSize = PS_SIMD_PADSIZE(m_blobPrims.ctPrims * PS_SIMD_FLEN);
	//const U32 szOpFieldArrSize   = PS_SIMD_PADSIZE(m_blobOps.ctOps * PS_SIMD_FLEN);

	float PS_SIMD_ALIGN(arrPrimFields[m_szConstFieldPadded]);
	float PS_SIMD_ALIGN(arrOpFields[m_szConstFieldPadded]);

	outColorX.setZero();
	outColorY.setZero();
	outColorZ.setZero();

	//The only Field Evaluation we do
	fieldValue(pX, pY, pZ, outField, idxRootNode, arrPrimFields, arrOpFields);

	//Evaluate Operators
	if(m_blobOps.count > 0)
	{
		float PS_SIMD_ALIGN(arrOpColorX[m_szConstFieldPadded]);
		float PS_SIMD_ALIGN(arrOpColorY[m_szConstFieldPadded]);
		float PS_SIMD_ALIGN(arrOpColorZ[m_szConstFieldPadded]);

		U8 arrOpsColorComputed[MAX_TREE_NODES];
		memset(arrOpsColorComputed, 0, MAX_TREE_NODES);

		//Stack for color computation
		SIMPLESTACK<MAX_TREE_NODES> stkOps;
		stkOps.push(idxRootNode);

		U16 idxNode, idxLC, idxRC;
		U8 childKind, opType;
		bool bLCIsOp, bRCIsOp, bReady, bRange;

		while(!stkOps.empty())
		{
			idxNode = stkOps.top();
			idxLC = m_blobOps.opLeftChild[idxNode];
			idxRC = m_blobOps.opRightChild[idxNode];
			childKind = m_blobOps.opFlags[idxNode];
			bRange 	   = (childKind & 4) >> 2;
			bLCIsOp = (childKind & 2) >> 1;
			bRCIsOp = (childKind & 1);

			if(bRange)
			{
				stkOps.pop();

				Float_ curField;
				Float_ half(0.5f);
				Float_ one(1.0f);
				Float_ two(2.0f);

				for(U16 i = idxLC; i <= idxRC; i++)
				{
					curField = two * (half + Float_(&arrPrimFields[i * PS_SIMD_FLEN])) - one;
					outColorX = outColorX + curField * Float_(m_blobPrims.colorX[i]);
					outColorY = outColorY + curField * Float_(m_blobPrims.colorY[i]);
					outColorZ = outColorZ + curField * Float_(m_blobPrims.colorZ[i]);
				}

				//Set Compute Flag and store fields
				arrOpsColorComputed[idxNode] = 1;
				outColorX.store(&arrOpColorX[idxNode * PS_SIMD_FLEN]);
				outColorY.store(&arrOpColorY[idxNode * PS_SIMD_FLEN]);
				outColorZ.store(&arrOpColorZ[idxNode * PS_SIMD_FLEN]);
			}
			else
			{
				bReady = !((bLCIsOp && (arrOpsColorComputed[idxLC] == 0))||
						(bRCIsOp && (arrOpsColorComputed[idxRC] == 0)));
				if(bReady)
				{
					stkOps.pop();

					//VARS
					Float_ curOpField;
					Float_ leftChildField;
					Float_ rightChildField;
					Float_ leftChildColorX;
					Float_ leftChildColorY;
					Float_ leftChildColorZ;
					Float_ rightChildColorX;
					Float_ rightChildColorY;
					Float_ rightChildColorZ;


					//
					curOpField = Float_(&arrOpFields[idxNode * PS_SIMD_FLEN]);

					//Left Child
					if(bLCIsOp)
					{
						leftChildField = Float_(&arrOpFields[idxLC * PS_SIMD_FLEN]);
						leftChildColorX = arrOpColorX[idxLC * PS_SIMD_FLEN];
						leftChildColorY = arrOpColorY[idxLC * PS_SIMD_FLEN];
						leftChildColorZ = arrOpColorZ[idxLC * PS_SIMD_FLEN];
					}
					else
					{
						//Compute for instanced node
						if(m_blobPrims.type[idxLC] == primInstance)
						{
							computeInstancedNodeFieldAndColor(pX, pY, pZ, leftChildField,
															  leftChildColorX, leftChildColorY, leftChildColorZ, idxLC);
						}
						else
						{
							leftChildField = Float_(&arrPrimFields[idxLC * PS_SIMD_FLEN]);
							leftChildColorX = Float_(m_blobPrims.colorX[idxLC]);
							leftChildColorY = Float_(m_blobPrims.colorY[idxLC]);
							leftChildColorZ = Float_(m_blobPrims.colorZ[idxLC]);
						}
					}


					//Right Child
					if(bRCIsOp)
					{
						rightChildField  = Float_(&arrOpFields[idxRC * PS_SIMD_FLEN]);
						rightChildColorX = arrOpColorX[idxRC * PS_SIMD_FLEN];
						rightChildColorY = arrOpColorY[idxRC * PS_SIMD_FLEN];
						rightChildColorZ = arrOpColorZ[idxRC * PS_SIMD_FLEN];
					}
					else
					{
						//Compute for instanced node
						if(m_blobPrims.type[idxRC] == primInstance)
						{
							computeInstancedNodeFieldAndColor(pX, pY, pZ, rightChildField,
															  rightChildColorX, rightChildColorY, rightChildColorZ, idxRC);

						}
						else
						{
							rightChildField = Float_(&arrPrimFields[idxRC * PS_SIMD_FLEN]);
							rightChildColorX = Float_(m_blobPrims.colorX[idxRC]);
							rightChildColorY = Float_(m_blobPrims.colorY[idxRC]);
							rightChildColorZ = Float_(m_blobPrims.colorZ[idxRC]);
						}
					}


					//Process Children
					opType = m_blobOps.type[idxNode];
					switch(opType)
					{
					case(opBlend): case(opRicciBlend):
						{
						Float_ half(0.5f);
						Float_ one(1.0f);
						Float_ two(2.0f);

						Float_ lcf  = two * (half + leftChildField) - one;
						Float_ rcf  = two * (half + rightChildField) - one;

						outColorX = lcf * leftChildColorX + rcf * rightChildColorX;
						outColorY = lcf * leftChildColorY + rcf * rightChildColorY;
						outColorZ = lcf * leftChildColorZ + rcf * rightChildColorZ;

						}
					break;

					case(opUnion):
						{
						Float_ half(0.5f);
						Float_ one(1.0f);
						Float_ two(2.0f);

						Float_ lcf  = two * (half + leftChildField) - one;
						Float_ rcf  = two * (half + rightChildField) - one;

						outColorX = SimdMax(lcf * leftChildColorX, rcf * rightChildColorX);
						outColorY = SimdMax(lcf * leftChildColorY, rcf * rightChildColorY);
						outColorZ = SimdMax(lcf * leftChildColorZ, rcf * rightChildColorZ);
						}
					break;

					case(opIntersect):
					{
						Float_ half(0.5f);
						Float_ one(1.0f);
						Float_ two(2.0f);

						leftChildField  = two * (half + leftChildField) - one;
						rightChildField = two * (half + rightChildField) - one;
						outColorX = SimdMin(leftChildField * leftChildColorX, rightChildField * rightChildColorX);
						outColorY = SimdMin(leftChildField * leftChildColorY, rightChildField * rightChildColorY);
						outColorZ = SimdMin(leftChildField * leftChildColorZ, rightChildField * rightChildColorZ);
					}
					break;

					case(opDif): case(opSmoothDif):
						{
						Float_ one(1.0f);

						leftChildField  = SimdAnd(SimdEQ(leftChildField, curOpField), one);
						rightChildField = SimdAnd(SimdEQ(one - rightChildField, curOpField), one);

						outColorX = leftChildField * leftChildColorX + rightChildField * rightChildColorX;
						outColorY = leftChildField * leftChildColorY + rightChildField * rightChildColorY;
						outColorZ = leftChildField * leftChildColorZ + rightChildField * rightChildColorZ;
						}
					break;

					case(opWarpBend):case(opWarpTwist):case(opWarpTaper):case(opWarpShear):
						{
						outColorX = leftChildColorX;
						outColorY = leftChildColorY;
						outColorZ = leftChildColorZ;
						}
					break;
					}

					//Set Compute Flag and store fields
					arrOpsColorComputed[idxNode] = 1;
					outColorX.store(&arrOpColorX[idxNode * PS_SIMD_FLEN]);
					outColorY.store(&arrOpColorY[idxNode * PS_SIMD_FLEN]);
					outColorZ.store(&arrOpColorZ[idxNode * PS_SIMD_FLEN]);

				}
				else
				{
					//If lchild is op and not processed
					if(bLCIsOp && (arrOpsColorComputed[idxLC] == 0))
						stkOps.push(idxLC);

					//If rchild is op and not processed
					if(bRCIsOp && (arrOpsColorComputed[idxRC] == 0))
						stkOps.push(idxRC);
				}//ENDIF bReady
			}//ENDIF bRange
		}
	}
	else
	{
		outColorX = Float_(m_blobPrims.colorX[0]);
		outColorY = Float_(m_blobPrims.colorY[0]);
		outColorZ = Float_(m_blobPrims.colorZ[0]);
	}


	return 1;
}

void FieldComputer::computeInstancedNodeFieldAndColor(const Float_& pX, const Float_& pY, const Float_& pZ, Float_& outField,
									   	   	   	   	  Float_& outColorX, Float_& outColorY, Float_& outColorZ, U32 idxInstanceNode) const
{
	assert(m_blobPrims.type[idxInstanceNode] == primInstance);

	//Transformed Point
	Float_ ptX, ptY, ptZ;

	//Get Original Index
	U32 idxOrigin = m_blobPrims.resX[idxInstanceNode];
	U32 isOriginOp = m_blobPrims.resZ[idxInstanceNode];
	U32 idxMatrix = m_blobPrims.idxMatrix[idxInstanceNode];
	if(idxMatrix == 0)
	{
		ptX = pX;
		ptY = pY;
		ptZ = pZ;
	}
	else
	{
		assert(idxMatrix < m_blobPrimMatrices.count);
		idxMatrix *= PRIM_MATRIX_STRIDE;

		Float_ matc1 = m_blobPrimMatrices.matrix[idxMatrix];
		Float_ matc2 = m_blobPrimMatrices.matrix[idxMatrix + 1];
		Float_ matc3 = m_blobPrimMatrices.matrix[idxMatrix + 2];
		Float_ matc4 = m_blobPrimMatrices.matrix[idxMatrix + 3];
		ptX = matc1 * pX + matc2 * pY + matc3 * pZ + matc4;

		matc1 = m_blobPrimMatrices.matrix[idxMatrix + 4];
		matc2 = m_blobPrimMatrices.matrix[idxMatrix + 5];
		matc3 = m_blobPrimMatrices.matrix[idxMatrix + 6];
		matc4 = m_blobPrimMatrices.matrix[idxMatrix + 7];
		ptY = matc1 * pX + matc2 * pY + matc3 * pZ + matc4;

		matc1 = m_blobPrimMatrices.matrix[idxMatrix + 8];
		matc2 = m_blobPrimMatrices.matrix[idxMatrix + 9];
		matc3 = m_blobPrimMatrices.matrix[idxMatrix + 10];
		matc4 = m_blobPrimMatrices.matrix[idxMatrix + 11];
		ptZ = matc1 * pX + matc2 * pY + matc3 * pZ + matc4;
	}

	if(isOriginOp)
	{
		//FieldValue and Color
		fieldValueAndColor(ptX, ptY, ptZ, outField, outColorX, outColorY, outColorZ, idxOrigin);
	}
	else
	{
		computePrimitiveField(ptX, ptY, ptZ, outField, idxOrigin);
		outColorX = Float_(m_blobPrims.colorX[idxOrigin]);
		outColorY = Float_(m_blobPrims.colorY[idxOrigin]);
		outColorZ = Float_(m_blobPrims.colorZ[idxOrigin]);
	}
}

int FieldComputer::fieldValueAndGradient(const Float_& pX, const Float_& pY, const Float_& pZ,
			  						     Float_& outField, Float_& outGradX, Float_& outGradY, Float_& outGradZ,
										 float delta) const
{
	fieldValue(pX, pY, pZ, outField);

	Float_ delta_(delta);
	Float_ deltaInv_(1.0f / delta);
	Float_ divX;
	Float_ divY;
	Float_ divZ;

	fieldValue(pX + delta, pY, pZ, divX);
	fieldValue(pX, pY + delta, pZ, divY);
	fieldValue(pX, pY, pZ + delta, divZ);

	outGradX = (divX - outField) * deltaInv_;
	outGradY = (divY - outField) * deltaInv_;
	outGradZ = (divZ - outField) * deltaInv_;

	return 4;
}

int FieldComputer::gradient(const Float_& pX, const Float_& pY, const Float_& pZ,
							 const Float_& inFieldValue,
							 Float_& outGradX, Float_& outGradY, Float_& outGradZ,
							 float delta) const
{
	Float_ delta_(delta);
	Float_ deltaInv_(1.0f / delta);
	Float_ divX;
	Float_ divY;
	Float_ divZ;

	fieldValue(pX + delta_, pY, pZ, divX);
	fieldValue(pX, pY + delta_, pZ, divY);
	fieldValue(pX, pY, pZ + delta, divZ);

	outGradX = (divX - inFieldValue) * deltaInv_;
	outGradY = (divY - inFieldValue) * deltaInv_;
	outGradZ = (divZ - inFieldValue) * deltaInv_;

	return 3;
}

int FieldComputer::normal(const Float_& pX,
						  const Float_& pY,
						  const Float_& pZ,
						  const Float_& inFieldValue,
						  Float_& outNormalX, Float_& outNormalY, Float_& outNormalZ,
						  float delta) const
{
	Float_ delta_(delta);
	Float_ deltaInv_(-1.0f / delta);
	Float_ divX;
	Float_ divY;
	Float_ divZ;

	fieldValue(pX + delta_, pY, pZ, divX);
	fieldValue(pX, pY + delta_, pZ, divY);
	fieldValue(pX, pY, pZ + delta, divZ);

	outNormalX = (divX - inFieldValue) * deltaInv_;
	outNormalY = (divY - inFieldValue) * deltaInv_;
	outNormalZ = (divZ - inFieldValue) * deltaInv_;

	SimdNormalize(outNormalX, outNormalY, outNormalZ);

	return 3;
}

bool FieldComputer::computeRootSimd(const Float_& pX, const Float_& pY, const Float_& pZ, vec3f& outRoot) const
{
	outRoot = vec3f(0.0f, 0.0f, 0.0f);

	Float_ F;
	Float_ one(1.0f);
	Float_ arrIsoVal(ISO_VALUE);

	//Compute Field
	this->fieldValue(pX, pY, pZ, F);
	Float_ inside = SimdAnd(SimdGTE(F, arrIsoVal), one);
	VecNMask mask(inside.v);
	if((mask == PS_SIMD_ALLZERO)||(mask == VecNMask(one.v)))
		return false;

	//Find the interval that contains the root
	U32 state = mask.u.m[0];
	int interval = 0;
	for(int i=1; i<PS_SIMD_FLEN; i++)
	{
		interval = i;
		if(mask.u.m[i] != state)
			break;
	}

	//Update e1 and e2 for the high resolution field
	vec3f e1 = vec3f(pX[interval - 1], pY[interval - 1], pZ[interval - 1]);
	vec3f e2 = vec3f(pX[interval], pY[interval], pZ[interval]);

	float scale = (ISO_VALUE - F[interval - 1])/(F[interval] - F[interval-1]);
	outRoot = vec3f::add(e1, vec3f::mul(scale, vec3f::sub(e2, e1)));
	return true;
}

int FieldComputer::computeRootNewtonRaphson(const vec3f& p1, const vec3f& p2,
											float fp1, float fp2,
											vec3f& output, float& outputField,
											float target_field, int iterations) const
{
	vec3f grad;
	Float_ rootX, rootY, rootZ;
	Float_ gradX, gradY, gradZ;
	Float_ field;
	//Float_ d;
	Float_ g;


	if(iterations <= 0) return -1;

	if(Absolutef(fp1 - target_field) < Absolutef(fp2 - target_field))
	{
		rootX = Float_(p1.x);
		rootY = Float_(p1.y);
		rootZ = Float_(p1.z);
		field = Float_(fp1);
	}
	else
	{
		rootX = Float_(p2.x);
		rootY = Float_(p2.y);
		rootZ = Float_(p2.z);
		field = Float_(fp2);
	}

	int i=0;
	for(i=0; i<iterations; i++)
	{
		//Use faster method to compute gradient at once
		gradient(rootX, rootY, rootZ, field, gradX, gradY, gradZ, FIELDVALUE_EPSILON);

		//Uses shrink-wrap method to converge to surface
		g = (Float_(target_field) - field) / (gradX * gradX + gradY * gradY + gradZ * gradZ);
		rootX = rootX + gradX * g;
		rootY = rootY + gradY * g;
		rootZ = rootZ + gradZ * g;

		//x = x + (d*grad) * g;
		fieldValue(rootX, rootY, rootZ, field);

		outputField = field[0];
		//outputField = cptblobPrims->fieldvalue(x, lpStoreFVOps, lpStoreFVPrims);
		//output = x;
		if(Absolutef(outputField - target_field) < FIELDVALUE_EPSILON)
			break;
	}

	output = vec3f(rootX[0], rootY[0], rootZ[0]);
	outputField = field[0];

	return (i+1)*4;
}

}
}

