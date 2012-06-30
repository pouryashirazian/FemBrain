/*
 * PS_KDTree.h
 *
 *  Created on: 2011-10-12
 *      Author: pourya
 */

#ifndef PS_KDTREE_H_
#define PS_KDTREE_H_

#include "PS_SIMDVecN.h"
#include "PS_Polygonizer.h"

#define MAX_BVH_NODES 256

using namespace PS::MATHSIMD;
using namespace PS::SIMDPOLY;
using namespace PS::FUNCTIONALMATH;


struct BoundEdge;

//Aligned Structure for KDTree
struct PS_BEGIN_ALIGNED(PS_SIMD_FLEN) SOALinearKDTree
{
	float vBoundsLoX[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	float vBoundsLoY[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	float vBoundsLoZ[PS_SIMD_PADSIZE(MAX_BVH_NODES)];

	float vBoundsHiX[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	float vBoundsHiY[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	float vBoundsHiZ[PS_SIMD_PADSIZE(MAX_BVH_NODES)];

	union{
		float split[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
		U32 onePrimitive[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	};

	SOALinearKDTree()
	{
		ctNodes = 0;
	}

	void initLeaf(U32 idx, U32* primNums, int np, const svec3f& lo, const svec3f& hi)
	{
	    flags[idx] = 3;
	    nPrims[idx] |= (np << 2);
	    // Store primitive ids for leaf node
	    if (np == 0)
	        onePrimitive[idx] = 0;
	    else if (np == 1)
	        onePrimitive[idx] = primNums[0];
	    vBoundsLoX[idx] = lo.x;
	    vBoundsLoY[idx] = lo.y;
	    vBoundsLoZ[idx] = lo.z;

	    vBoundsHiX[idx] = hi.x;
	    vBoundsHiY[idx] = hi.y;
	    vBoundsHiZ[idx] = hi.z;
	}

	void initInterior(U32 idx, U32 axis, U32 ac, float s, const svec3f& lo, const svec3f& hi)
	{
		split[idx] = s;
		flags[idx] = axis;
		aboveChild[idx] |= (ac << 2);

	    vBoundsLoX[idx] = lo.x;
	    vBoundsLoY[idx] = lo.y;
	    vBoundsLoZ[idx] = lo.z;

	    vBoundsHiX[idx] = hi.x;
	    vBoundsHiY[idx] = hi.y;
	    vBoundsHiZ[idx] = hi.z;
	}

    U32 GetSplitAxis(U32 idx) const { return (flags[idx]) & 3; }
    bool IsLeaf(U32 idx) const { return (flags[idx] & 3) == 3; }

    U32 GetPrimCount(U32 idx) const { return (nPrims[idx]) >> 2; }
    U32 GetAboveChild(U32 idx) const { return (aboveChild[idx]) >> 2; }

    void setNodesCount(U32 count) {ctNodes = count;}
    U32 getNodesCount() {return ctNodes;}
private:
	union{
		U32 flags[PS_SIMD_PADSIZE(MAX_BVH_NODES)];		//Both
		U32 nPrims[PS_SIMD_PADSIZE(MAX_BVH_NODES)]; 	//Leaf
		U32 aboveChild[PS_SIMD_PADSIZE(MAX_BVH_NODES)]; //Interior
	};

	U32 ctNodes;
} PS_END_ALIGNED(PS_SIMD_FLEN);


class KDTreeAccel
{
public:
	KDTreeAccel(const SOABlobPrims* lpPrimitives, SOALinearKDTree& kdtree,
				int icost = 80, int tcost = 1, float ebonus = 0.5f, int maxp = 1, int maxDepth = 6);
private:
	void buildTree(int nodeNum,
				   const svec3f& nodeBoundsLo,
				   const svec3f& nodeBoundsHi,
				   SOALinearKDTree& kdtree,
				   U32 *primNums, int nPrimitives, int depth,
				   BoundEdge *edges[3], U32* prims0, U32* prims1, int badRefines = 0);

	// KdTreeAccel Private Data
	int m_isectCost;
	int m_traversalCost;
	int m_maxPrims;
	int m_maxDepth;

	int m_nextFreeNode;
	float m_emptyBonus;


	SOABlobPrims* m_lpPrimitives;
	U8* m_lpCosts;
	svec3f m_boundsLo;
	svec3f m_boundsHi;
};

#endif /* PS_KDTREE_H_ */
