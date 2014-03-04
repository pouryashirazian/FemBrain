/*
 * PS_KDTree.cpp
 *
 *  Created on: 2011-10-12
 *      Author: pourya
 */
#include "KDTree.h"
#include <math.h>
#include <assert.h>
#include <algorithm>

struct BoundEdge {
    // BoundEdge Public Methods
    BoundEdge() { }
    BoundEdge(float tt, int pn, bool starting) {
        t = tt;
        primNum = pn;
        type = starting ? START : END;
    }
    bool operator<(const BoundEdge &e) const {
        if (t == e.t)
            return (int)type < (int)e.type;
        else return t < e.t;
    }
    float t;
    int primNum;
    enum { START, END } type;
};


///////////////////////////////////////////////////////////////////////////////////////////
KDTreeAccel::KDTreeAccel(const SOABlobPrims* lpPrimitives,
						 SOALinearKDTree& kdtree,
						 int icost, int tcost, float ebonus, int maxp, int maxDepth)
{
	m_lpPrimitives = const_cast<SOABlobPrims*>(lpPrimitives);
	m_nextFreeNode = 0;
	//m_lpCosts = const_cast<U8*>(lpCosts);

	m_isectCost		= icost;
	m_traversalCost = tcost;
	m_emptyBonus = ebonus;
	m_maxPrims = maxp;
	m_maxDepth = maxDepth;

	U32 ctPrims = m_lpPrimitives->count;

	if (m_maxDepth <= 0)
		m_maxDepth = static_cast<int>(8 + 1.3f * log(float(ctPrims)));

	m_boundsLo = lpPrimitives->bboxLo;
	m_boundsHi = lpPrimitives->bboxHi;

	// Allocate working memory for kd-tree construction
	BoundEdge *edges[3];
	for (int i = 0; i < 3; ++i)
		edges[i] = new BoundEdge[2 * ctPrims];
	U32 *prims0 = new U32[ctPrims];
	U32 *prims1 = new U32[(m_maxDepth + 1) * ctPrims];

	// Initialize _primNums_ for kd-tree construction
	U32 *primNums = new U32[ctPrims];
	for (U32 i = 0; i < ctPrims; ++i)
		primNums[i] = i;

	// Start recursive construction of kd-tree
	buildTree(0, m_boundsLo, m_boundsHi, kdtree, primNums, ctPrims, m_maxDepth, edges, prims0, prims1);
	kdtree.setNodesCount(m_nextFreeNode);

	// Free working memory for kd-tree construction
	SAFE_DELETE_ARRAY(primNums);
	for (int i = 0; i < 3; ++i)
		SAFE_DELETE_ARRAY(edges[i]);
	SAFE_DELETE_ARRAY(prims0);
	SAFE_DELETE_ARRAY(prims1);

}


void KDTreeAccel::buildTree(int nodeNum, const vec3f& nodeBoundsLo, const vec3f& nodeBoundsHi,
							SOALinearKDTree& kdtree,
						    U32 *primNums, int nPrimitives, int depth,
							BoundEdge *edges[3], U32* prims0, U32* prims1, int badRefines)
{
	assert(nodeNum == m_nextFreeNode);
	assert(nodeNum < (int)PS_SIMD_PADSIZE(MAX_BVH_NODES));
	m_nextFreeNode++;

	// Initialize leaf node if termination criteria met
	if (nPrimitives <= m_maxPrims || depth == 0)
	{
		//PBRT_KDTREE_CREATED_LEAF(nPrimitives, maxDepth - depth);
		kdtree.initLeaf(nodeNum, primNums, nPrimitives, nodeBoundsLo, nodeBoundsHi);
		return;
	}


	// Initialize interior node and continue recursion

	// Choose split axis position for interior node
	int bestAxis = -1, bestOffset = -1;
	float bestCost = PS_PLUS_INFINITY;
	float oldCost = m_isectCost * float(nPrimitives);
	float totalSA =  vec3f::cubeSurface(nodeBoundsLo, nodeBoundsHi);
	float invTotalSA = 1.f / totalSA;

	vec3f d = vec3f::sub(nodeBoundsHi, nodeBoundsLo);

	// Choose which axis to split along
	int axis = (nodeBoundsHi - nodeBoundsLo).longestAxis();
	int retries = 0;
	retrySplit:

	// Initialize edges for _axis_
	for (int i = 0; i < nPrimitives; ++i)
	{
		int pn = primNums[i];
		//const BBox &bbox = allPrimBounds[pn];

		vec3f bboxLo = vec3f(m_lpPrimitives->bboxLoX[pn],
							 m_lpPrimitives->bboxLoY[pn],
							 m_lpPrimitives->bboxLoZ[pn]);

		vec3f bboxHi = vec3f(m_lpPrimitives->bboxHiX[pn],
						     m_lpPrimitives->bboxHiY[pn],
							 m_lpPrimitives->bboxHiZ[pn]);

		edges[axis][2 * i] 	   = BoundEdge(bboxLo.element(axis), pn, true);
		edges[axis][2 * i + 1] = BoundEdge(bboxHi.element(axis), pn, false);
	}

	std::sort(&edges[axis][0], &edges[axis][2 * nPrimitives]);

	// Compute cost of all splits for _axis_ to find best
	int nBelow = 0, nAbove = nPrimitives;
	for (int i = 0; i < 2 * nPrimitives; ++i)
	{
		if (edges[axis][i].type == BoundEdge::END)
			--nAbove;

		float edget = edges[axis][i].t;
		//U8 edgecost = m_lpCosts[edges[axis][i].primNum];

		if (edget > nodeBoundsLo.element(axis) && edget < nodeBoundsHi.element(axis))
		{
			// Compute cost for split at _i_th edge
			int otherAxis0 = (axis + 1) % 3;
			int otherAxis1 = (axis + 2) % 3;

			float belowSA = 2 * (d.element(otherAxis0) * d.element(otherAxis1) +
								(edget - nodeBoundsLo.element(axis)) * (d.element(otherAxis0) + d.element(otherAxis1)));

			float aboveSA = 2 * (d.element(otherAxis0) * d.element(otherAxis1) +
								(nodeBoundsHi.element(axis) - edget) * (d.element(otherAxis0) + d.element(otherAxis1)));


			float pBelow = belowSA * invTotalSA;
			float pAbove = aboveSA * invTotalSA;
			float eb = (nAbove == 0 || nBelow == 0) ? m_emptyBonus : 0.0f;
			float cost = m_traversalCost + m_isectCost * (1.0f - eb) * (pBelow * nBelow + pAbove * nAbove);

			// Update best split if this is lowest cost so far
			if (cost < bestCost)
			{
				bestCost = cost;
				bestAxis = axis;
				bestOffset = i;
			}
		}
		if (edges[axis][i].type == BoundEdge::START)
			++nBelow;
	}

	assert(nBelow == nPrimitives && nAbove == 0);

	// Create leaf if no good splits were found
	if (bestAxis == -1 && retries < 2)
	{
		++retries;
		axis = (axis + 1) % 3;
		goto retrySplit;
	}

	if (bestCost > oldCost)
		++badRefines;

	if ((bestCost > 4.f * oldCost && nPrimitives < 16) || bestAxis == -1 || badRefines == 3)
	{
		//PBRT_KDTREE_CREATED_LEAF(nPrimitives, maxDepth - depth);
		kdtree.initLeaf(nodeNum, primNums, nPrimitives, nodeBoundsLo, nodeBoundsHi);
		return;
	}

	// Classify primitives with respect to split
	int n0 = 0, n1 = 0;
	for (int i = 0; i < bestOffset; ++i)
		if (edges[bestAxis][i].type == BoundEdge::START)
			prims0[n0++] = edges[bestAxis][i].primNum;
	for (int i = bestOffset + 1; i < 2 * nPrimitives; ++i)
		if (edges[bestAxis][i].type == BoundEdge::END)
			prims1[n1++] = edges[bestAxis][i].primNum;

	// Recursively initialize children nodes
	float tsplit = edges[bestAxis][bestOffset].t;


	//PBRT_KDTREE_CREATED_INTERIOR_NODE(bestAxis, tsplit);
	vec3f bounds0Lo = nodeBoundsLo;
	vec3f bounds0Hi = nodeBoundsHi;
	vec3f bounds1Lo = nodeBoundsLo;
	vec3f bounds1Hi = nodeBoundsHi;

	bounds0Hi.setElement(bestAxis, tsplit);
	bounds1Lo.setElement(bestAxis, tsplit);
	//buildTree(0, m_boundsLo, m_boundsHi, primNums, ctPrims, m_maxDepth, edges, prims0, prims1);

	buildTree(nodeNum + 1, bounds0Lo, bounds0Hi, kdtree, prims0, n0, depth - 1,
			  edges, prims0, prims1 + nPrimitives, badRefines);

	U32 aboveChild = m_nextFreeNode;

	kdtree.initInterior(nodeNum, bestAxis, aboveChild, tsplit, nodeBoundsLo, nodeBoundsHi);

	buildTree(aboveChild, bounds1Lo, bounds1Hi, kdtree, prims1, n1, depth - 1,
			  edges, prims0, prims1 + nPrimitives, badRefines);
}

