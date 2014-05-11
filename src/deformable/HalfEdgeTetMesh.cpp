/*
 * WingedEdgeTetMesh.cpp
 *
 *  Created on: May 6, 2014
 *      Author: pourya
 */

#include "HalfEdgeTetMesh.h"
#include "base/Logger.h"
#include <map>
#include <utility>
#include <algorithm>


using namespace std;
using namespace PS;

namespace PS {
namespace FEM {

struct FaceKey
{
    FaceKey(U32 f[3], U32 count) { key = (f[0] * count + f[1]) * count + f[2];}

    static void order_lo2hi(U32& a, U32& b, U32& c) {
    	if(a > b)
    		swap(a, b);
    	if(b > c)
    		swap(b, c);
    	if(a > b)
    		swap(a, b);
    }

    bool operator<(const FaceKey& k) const { return key < k.key; }

    bool operator>(const FaceKey& k) const { return key > k.key; }

    U64 key;
};

HalfEdgeTetMesh::HalfEdgeTetMesh() {

}

HalfEdgeTetMesh::HalfEdgeTetMesh(U32 ctVertices, double* vertices, U32 ctElements, U32* elements) {
	setup(ctVertices, vertices, ctElements, elements);
}

HalfEdgeTetMesh::HalfEdgeTetMesh(const vector<double>& vertices, const vector<U32>& elements) {

	U32 ctVertices = vertices.size() / 3;
	U32 ctElements = elements.size() / 4;

	setup(ctVertices, &vertices[0], ctElements, &elements[0]);
}

HalfEdgeTetMesh::~HalfEdgeTetMesh() {
	cleanup();
}

bool HalfEdgeTetMesh::setup(U32 ctVertices, const double* vertices, U32 ctElements, const U32* elements) {

	//add all vertices first
	for(U32 i=0; i<ctVertices; i++) {
		HalfEdgeTetMesh::NODE node;
		node.pos = node.restpos = vec3d(&vertices[i*3]);
		node.outHE = INVALID_INDEX;
		m_vNodes.push_back(node);
	}

	//Face Mask
	int faceMaskPos[4][3] = { {1, 2, 3}, {2, 0, 3}, {3, 0, 1}, {1, 0, 2} };
	int faceMaskNeg[4][3] = { {3, 2, 1}, {3, 0, 2}, {1, 0, 3}, {2, 0, 1} };

	//add all edges and elements
	std::map< pair<U32, U32>, HEDGE* > mapHalfEdges;
	typedef std::map< pair<U32, U32>, HEDGE* >::iterator HEITER;

	vector<double> vDeterminants;
	vDeterminants.resize(ctElements);

	m_vElements.reserve(ctElements);
	for(U32 i=0; i<ctElements; i++) {

		vec4u32 n = vec4u32(&elements[i * 4]);
		vec3d v[4];
		for(int j=0; j<4; j++)
			v[j] = vec3d(&vertices[ 3 * n[j] ]);

		//Add element nodes set only for now
		HalfEdgeTetMesh::ELEM elem;
		n.store(&elem.nodes[0]);
		for(int j=0; j<4; j++)
			elem.faces[j] = INVALID_INDEX;
		m_vElements.push_back(elem);


		//compute determinant
		vDeterminants[i] = vec3d::dot(v[1] - v[0], vec3d::cross(v[2] - v[0], v[3] - v[0]));

		//Loop over faces. Per each tet 6 edges or 12 half-edges added
		for(int f = 0; f < 4; f++) {
			U32 fv[3];

			if(vDeterminants[i] >= 0) {
				fv[0] = n[ faceMaskPos[f][0] ];
				fv[1] = n[ faceMaskPos[f][1] ];
				fv[2] = n[ faceMaskPos[f][2] ];
			}
			else {
				fv[0] = n[ faceMaskNeg[f][0] ];
				fv[1] = n[ faceMaskNeg[f][1] ];
				fv[2] = n[ faceMaskNeg[f][2] ];
			}

			//Add all halfedges for this face
			for(int e=0; e<3; e++) {
				HalfEdgeTetMesh::HEDGE* he = new HalfEdgeTetMesh::HEDGE();

				he->from = fv[e];
				he->to = fv[ (e + 1) % 3 ];
				mapHalfEdges.insert( make_pair( std::make_pair(he->from, he->to), he));
			}
		}
	}

	//add final half edges in the order of (u, v) and (v, u)
	m_vHalfEdges.reserve(mapHalfEdges.size());
	std::map< pair<U32, U32>, U32 > mapHalfEdgesIndex;
	typedef std::map< pair<U32, U32>, U32 >::iterator HEINDEXITER;

	U32 from, to = -1;
	U32 idxForward, idxBackward = -1;

	while(mapHalfEdges.size() > 0) {
		//add forward
		HEITER forward = mapHalfEdges.begin();
		from = forward->second->from;
		to = forward->second->to;

		m_vHalfEdges.push_back(* forward->second);
		idxForward = m_vHalfEdges.size() - 1;
		mapHalfEdgesIndex.insert( make_pair( std::make_pair(from, to), idxForward));


		//Remove from map
		mapHalfEdges.erase(forward);

		//add backward
		HEITER backward = mapHalfEdges.find(std::make_pair(to, from));
		if(backward != mapHalfEdges.end()) {
			m_vHalfEdges.push_back(* backward->second);
			idxBackward = idxForward + 1;
			mapHalfEdgesIndex.insert( make_pair( std::make_pair(to, from), idxBackward));

			//Remove from map
			mapHalfEdges.erase(backward);

			//Set opposite indices
			m_vHalfEdges[idxForward].opposite = idxBackward;
			m_vHalfEdges[idxBackward].opposite = idxForward;


		}
		else {
			LogErrorArg2("Opposite edge not found for: <%d. %d>", from, to);
			return false;
		}
	}

	//add all faces now and finish setting information for all
	std::map< FaceKey, U32 > mapFaces;
	typedef std::map< FaceKey, U32 >::iterator FITER;

	m_vFaces.reserve(ctElements * 4);
	for(U32 i=0; i<ctElements; i++) {
		vec4u32 n = vec4u32(&elements[i * 4]);

		//Loop over faces. Per each tet 6 edges or 12 half-edges added
		for(int f = 0; f < 4; f++) {
			U32 fv[3];

			if(vDeterminants[i] >= 0) {
				fv[0] = n[ faceMaskPos[f][0] ];
				fv[1] = n[ faceMaskPos[f][1] ];
				fv[2] = n[ faceMaskPos[f][2] ];
			}
			else {
				fv[0] = n[ faceMaskNeg[f][0] ];
				fv[1] = n[ faceMaskNeg[f][1] ];
				fv[2] = n[ faceMaskNeg[f][2] ];
			}

			//add faces
			HalfEdgeTetMesh::FACE face;

			//Add all halfedges for this face
			for(int e=0; e<3; e++) {
				from = fv[e];
				to = fv[ (e + 1) % 3 ];

				HEINDEXITER it = mapHalfEdgesIndex.find(std::make_pair(from, to));
				if(it != mapHalfEdgesIndex.end())
					face.halfedge[e] = it->second;
				else {
					LogErrorArg2("Setting face edges failed! Unable to find edge <%d, %d>", from, to);
					return false;
				}
			}

			//Resolve unique faces
			//printf("Face order before: %d, %d, %d\n", fv[0], fv[1], fv[2]);
			FaceKey::order_lo2hi(fv[0], fv[1], fv[2]);
			//printf("Face order after: %d, %d, %d\n", fv[0], fv[1], fv[2]);
			FaceKey key(fv, m_vHalfEdges.size());
			FITER fit = mapFaces.find(key);

			//if face not found then add it
			U32 idxFace = INVALID_INDEX;
			if(fit == mapFaces.end()) {
				m_vFaces.push_back(face);
				idxFace = m_vFaces.size() - 1;
				mapFaces.insert( std::make_pair(key, idxFace));

				//set prev, next, face for half edges
				for(int e=0; e<3; e++) {
					if(e == 0)
						m_vHalfEdges[ face.halfedge[e] ].prev = face.halfedge[2];
					else
						m_vHalfEdges[ face.halfedge[e] ].prev = face.halfedge[e - 1];
					m_vHalfEdges[ face.halfedge[e] ].next = face.halfedge[(e + 1) % 3];
					m_vHalfEdges[ face.halfedge[e] ].face = idxFace;
				}
			}
			else
				idxFace = fit->second;

			//Set element face index
			m_vElements[i].faces[f] = idxFace;
		}
	}

	//Set outgoing edges for nodes
	for(U32 i=0; i < m_vHalfEdges.size(); i++) {
		HalfEdgeTetMesh::HEDGE e = m_vHalfEdges[i];

		//set the outHalfEdge for all nodes in this edge
		if(m_vNodes[e.from].outHE == INVALID_INDEX)
			m_vNodes[e.from].outHE = i;
		if(m_vNodes[e.to].outHE == INVALID_INDEX)
			m_vNodes[e.to].outHE = i;
	}

	this->printInfo();
	return true;
}

void HalfEdgeTetMesh::cleanup() {
	m_vElements.resize(0);
	m_vFaces.resize(0);
	m_vHalfEdges.resize(0);
	m_vNodes.resize(0);
}

void HalfEdgeTetMesh::printInfo() const {
	//print all nodes
	printf("NODES #%u\n", m_vNodes.size());
	for(U32 i=0; i < m_vNodes.size(); i++) {
		HalfEdgeTetMesh::NODE n = m_vNodes[i];

		printf("NODE outgoing edge: %d, pos: [%.3f, %.3f, %.3f]\n", n.outHE, n.pos.x, n.pos.y, n.pos.z);
	}

	//print all edges
	printf("HALFEDGES #%u\n", m_vHalfEdges.size());
	for(U32 i=0; i < m_vHalfEdges.size(); i++) {
		HalfEdgeTetMesh::HEDGE e = m_vHalfEdges[i];

		printf("HEDGE %d, from: %d, to %d, next: %d, prev: %d, opposite: %d, face: %d\n", i, e.from, e.to, e.next, e.prev, e.opposite, e.face);
	}

	//print all face
	printf("FACES #%u\n", m_vFaces.size());
	for(U32 i=0; i < m_vFaces.size(); i++) {
		HalfEdgeTetMesh::FACE face = m_vFaces[i];
		printf("FACE %d, [%d, %d, %d]\n", i, face.halfedge[0], face.halfedge[1], face.halfedge[2]);
	}

	//print all elements
	printf("ELEMS #%u\n", m_vElements.size());
	for(U32 i=0; i < m_vElements.size(); i++) {
		HalfEdgeTetMesh::ELEM elem = m_vElements[i];
		printf("ELEM %d, NODE: [%u, %u, %u, %u]", i ,elem.nodes[0], elem.nodes[1], elem.nodes[2], elem.nodes[3]);
		printf(" FACE: [%u, %u, %u, %u]\n", elem.faces[0], elem.faces[1], elem.faces[2], elem.faces[3]);
	}

}

//add/remove
int HalfEdgeTetMesh::insert_element(const ELEM& e) {

}

void HalfEdgeTetMesh::remove_element(U32 t) {

}

HalfEdgeTetMesh::ELEM& HalfEdgeTetMesh::elemAt(U32 i) {
	assert(isElemIndex(i));
	return m_vElements[i];
}

HalfEdgeTetMesh::FACE& HalfEdgeTetMesh::faceAt(U32 i) {
	assert(isFaceIndex(i));
	return m_vFaces[i];
}

HalfEdgeTetMesh::HEDGE& HalfEdgeTetMesh::halfedgeAt(U32 i) {
	assert(isHalfEdgeIndex(i));
	return m_vHalfEdges[i];
}

HalfEdgeTetMesh::EDGE HalfEdgeTetMesh::edgeAt(U32 i) const {
	assert(isEdgeIndex(i));
	EDGE e(const_halfedgeAt(i * 2), const_halfedgeAt(i * 2 + 1));
	return e;
}

HalfEdgeTetMesh::NODE& HalfEdgeTetMesh::nodeAt(U32 i) {
	assert(isElemIndex(i));
	return m_vNodes[i];
}


//access
const HalfEdgeTetMesh::ELEM& HalfEdgeTetMesh::const_elemAt(U32 i) const {
	assert(isElemIndex(i));
	return m_vElements[i];
}

const HalfEdgeTetMesh::FACE& HalfEdgeTetMesh::const_faceAt(U32 i) const {
	assert(isFaceIndex(i));
	return m_vFaces[i];
}

const HalfEdgeTetMesh::HEDGE& HalfEdgeTetMesh::const_halfedgeAt(U32 i) const {
	assert(isHalfEdgeIndex(i));
	return m_vHalfEdges[i];
}

const HalfEdgeTetMesh::NODE& HalfEdgeTetMesh::const_nodeAt(U32 i) const {
	assert(isNodeIndex(i));
	return m_vNodes[i];
}


int HalfEdgeTetMesh::split_edge(int idxEdge, double t) {

	U32 idxHE0 = idxEdge*2;
	U32 idxHE1 = idxHE0 + 1;

	HEDGE& he0 = halfedgeAt(idxHE0);
	HEDGE& he1 = halfedgeAt(idxHE1);

	U32 from = vertex_from_hedge(idxHE0);
	U32 to = vertex_to_hedge(idxHE1);

	NODE n0 = const_nodeAt(from);
	NODE n1 = const_nodeAt(to);

	//add new point
	NODE np;
	np.pos = n0.pos + (n1.pos - n0.pos) * t;
	np.restpos = n0.restpos + (n1.restpos - n0.restpos) * t;
	np.outHE = idxEdge*2 + 1;
	m_vNodes.push_back(np);
	U32 idxNewNode = m_vNodes.size() - 1;

	//Add new halfedges
	HEDGE nh0 = he0;
	HEDGE nh1 = he1;
	nh0.from = idxNewNode;
	nh0.prev = idxHE0;
	nh1.to 	 = idxNewNode;
	nh1.next = idxHE1;

	m_vHalfEdges.push_back(nh0);
	idxHE0 = m_vHalfEdges.size() - 1;
	m_vHalfEdges.push_back(nh1);
	idxHE1 = m_vHalfEdges.size() - 1;

	//update old halfedges
	he0.to = idxNewNode;
	he0.next = idxHE0;
	he1.from = idxNewNode;
	he1.prev = idxHE1;


	return 1;
}

int HalfEdgeTetMesh::getFirstRing(int idxNode, vector<U32>& ringNodes) const {
	U32 he = m_vNodes[idxNode].outHE;
	ringNodes.resize(0);

	U32 node = INVALID_INDEX;
	U32 orgNode = vertex_to_hedge(he);
	while(node != orgNode) {
		he = next_hedge(opposite_hedge(he));
		node = vertex_to_hedge(he);
		ringNodes.push_back(node);
	}

	return (int)ringNodes.size();
}


void HalfEdgeTetMesh::draw() {

}


}
}
