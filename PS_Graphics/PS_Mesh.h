/*
 * PS_Mesh.h
 *
 *  Created on: Jul 4, 2011
 *      Author: pourya
 */

#ifndef PS_MESH_H_
#define PS_MESH_H_

#include <vector>

#include "../PS_Base/PS_String.h"
#include "PS_GLMeshBuffer.h"
#include "AssetManager.h"
#include "PS_Vector.h"
#include "PS_Quaternion.h"
#include "PS_Box.h"


using namespace std;
using namespace PS;
using namespace PS::MATH;

namespace PS{
namespace MESH{


/*!
 * structure holding a mesh material object
 */
class MeshMaterial : public Asset
{
public:
    MeshMaterial();
    MeshMaterial(const char* chrFilePath);
    virtual ~MeshMaterial();

    bool load(const char *chrFilePath);
    bool store(const char *chrFilePath);
public:
    string strMTLName;
    string strTexName;
	vec3f ambient;
	vec3f diffused;
	vec3f specular;
	float reflect;
	float refract;
	float trans;
	float shininess;
	float glossy;
	float refractIndex;

	int illuminationModel;
};

/*!
 * MeshNode represents a single node in our mesh file.
 * It is complete mesh by itself.
 *
 */
class MeshNode {
public:
	MeshNode();
    MeshNode(const string& name);
	virtual ~MeshNode();

    void init();

	//Readback mesh
	bool readbackMeshV3T3(U32& ctVertices, vector<float>& vertices,
							 U32& ctTriangles, vector<U32>& elements);

	/*!
	 * Adds a vec3f v to the destination buffer dest.
	 * @return designated index of added vertex. This index is the actual vertex number
	 * not the position in the float array. The position in the float array can be computed as
	 * index x szUnitVertex
	 */
	int add(const vec3f& v, VertexAttribType vat = vatPosition, int step = 3);
	void add(const vector<float>& arrAttribs, VertexAttribType vat = vatPosition, int step = 3);

	int addVertex(const vec3f& v);
	int addNormal(const vec3f& n);
	int addTexCoord2(const vec2f& t);
	int addTexCoord3(const vec3f& t);
	void addFaceIndex(U32 index);
	void addFaceIndices(const vector<U32>& arrFaces, int unitFace);


	/*!
	 * Checks all face indices for invalid values
	 * @return number of indices found out of range
	 */
	int countFaceIndexErrors() const;

    /*!
     * \brief computeVertexNormalsFromFaces compute one normal per each vertex
     * by summing up all adjacent face normals and normalize it.
     */
    void computeVertexNormalsFromFaces();


	//Statistics
	U32 countVertices() const {return arrVertices.size() / (U32)szUnitVertex;}
	U32 countNormals() const {return arrNormals.size() / 3;}
	U32 countFaces() const {return arrIndices.size() / (U32)szUnitFace;}
	U32 countTexCoords() const {return arrTexCoords.size() / (U32)szUnitTexCoord;}

	/*!
	 * returns unit vertex memory stride in bytes
	 */
	U32 getVertexStride() const {return szUnitVertex * sizeof(float);}
	U32 getNormalStride() const {return 3 * sizeof(float);}
	U32 getFaceStride() const {return szUnitFace * sizeof(U32);}


	//Access Vertex, Normal
	vec3f getVertex(int idxVertex) const;
	vec3f getNormal(int idxVertex) const;
	vec2f getTexCoord2(int idxVertex) const;
	vec3f getTexCoord3(int idxVertex) const;

	//Access Material
	MeshMaterial* getMaterial() const {return lpMaterial;}
	void setMaterial(MeshMaterial* aMaterial) {lpMaterial = aMaterial;}

	AABB computeBoundingBox() const;

    //Name
    string getName() const {return strNodeName;}
    void setName(const string& strName) {strNodeName = strName;}

    //Steps
    U8 getUnitVertex() const {return szUnitVertex;}
    void setUnitVertex(U8 s) { szUnitVertex = s;}

    U8 getUnitTexCoord() const {return szUnitTexCoord;}
    void setUnitTexCoord(U8 s) {szUnitTexCoord = s;}

    U8 getUnitFace() const {return szUnitFace;}
    void setUnitFace(U8 s) {szUnitFace = s;}

    //Mesh parts
    void getVertexAttrib(U32& count, vector<float>& arrAttribs, VertexAttribType vat) const;
    void getFaces(U32& count, vector<U32>& faces) const;

    /*!
     * Move the entire mesh to a new location
     */
    void move(const vec3f& d);
    void scale(const vec3f& s);
    void rotate(const quat& q);
    void fitToBBox(const AABB& box);
public:
	std::vector<float> arrVertices;
	std::vector<float> arrNormals;
	std::vector<float> arrTexCoords;
	std::vector<U32>	arrIndices;

    //Init data
    string      strNodeName;
	U8			szUnitVertex;
	U8   		szUnitTexCoord;
	U8			szUnitFace;
	MeshMaterial* lpMaterial;
};


/*
 * structure holding a complete mesh
 */
class Mesh : public Asset{
public:
    Mesh();
	Mesh(const char* chrFileName);
	virtual ~Mesh();

	//IO
	bool load(const char* chrFilePath);
	bool store(const char* chrFilePath);

	//Mesh Nodes
	void addNode(MeshNode* lpMeshNode);
	MeshNode*	getNode(int idx) const;
    U32 countNodes() const {return m_nodes.size();}

	//Mesh Materials
	void addMeshMaterial(MeshMaterial* lpMaterial);
	MeshMaterial* getMaterial(int idx) const;
    MeshMaterial* getMaterial(const string& strName) const;
    U32 countMaterials() const { return m_materials.size();}

    //Get the AABB for the entire mesh
	AABB computeBoundingBox() const;

    //Compute all missing normals
    void computeMissingNormals();

    //Moves the entire mesh to a new location
    void move(const vec3f& d);
    void scale(const vec3f& s);
    void rotate(const quat& q);
    void fitToBBox(const AABB& box);
private:
	bool loadObj(const char* chrFileName);
	bool loadObjGlobalVertexNormal(const char* chrFileName);

private:
	std::vector<MeshNode*> m_nodes;
	std::vector<MeshMaterial*> m_materials;
    std::map<string, MeshMaterial*> m_mapMaterial;
    string m_strFilePath;
};

/*
class CMeshManager : public CResourceManager<CMesh>
{
public:
	CMeshManager();
	~CMeshManager();

	void cleanup();

	CMesh* loadResource(const DAnsiStr& inStrFilePath, DAnsiStr& inoutStrName);
};
*/

}
}
#endif /* PS_MESH_H_ */
