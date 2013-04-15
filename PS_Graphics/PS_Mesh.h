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

    bool read(const char *chrFilePath);
    bool write(const char *chrFilePath);
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
	enum ArrayType {atVertex, atNormal, atTexture, atFaceIndex, atNone};
	enum FaceType {ftTriangle = 3, ftQuad = 4};

	/*!
	 * Adds a vec3f v to the destination buffer dest.
	 * @return designated index of added vertex. This index is the actual vertex number
	 * not the position in the float array. The position in the float array can be computed as
	 * index x szUnitVertex
	 */
	int add(const vec3f& v, ArrayType dest = atVertex, int count = 3);
	int addVertex(const vec3f& v);
	int addNormal(const vec3f& n);
	int addTexCoord2(const vec2f& t);
	int addTexCoord3(const vec3f& t);
	void addFaceIndex(U32 index);


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
	size_t countVertices() const {return arrVertices.size() / (size_t)szUnitVertex;}
	size_t countNormals() const {return arrNormals.size() / 3;}
	size_t countFaces() const {return arrIndices.size() / (size_t)szUnitFace;}
	size_t countTexCoords() const {return arrTexCoords.size() / (size_t)szUnitTexCoord;}

	/*!
	 * returns unit vertex memory stride in bytes
	 */
	size_t getVertexStride() const {return szUnitVertex * sizeof(float);}
	size_t getNormalStride() const {return 3 * sizeof(float);}
	size_t getFaceStride() const {return szUnitFace * sizeof(U32);}


	//Access Vertex, Normal
	vec3f getAsVec3(int index, ArrayType dest = atVertex) const;
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
    vector<float> vertices() const {return arrVertices;}
    vector<float> normals() const {return arrNormals;}
    vector<float> texcoords() const {return arrTexCoords;}
    vector<U32> indices() const {return arrIndices;}

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

	bool load(const char* chrFileName);

	//Mesh Nodes
	void addNode(MeshNode* lpMeshNode);
	MeshNode*	getNode(int idx) const;
    int countNodes() const;

	//Mesh Materials
	void addMeshMaterial(MeshMaterial* lpMaterial);
	MeshMaterial* getMaterial(int idx) const;
    MeshMaterial* getMaterial(const string& strName) const;

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
