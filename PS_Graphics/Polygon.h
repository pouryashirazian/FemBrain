#ifndef POLYGON_H
#define POLYGON_H

#include "PS_Vector.h"
#include <vector>

using namespace std;

namespace PS {
namespace MATH {

class Polygon {
public:
	Polygon();
	virtual ~Polygon();

	float area() const;
	vec3f centroid() const;
	bool isConvex() const;

	vec3f point(int index) { return m_vPoints[index];}
	vec3f normal(int index) {return m_vNormals[index];}
	U32 countPoints() const {return m_vPoints.size();}
private:
	vector<vec3f> m_vPoints;
	vector<vec3f> m_vNormals;
};

}
}

#endif
