#include "PS_Box.h"

namespace PS{

namespace MATH{


bool AABB::contains(const vec3& p) const {
	if(((p.x >= m_lower.x) && (p.x <= m_upper.x))&&
	  ((p.y >= m_lower.y) && (p.y <= m_upper.y))&&
 	  ((p.z >= m_lower.z) && (p.z <= m_upper.z)))
		return true;
	else
		return false;
}

bool AABB::intersect(const AABB& rhs) const {
    if ((m_lower.x >= rhs.m_upper.x) || (m_upper.x <= rhs.m_lower.x))
        return false;
    if ((m_lower.y >= rhs.m_upper.y) || (m_upper.y <= rhs.m_lower.y))
        return false;
    if ((m_lower.z >= rhs.m_upper.z) || (m_upper.z <= rhs.m_lower.z))
        return false;

    return true;
}

bool AABB::intersect(const Ray& ray, float t0, float t1) const
{
	float tmin, tmax, tymin, tymax, tzmin, tzmax;

	tmin = (bounds(ray.sign[0]).x - ray.start.x) * ray.inv_direction.x;
	tmax = (bounds(1 - ray.sign[0]).x - ray.start.x) * ray.inv_direction.x;
	tymin = (bounds(ray.sign[1]).y - ray.start.y) * ray.inv_direction.y;
	tymax = (bounds(1 - ray.sign[1]).y - ray.start.y) * ray.inv_direction.y;
	if ((tmin > tymax) || (tymin > tmax))
		return false;
	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;
	tzmin = (bounds(ray.sign[2]).z - ray.start.z) * ray.inv_direction.z;
	tzmax = (bounds(1 - ray.sign[2]).z - ray.start.z) * ray.inv_direction.z;
	if ((tmin > tzmax) || (tzmin > tmax))
		return false;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;
	return ((tmin < t1) && (tmax > t0));
}

void AABB::getVertices(vector<vec3f>& vertices) const {
	vertices.resize(8);
	vec3f dims = m_upper - m_lower;
	for(int i=0; i<8; i++) {
		vertices[i] = m_lower + vec3f::mul(vec3f((i & 0x04) >> 2, (i & 0x02) >> 1, i & 0x01), dims);
	}
}

}
}
