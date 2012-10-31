#include "PS_Box.h"

bool AABB::intersect(const AABB& rhs) const {
    if ((lower.x >= rhs.upper.x) || (upper.x <= rhs.lower.x))
        return false;
    if ((lower.y >= rhs.upper.y) || (upper.y <= rhs.lower.y))
        return false;
    if ((lower.z >= rhs.upper.z) || (upper.z <= rhs.lower.z))
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
