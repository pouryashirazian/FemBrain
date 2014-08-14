#ifndef AVATAR_H
#define AVATAR_H

#include "graphics/SGMesh.h"
#include "graphics/Gizmo.h"
#include "Deformable.h"

using namespace PS;
using namespace PS::SG;

/*!
 * Synopsis: Haptics Avatar guide
 */
class AvatarProbe : public SGMesh, public IGizmoListener {
public:
	AvatarProbe();
	AvatarProbe(Deformable* tissue);
	virtual ~AvatarProbe();

	void draw();

	//From Gizmo Manager
	bool getPickMode() const { return m_pickMode;}
	void setPickMode(bool pick) { m_pickMode = pick;}

	void mousePress(int button, int state, int x, int y);
	void onTranslate(const vec3f& delta, const vec3f& pos);

	vec3f lower() const;
	vec3f upper() const;
private:
	void setup();

protected:
	int m_pickedNode;
	bool m_pickMode;

	Deformable* m_lpTissue;
	std::map<int, vec3d> m_hashVertices;
	int m_idxContactFace;
	double m_hapticForceCoeff;
	double m_contactDist;
	vec3d m_closestPoint;

	//Lower and upper corners
	vec3f m_lower;
	vec3f m_upper;

	//Outline mesh for easier view
	SGMesh m_outline;

	//Transformed aabb
	AABB m_aabbCurrent;
};


#endif
