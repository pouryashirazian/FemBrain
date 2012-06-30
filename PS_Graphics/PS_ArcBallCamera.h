#ifndef PS_ARCBALLCAMERA_H
#define PS_ARCBALLCAMERA_H

#include "PS_VectorMath.h"

const float zoomMin = 0.01f;
const float zoomMax = 80.0f;
const float zoom = 30.0f;
const float verticalAngleMin = 0.01f;
const float verticalAngleMax = Pi - 0.01f;
const float horizontalAngle = -1.0f * PiOver2;
const float verticalAngle = PiOver2;

//A basic ArcBall Camera class to be used with openGL or directX
namespace PS{

using namespace PS::FUNCTIONALMATH;

class CArcBallCamera
{
public:
	enum MOUSEBUTTONSTATE {mbLeft, mbMiddle, mbRight, mbNone};

private:
	//Omega and Phi are horizontal and vertical angles of spherical coordinates respectively
	//rho is the CCamera distance from scene (Zoom)
	float m_omega, m_phi, m_rho;
	svec3f m_origin;

	//Center point in scene to lookAt
	svec3f  m_center;
	svec2i m_lastPos;
	MOUSEBUTTONSTATE m_mouseButton;
public:

	//Default Constructor
	CArcBallCamera();

	//Constructor with valid values
	CArcBallCamera(float o, float p, float r);

	//Access to member variables of this class
	const float getHorizontalAngle() const {return m_omega;}
	const float getVerticalAngle() const {return m_phi;}
	const float getCurrentZoom() const {return m_rho;}
	svec3f getOrigin() {return m_origin;}
	svec3f getCenter() {return m_center;}

	void mousePress(MOUSEBUTTONSTATE btn, int x, int y);
	void mouseMove(int x, int y);

	//Set our horizontal angle can be any value (Omega)
	void setHorizontalAngle(float o);

	//Set our vertical angle. This is clamped between 0 and 180
	void setVerticalAngle(float p);

	//Zoom or CCamera distance from scene is clamped.
	void setZoom(float r);

	//Set Origin
	void setOrigin(const svec3f& org) { m_origin = org;}

	//Set Center point inside scene
	void setCenter(const svec3f& c) {m_center = c;}

	//convert spherical coordinates to Eulerian values
	svec3f getCoordinates() const;

	//Return Current CCamera Direction
	svec3f getDirection() const;

	//Calculate an Up vector
	svec3f getUp() const;

	svec3f getStrafe() const;

	//Set the last position
	void setLastPos(const svec2i& lastPos)
	{
		m_lastPos = lastPos;
	}

	svec2i getLastPos() const {return m_lastPos;}

	void goHome();
};

}
#endif
