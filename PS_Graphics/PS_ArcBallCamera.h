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

class ArcBallCamera
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
	svec2f m_pan;

	svec3f m_xAxis;
	svec3f m_yAxis;
	svec3f m_zAxis;

	MOUSEBUTTONSTATE m_mouseButton;
public:
	//Default Constructor
	ArcBallCamera();

	//Constructor with valid values
	ArcBallCamera(float o, float p, float r);

	//Look Matrix
	void look();

	//Roll is the horizontal movement
	float getRoll() const {return RADTODEG(m_omega);}
	void setRoll(float rollHDeg);

	//Tilt is the vertical movement
	float getTilt() const {return RADTODEG(m_phi);}
	void setTilt(float tiltVDeg);

	//Zoom
	float getCurrentZoom() const {return m_rho;}
	void setZoom(float r);

	//Pan
	svec2f getPan() const { return m_pan;}
	void setPan(const svec2f& pan) { m_pan = pan;}

	//Origin position
	svec3f getOrigin() {return m_origin;}
	void setOrigin(const svec3f& org) { m_origin = org;}

	//Center Position
	svec3f getCenter() {return m_center;}
	void setCenter(const svec3f& c) {m_center = c;}


	//Handle Mouse Events
	void mousePress(int button, int state, int x, int y);
	void mouseMove(int x, int y);
	void mouseWheel(int button, int dir, int x, int y);


	//convert spherical coordinates to Eulerian values
	svec3f getPos() const;

	//Return Current CCamera Direction
	svec3f getDir() const;

	//Calculate an Up vector
	svec3f getUp() const;

	svec3f getStrafe() const;

	//Last position
	svec2i getLastPos() const {return m_lastPos;}
	void setLastPos(const svec2i& lastPos) { m_lastPos = lastPos;}

	//
	//svec3f convertToWorld(const svec3f& pos);
	void computeLocalCoordinateSystem();
	void screenToWorld_OrientationOnly3D(const svec3f& ptScreen, svec3f& ptWorld);

	void goHome();
};

}
#endif
