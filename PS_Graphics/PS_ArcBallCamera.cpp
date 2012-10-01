//#include "stdafx.h"
#include "PS_ArcBallCamera.h"
#include <GL/glew.h>

namespace PS
{


ArcBallCamera::ArcBallCamera()
{
	m_omega = horizontalAngle;
	m_phi = verticalAngle;
	m_rho = zoom;
	m_origin = svec3f(0.0f, 0.0f, 0.0f);
	m_center = svec3f(0.0f, 0.0f, 0.0f);
	m_mouseButton = mbNone;
	m_lastPos = svec2i(0, 0);
	m_pan = svec2f(0, 0);
}

//Constructor with valid values
ArcBallCamera::ArcBallCamera(float o, float p, float r)
{
	setRoll(o);
	setTilt(p);
	setZoom(r);
	setOrigin(svec3f(0.0f, 0.0f, 0.0f));
	setCenter(svec3f(0.0f, 0.0f, 0.0f));

	m_mouseButton = mbNone;
	m_lastPos = svec2i(0, 0);
}

//Set our horizontal angle can be any value (m_omega)
void ArcBallCamera::setRoll(float rollHDeg)
{
	m_omega = DEGTORAD(rollHDeg);
}

//Set our vertical angle. This is clamped between 0 and 180
void ArcBallCamera::setTilt(float tiltVDeg)
{
	float tiltVRad = DEGTORAD(tiltVDeg);
	Clampf(tiltVRad, verticalAngleMin, verticalAngleMax);
	m_phi = tiltVRad;
}

//Zoom or CCamera distance from scene is clamped.
void ArcBallCamera::setZoom(float r)
{
	Clampf(r, zoomMin, zoomMax);
	m_rho = r;
}

//convert spherical coordinates to Eulerian values
svec3f ArcBallCamera::getPos() const
{
	svec3f p = m_origin;

	p.x += float(m_rho * sin(m_phi) * cos(m_omega));
	p.z += float(m_rho * sin(m_phi) * sin(m_omega));
	p.y += float(m_rho * cos(m_phi));
	return p;
}

//Return Current CCamera Direction
svec3f ArcBallCamera::getDir() const
{	
	svec3f dir = vsub3f(m_origin, getPos());

	vnormalize3f(dir);
	return dir;
}

//Calculate an Up vector
svec3f ArcBallCamera::getUp() const
{	
	float o = (m_omega + PiOver2);
	float ph = Absolutef(m_phi - PiOver2);

	svec3f p;
	p.x = (m_rho * cos(o) * sin(ph));
	p.z = (m_rho * sin(o) * sin(ph));
	p.y = (m_rho * cos(ph));

	vnormalize3f(p);
	return p;
}

svec3f ArcBallCamera::getStrafe() const
{
	svec3f dir;
	dir = getPos();

	vnormalize3f(dir);
	dir = vcross3f(dir, getUp());
	return dir;

}

void ArcBallCamera::goHome()
{
	m_omega = horizontalAngle;
	m_phi = verticalAngle;
	m_rho = zoom;
	m_origin = svec3f(0.0f, 0.0f, 0.0f);
	m_center = svec3f(0.0f, 0.0f, 0.0f);
}

void ArcBallCamera::mousePress(int button, int state, int x, int y)
{
	m_mouseButton = (MOUSEBUTTONSTATE)button;
	m_lastPos = svec2i(x, y);
}

void ArcBallCamera::mouseMove(int x, int y)
{
	float dx = x - m_lastPos.x;
	float dy = m_lastPos.y - y;
	m_lastPos = svec2i(x, y);

	//Spherical movement on the left button
	if(m_mouseButton == mbLeft)
	{
		setRoll(this->getRoll() + dx);
		setTilt(this->getTilt() + dy);
	}
	else if(m_mouseButton == mbMiddle)
	//Pan on Middle
	{
		m_pan.x += 0.01f * dx;
		m_pan.y += 0.01f * dy;
	}
}

void ArcBallCamera::mouseWheel(int button, int dir, int x, int y)
{
	//Zoom on Mouse Wheel
	if(dir > 0)
		setZoom(m_rho + 1);
	else
		setZoom(m_rho - 1);
}

void ArcBallCamera::look()
{
	glTranslatef(m_pan.x, m_pan.y, 0.0f);
	svec3f p = this->getPos();
	svec3f c = this->getCenter();
	gluLookAt(p.x, p.y, p.z, c.x, c.y, c.z, 0.0f, 1.0f, 0.0f);
}


}
