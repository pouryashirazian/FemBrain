//#include "stdafx.h"
#include "PS_ArcBallCamera.h"

namespace PS
{


CArcBallCamera::CArcBallCamera()
{
	m_omega = horizontalAngle;
	m_phi = verticalAngle;
	m_rho = zoom;
	m_origin = svec3f(0.0f, 0.0f, 0.0f);
	m_center = svec3f(0.0f, 0.0f, 0.0f);
	m_mouseButton = mbNone;
	m_lastPos = svec2i(0, 0);
}

//Constructor with valid values
CArcBallCamera::CArcBallCamera(float o, float p, float r)
{
	setHorizontalAngle(o);
	setVerticalAngle(p);
	setZoom(r);
	setOrigin(svec3f(0.0f, 0.0f, 0.0f));
	setCenter(svec3f(0.0f, 0.0f, 0.0f));

	m_mouseButton = mbNone;
	m_lastPos = svec2i(0, 0);
}

//Set our horizontal angle can be any value (m_omega)
void CArcBallCamera::setHorizontalAngle(float o)
{
	m_omega = o;
}

//Set our vertical angle. This is clamped between 0 and 180
void CArcBallCamera::setVerticalAngle(float p)
{
	Clampf(p, verticalAngleMin, verticalAngleMax);
	m_phi = p;
}

//Zoom or CCamera distance from scene is clamped.
void CArcBallCamera::setZoom(float r)
{
	Clampf(r, zoomMin, zoomMax);
	m_rho = r;
}

//convert spherical coordinates to Eulerian values
svec3f CArcBallCamera::getCoordinates() const
{
	svec3f p = m_origin;

	p.x += float(m_rho * sin(m_phi) * cos(m_omega));
	p.z += float(m_rho * sin(m_phi) * sin(m_omega));
	p.y += float(m_rho * cos(m_phi));
	return p;
}

//Return Current CCamera Direction
svec3f CArcBallCamera::getDirection() const
{	
	svec3f dir = vsub3f(m_origin, getCoordinates());

	vnormalize3f(dir);
	return dir;
}

//Calculate an Up vector
svec3f CArcBallCamera::getUp() const
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

svec3f CArcBallCamera::getStrafe() const
{
	svec3f dir;
	dir = getCoordinates();

	vnormalize3f(dir);
	dir = vcross3f(dir, getUp());
	return dir;

}

void CArcBallCamera::goHome()
{
	m_omega = horizontalAngle;
	m_phi = verticalAngle;
	m_rho = zoom;
	m_origin = svec3f(0.0f, 0.0f, 0.0f);
	m_center = svec3f(0.0f, 0.0f, 0.0f);
}

void CArcBallCamera::mousePress(MOUSEBUTTONSTATE btn, int x, int y)
{
	m_mouseButton = btn;
	m_lastPos = svec2i(x, y);
}

void CArcBallCamera::mouseMove(int x, int y)
{
	float dx = x - m_lastPos.x;
	float dy = m_lastPos.y - y;
	m_lastPos = svec2i(x, y);
	if(m_mouseButton == mbLeft)
	{
		setHorizontalAngle(m_omega + 0.03f * dx);
		setVerticalAngle(m_phi + 0.03f * dy);
	}
	else if(m_mouseButton == mbRight)
	{
		setZoom(m_rho + dy);
	}

}


}
