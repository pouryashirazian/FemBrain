#include "AffineWidgets.h"
#include "PS_Box.h"
#include <GL/glew.h>

GLfloat g_vertices[][3] = { { -1.0, -1.0, 1.0 }, { -1.0, 1.0, 1.0 }, { 1.0, 1.0,
		1.0 }, { 1.0, -1.0, 1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, 1.0, -1.0 }, {
		1.0, 1.0, -1.0 }, { 1.0, -1.0, -1.0 } };

svec3f CUIWidget::maskDisplacement(svec3f v, UITRANSFORMAXIS axis) {
	if (axis == uiaFree)
		return v;
	else if (axis == uiaX)
		return svec3f(v.x, 0.0f, 0.0f);
	else if (axis == uiaY)
		return svec3f(0.0f, v.y, 0.0f);
	else if (axis == uiaZ)
		return svec3f(0.0f, 0.0f, v.z);
	else
		return v;
}

void CUIWidget::drawCubePolygon(int a, int b, int c, int d) {
	glBegin(GL_POLYGON);
	glVertex3fv(g_vertices[a]);
	glVertex3fv(g_vertices[b]);
	glVertex3fv(g_vertices[c]);
	glVertex3fv(g_vertices[d]);
	glEnd();
}

svec4f CUIWidget::maskColor(UITRANSFORMAXIS axis) {
	static const GLfloat red[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	static const GLfloat green[] = { 0.0f, 1.0f, 0.0f, 1.0f };
	static const GLfloat blue[] = { 0.0f, 0.0f, 1.0f, 1.0f };
	static const GLfloat white[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	svec4f result;

	if (TheUITransform.axis == axis)
		result = vload4f(white);
	else {
		if (axis == uiaX)
			result = vload4f(red);
		else if (axis == uiaY)
			result = vload4f(green);
		else if (axis == uiaZ)
			result = vload4f(blue);
	}

	return result;
}

void CUIWidget::maskColorSetGLFront(UITRANSFORMAXIS axis) {
	svec4f color = maskColor(axis);
	glColor4f(color.x, color.y, color.z, color.w);
}

/////////////////////////////////////////////////////////////////
void CMapRotate::createWidget() {
	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);

	m_glList = glGenLists(1);
	glNewList(m_glList, GL_COMPILE);

	//Draw end points
	svec3f v;
	float theta;

	int n = 31;
	svec3f origin(0, 0, 0);

	glPushAttrib(GL_ALL_ATTRIB_BITS);

	//X
	glLineWidth(3.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
	glBegin(GL_LINE_STRIP);
	maskColorSetGLFront(uiaX);

	for (int i = 0; i <= n; i++) {
		theta = static_cast<float>(i) * (TwoPi / (float) n);
		v.x = 0.0f;
		v.y = m_length.y * sin(theta);
		v.z = m_length.z * cos(theta);

		v = vadd3f(v, origin);
		glVertex3f(v.x, v.y, v.z);
	}
	glEnd();

	//Y
	glBegin(GL_LINE_STRIP);
	maskColorSetGLFront(uiaY);
	for (int i = 0; i <= n; i++) {
		theta = static_cast<float>(i) * (TwoPi / (float) n);
		v.x = m_length.x * cos(theta);
		v.y = 0.0f;
		v.z = m_length.z * sin(theta);

		v = vadd3f(v, origin);
		glVertex3f(v.x, v.y, v.z);
	}
	glEnd();

	//Z
	glBegin(GL_LINE_STRIP);
	maskColorSetGLFront(uiaZ);
	for (int i = 0; i <= n; i++) {
		theta = static_cast<float>(i) * (TwoPi / (float) n);
		v.x = m_length.x * cos(theta);
		v.y = m_length.y * sin(theta);
		v.z = 0.0f;

		v = vadd3f(v, origin);
		glVertex3f(v.x, v.y, v.z);
	}
	glEnd();

	glPopAttrib();

	glEndList();
}

CMapRotate::~CMapRotate() {
	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);
}

void CMapRotate::draw() {
	glPushMatrix();
	glTranslatef(m_pos.x, m_pos.y, m_pos.z);
	glCallList(m_glList);
	glPopMatrix();
}

/////////////////////////////////////////////////////////////////
void CMapScale::createWidget() {
	svec3f ptEnd[3];
	svec3f origin = svec3f(0, 0, 0);

	ptEnd[0] = vscale3f(m_length.x, svec3f(1.0f, 0.0f, 0.0f));
	ptEnd[1] = vscale3f(m_length.y, svec3f(0.0f, 1.0f, 0.0f));
	ptEnd[2] = vscale3f(m_length.z, svec3f(0.0f, 0.0f, 1.0f));

	//Octree
	m_axisBoxesLo[0] = svec3f(0, -AXIS_SELECTION_RADIUS,
			-AXIS_SELECTION_RADIUS);
	m_axisBoxesLo[1] = svec3f(-AXIS_SELECTION_RADIUS, 0,
			-AXIS_SELECTION_RADIUS);
	m_axisBoxesLo[2] = svec3f(-AXIS_SELECTION_RADIUS, -AXIS_SELECTION_RADIUS,
			0);

	m_axisBoxesHi[0] = svec3f(ptEnd[0].x, AXIS_SELECTION_RADIUS,
			AXIS_SELECTION_RADIUS);
	m_axisBoxesHi[1] = svec3f(AXIS_SELECTION_RADIUS, ptEnd[1].y,
			AXIS_SELECTION_RADIUS);
	m_axisBoxesHi[2] = svec3f(AXIS_SELECTION_RADIUS, AXIS_SELECTION_RADIUS,
			ptEnd[2].z);

	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);

	m_glList = glGenLists(1);
	glNewList(m_glList, GL_COMPILE);

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(3.0f);
	glBegin(GL_LINES);

	maskColorSetGLFront(uiaX);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(ptEnd[0].x, ptEnd[0].y, ptEnd[0].z);

	maskColorSetGLFront(uiaY);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(ptEnd[1].x, ptEnd[1].y, ptEnd[1].z);

	maskColorSetGLFront(uiaZ);
	glVertex3f(origin.x, origin.y, origin.z);
	glVertex3f(ptEnd[1].x, ptEnd[1].y, ptEnd[1].z);
	glEnd();
	glPopAttrib();

	//Draw end points
	float r = 0.05f;

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//X
	glPushMatrix();
	glTranslated(ptEnd[0].x, ptEnd[0].y, ptEnd[0].z);
	glScalef(r, r, r);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red);
	maskColorSetGLFront(uiaX);
	drawCubePolygon(0, 3, 2, 1);
	drawCubePolygon(4, 5, 6, 7);
	drawCubePolygon(3, 0, 4, 7);
	drawCubePolygon(1, 2, 6, 5);
	drawCubePolygon(2, 3, 7, 6);
	drawCubePolygon(5, 4, 0, 1);
	glPopMatrix();

	//Y
	glPushMatrix();
	glTranslated(ptEnd[1].x, ptEnd[1].y, ptEnd[1].z);
	glScalef(r, r, r);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	maskColorSetGLFront(uiaY);
	drawCubePolygon(0, 3, 2, 1);
	drawCubePolygon(4, 5, 6, 7);
	drawCubePolygon(3, 0, 4, 7);
	drawCubePolygon(1, 2, 6, 5);
	drawCubePolygon(2, 3, 7, 6);
	drawCubePolygon(5, 4, 0, 1);
	glPopMatrix();

	//Z
	glPushMatrix();
	glTranslated(ptEnd[2].x, ptEnd[2].y, ptEnd[2].z);
	glScalef(r, r, r);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	maskColorSetGLFront(uiaZ);
	drawCubePolygon(0, 3, 2, 1);
	drawCubePolygon(4, 5, 6, 7);
	drawCubePolygon(3, 0, 4, 7);
	drawCubePolygon(1, 2, 6, 5);
	drawCubePolygon(2, 3, 7, 6);
	drawCubePolygon(5, 4, 0, 1);
	glPopMatrix();

	glPopAttrib();

	glEndList();
}

CMapScale::~CMapScale() {
	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);
}

void CMapScale::draw() {
	glPushMatrix();
	glTranslatef(m_pos.x, m_pos.y, m_pos.z);
	glCallList(m_glList);
	glPopMatrix();
}

UITRANSFORMAXIS CMapScale::selectAxis(const Ray& ray, float zNear, float zFar) {
	AABB box;

	for (int i = 0; i < 3; i++) {
		box.set(m_axisBoxesLo[i], m_axisBoxesHi[i]);
		box.translate(m_pos);
		if (box.intersect(ray, zNear, zFar)) {
			TheUITransform.axis = (UITRANSFORMAXIS) i;
			this->createWidget();
			return (UITRANSFORMAXIS) i;
		}
	}

	return uiaFree;
}

//////////////////////////////////////////////////////////////////
void CMapTranslate::createWidget() {
	svec3f ptEnd[3];
	svec3f origin(0, 0, 0);

	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);

	m_glList = glGenLists(1);
	glNewList(m_glList, GL_COMPILE);
	ptEnd[0] = vscale3f(m_length.x, svec3f(1.0f, 0.0f, 0.0f));
	ptEnd[1] = vscale3f(m_length.y, svec3f(0.0f, 1.0f, 0.0f));
	ptEnd[2] = vscale3f(m_length.z, svec3f(0.0f, 0.0f, 1.0f));

	//Octree
	m_axisBoxesLo[0] = svec3f(0, -AXIS_SELECTION_RADIUS,
			-AXIS_SELECTION_RADIUS);
	m_axisBoxesLo[1] = svec3f(-AXIS_SELECTION_RADIUS, 0,
			-AXIS_SELECTION_RADIUS);
	m_axisBoxesLo[2] = svec3f(-AXIS_SELECTION_RADIUS, -AXIS_SELECTION_RADIUS,
			0);

	m_axisBoxesHi[0] = svec3f(ptEnd[0].x, AXIS_SELECTION_RADIUS,
			AXIS_SELECTION_RADIUS);
	m_axisBoxesHi[1] = svec3f(AXIS_SELECTION_RADIUS, ptEnd[1].y,
			AXIS_SELECTION_RADIUS);
	m_axisBoxesHi[2] = svec3f(AXIS_SELECTION_RADIUS, AXIS_SELECTION_RADIUS,
			ptEnd[2].z);

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glColor4fv(maskColor(uiaX).ptr());
	glVertex3fv(origin.ptr());
	glVertex3fv(ptEnd[0].ptr());

	glColor4fv(maskColor(uiaY).ptr());
	glVertex3fv(origin.ptr());
	glVertex3fv(ptEnd[1].ptr());

	glColor4fv(maskColor(uiaZ).ptr());
	glVertex3fv(origin.ptr());
	glVertex3fv(ptEnd[2].ptr());
	glEnd();

	//Draw end points
	svec3f v;
	float theta;
	float r = 0.05f;

	//X
	v = ptEnd[0] + svec3f(0.1f, 0.0f, 0.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLE_FAN);
	glColor4fv(maskColor(uiaX).ptr());
	glVertex3fv(v.ptr());
	for (int i = 0; i <= 8; i++) {
		theta = static_cast<float>(i) * (TwoPi / 8.0f);
		v.x = 0.0f;
		v.y = r * sin(theta);
		v.z = r * cos(theta);

		v += ptEnd[0];
		glVertex3fv(v.ptr());
	}
	glEnd();

	//Y
	v = ptEnd[1] + svec3f(0.0f, 0.1f, 0.0f);
	glBegin(GL_TRIANGLE_FAN);
	glColor4fv(maskColor(uiaY).ptr());
	glVertex3fv(v.ptr());
	for (int i = 0; i <= 8; i++) {
		theta = static_cast<float>(i) * (TwoPi / 8.0f);
		v.x = r * cos(theta);
		v.y = 0.0f;
		v.z = r * sin(theta);
		v += ptEnd[1];
		glVertex3fv(v.ptr());
	}
	glEnd();

	//Z
	v = ptEnd[2] + svec3f(0.0f, 0.0f, 0.1f);
	glBegin(GL_TRIANGLE_FAN);
	glColor4fv(maskColor(uiaZ).ptr());
	glVertex3fv(v.ptr());
	for (int i = 0; i <= 8; i++) {
		theta = static_cast<float>(i) * (TwoPi / 8.0f);
		v.x = r * cos(theta);
		v.y = r * sin(theta);
		v.z = 0.0f;

		v += ptEnd[2];
		glVertex3fv(v.ptr());
	}
	glEnd();

	glPopAttrib();

	glEndList();
}

CMapTranslate::~CMapTranslate() {
	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);

}

void CMapTranslate::draw() {
	glPushMatrix();
	glTranslatef(m_pos.x, m_pos.y, m_pos.z);
	glCallList(m_glList);
	glPopMatrix();
}

UITRANSFORMAXIS CMapTranslate::selectAxis(const CRay& ray, float zNear,
		float zFar) {
	COctree oct;

	for (int i = 0; i < 3; i++) {
		oct.set(m_axisBoxesLo[i], m_axisBoxesHi[i]);
		oct.translate(m_pos);
		if (oct.intersect(ray, zNear, zFar)) {
			TheUITransform.axis = (UITRANSFORMAXIS) i;
			this->createWidget();
			return (UITRANSFORMAXIS) i;
		}
	}

	return uiaFree;
}

