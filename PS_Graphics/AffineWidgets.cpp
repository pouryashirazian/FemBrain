#include "AffineWidgets.h"
#include "PS_Box.h"
#include <GL/glew.h>

GLfloat g_vertices[][3] = { { -1.0, -1.0, 1.0 }, { -1.0, 1.0, 1.0 }, { 1.0, 1.0,
		1.0 }, { 1.0, -1.0, 1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, 1.0, -1.0 }, {
		1.0, 1.0, -1.0 }, { 1.0, -1.0, -1.0 } };

UITRANSFORMAXIS AbstractWidget::selectAxis(int x, int y)
{
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);

	vec3d posNear(x, vp[3] - y, 0.0f);
	vec3d posFar(x, vp[3] - y, 1.0f);
	vec3d posTransNear;
	vec3d posTransFar;


	ScreenToWorld(posNear, posTransNear);
	ScreenToWorld(posFar, posTransFar);
	vec3d dir = posTransFar - posTransNear;
	dir.normalize();
	Ray ray(posTransNear, dir);

	return this->selectAxis(TheUITransform::Instance().translate, ray, 0.01, 1000.0);
}

vec3f AbstractWidget::maskDisplacement(vec3f v, UITRANSFORMAXIS axis) {
	if (axis == uiaFree)
		return v;
	else if (axis == uiaX)
		return vec3f(v.x, 0.0f, 0.0f);
	else if (axis == uiaY)
		return vec3f(0.0f, v.y, 0.0f);
	else if (axis == uiaZ)
		return vec3f(0.0f, 0.0f, v.z);
	else
		return v;
}

void AbstractWidget::drawCubePolygon(int a, int b, int c, int d) {
	glBegin(GL_POLYGON);
	glVertex3fv(g_vertices[a]);
	glVertex3fv(g_vertices[b]);
	glVertex3fv(g_vertices[c]);
	glVertex3fv(g_vertices[d]);
	glEnd();
}

vec4f AbstractWidget::maskColor(UITRANSFORMAXIS axis) {
	static const GLfloat red[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	static const GLfloat green[] = { 0.0f, 1.0f, 0.0f, 1.0f };
	static const GLfloat blue[] = { 0.0f, 0.0f, 1.0f, 1.0f };
	static const GLfloat black[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	vec4f result;

	if (TheUITransform::Instance().axis == axis)
		result = vec4f(black);
	else {
		if (axis == uiaX)
			result = vec4f(red);
		else if (axis == uiaY)
			result = vec4f(green);
		else if (axis == uiaZ)
			result = vec4f(blue);
	}

	return result;
}

void AbstractWidget::maskColorSetGLFront(UITRANSFORMAXIS axis) {
	vec4f color = maskColor(axis);
	glColor4f(color.x, color.y, color.z, color.w);
}

/////////////////////////////////////////////////////////////////
void RotationWidget::createWidget() {
	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);

	m_glList = glGenLists(1);
	glNewList(m_glList, GL_COMPILE);

	//Draw end points
	vec3f v;
	float theta;

	int n = 31;
	vec3f origin(0, 0, 0);
	vec3f vlen(1,1,1);

	glPushAttrib(GL_ALL_ATTRIB_BITS);

	//X
	glLineWidth(3.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
	glBegin(GL_LINE_STRIP);
	maskColorSetGLFront(uiaX);

	for (int i = 0; i <= n; i++) {
		theta = static_cast<float>(i) * (TwoPi / (float) n);
		v.x = 0.0f;
		v.y = vlen.y * sin(theta);
		v.z = vlen.z * cos(theta);

		v = v + origin;
		glVertex3f(v.x, v.y, v.z);
	}
	glEnd();

	//Y
	glBegin(GL_LINE_STRIP);
	maskColorSetGLFront(uiaY);
	for (int i = 0; i <= n; i++) {
		theta = static_cast<float>(i) * (TwoPi / (float) n);
		v.x = vlen.x * cos(theta);
		v.y = 0.0f;
		v.z = vlen.z * sin(theta);

		v = v + origin;
		glVertex3f(v.x, v.y, v.z);
	}
	glEnd();

	//Z
	glBegin(GL_LINE_STRIP);
	maskColorSetGLFront(uiaZ);
	for (int i = 0; i <= n; i++) {
		theta = static_cast<float>(i) * (TwoPi / (float) n);
		v.x = vlen.x * cos(theta);
		v.y = vlen.y * sin(theta);
		v.z = 0.0f;

		v = v + origin;
		glVertex3f(v.x, v.y, v.z);
	}
	glEnd();

	glPopAttrib();

	glEndList();
}

RotationWidget::~RotationWidget() {
	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);
}

void RotationWidget::draw() {
	glPushMatrix();
	glMultMatrixf(m_mtxTranform.cptr());
	glCallList(m_glList);
	glPopMatrix();
}

/////////////////////////////////////////////////////////////////
void ScaleWidget::createWidget() {
	vec3f ptEnd[3];
	vec3f origin = vec3f(0, 0, 0);

	ptEnd[0] = vec3f(1.0f, 0.0f, 0.0f);
	ptEnd[1] = vec3f(0.0f, 1.0f, 0.0f);
	ptEnd[2] = vec3f(0.0f, 0.0f, 1.0f);

	//Octree
	m_axisBoxesLo[0] = vec3f(0, -AXIS_SELECTION_RADIUS,
			-AXIS_SELECTION_RADIUS);
	m_axisBoxesLo[1] = vec3f(-AXIS_SELECTION_RADIUS, 0,
			-AXIS_SELECTION_RADIUS);
	m_axisBoxesLo[2] = vec3f(-AXIS_SELECTION_RADIUS, -AXIS_SELECTION_RADIUS,
			0);

	m_axisBoxesHi[0] = vec3f(ptEnd[0].x, AXIS_SELECTION_RADIUS,
			AXIS_SELECTION_RADIUS);
	m_axisBoxesHi[1] = vec3f(AXIS_SELECTION_RADIUS, ptEnd[1].y,
			AXIS_SELECTION_RADIUS);
	m_axisBoxesHi[2] = vec3f(AXIS_SELECTION_RADIUS, AXIS_SELECTION_RADIUS,
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
	glVertex3f(ptEnd[2].x, ptEnd[2].y, ptEnd[2].z);
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

ScaleWidget::~ScaleWidget() {
	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);
}

void ScaleWidget::draw() {
	glPushMatrix();
	glMultMatrixf(m_mtxTranform.cptr());
	glCallList(m_glList);
	glPopMatrix();
}

UITRANSFORMAXIS ScaleWidget::selectAxis(const vec3f& worldpos, const Ray& ray, float zNear, float zFar)
{
	AABB box;

	for (int i = 0; i < 3; i++) {
		box.set(m_axisBoxesLo[i], m_axisBoxesHi[i]);
		box.tranform(m_mtxTranform);
		box.translate(worldpos);

		if (box.intersect(ray, zNear, zFar)) {
			TheUITransform::Instance().axis = (UITRANSFORMAXIS) i;
			this->createWidget();
			return (UITRANSFORMAXIS) i;
		}
	}

	return uiaFree;
}

//////////////////////////////////////////////////////////////////
void TranslateWidget::createWidget() {
/*
	vec3f ptEnd[3];
	ptEnd[0] = vec3f(1.0f, 0.0f, 0.0f);
	ptEnd[1] = vec3f(0.0f, 1.0f, 0.0f);
	ptEnd[2] = vec3f(0.0f, 0.0f, 1.0f);

	std::vector<float> arrVertices;
	std::vector<float> arrColors;

	float r = 0.2;
	vec4f clX = maskColor(uiaX);
	for(int i=0; i<8; i++)
	{
		float theta = static_cast<float>(i) * (TwoPi / 8.0f);
		vec3f v1, v2;
		v1.x = 0.0f;
		v1.y = r * sin(theta);
		v1.z = r * cos(theta);

		v2.x = 1.0f;
		v2.y = r * sin(theta);
		v2.z = r * cos(theta);

		arrVertices
	}
*/

	//X
	/*
	vec3f ptEnd[3];
	vec3f v;
	float theta;
	float r = 0.2;
	v = ptEnd[0] + vec3f(0.1f, 0.0f, 0.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaX).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = 0.0f;
			v.y = r * sin(theta);
			v.z = r * cos(theta);

			v = v + ptEnd[0];
			glVertex3fv(v.ptr());
		}
	glEnd();

	//Y
	v = ptEnd[1] + vec3f(0.0f, 0.1f, 0.0f);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaY).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = r * cos(theta);
			v.y = 0.0f;
			v.z = r * sin(theta);
			v = v + ptEnd[1];
			glVertex3fv(v.ptr());
		}
	glEnd();

	//Z
	v = ptEnd[2] + vec3f(0.0f, 0.0f, 0.1f);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaZ).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = r * cos(theta);
			v.y = r * sin(theta);
			v.z = 0.0f;

			v = v + ptEnd[2];
			glVertex3fv(v.ptr());
		}
	glEnd();

*/

	vec3f ptEnd[3];
	vec3f origin(0, 0, 0);

	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);

	m_glList = glGenLists(1);
	glNewList(m_glList, GL_COMPILE);
	ptEnd[0] = vec3f(1.0f, 0.0f, 0.0f);
	ptEnd[1] = vec3f(0.0f, 1.0f, 0.0f);
	ptEnd[2] = vec3f(0.0f, 0.0f, 1.0f);

	//Octree
	m_axisBoxesLo[0] = vec3f(0, -AXIS_SELECTION_RADIUS,
			-AXIS_SELECTION_RADIUS);
	m_axisBoxesLo[1] = vec3f(-AXIS_SELECTION_RADIUS, 0,
			-AXIS_SELECTION_RADIUS);
	m_axisBoxesLo[2] = vec3f(-AXIS_SELECTION_RADIUS, -AXIS_SELECTION_RADIUS,
			0);

	m_axisBoxesHi[0] = vec3f(ptEnd[0].x, AXIS_SELECTION_RADIUS,
			AXIS_SELECTION_RADIUS);
	m_axisBoxesHi[1] = vec3f(AXIS_SELECTION_RADIUS, ptEnd[1].y,
			AXIS_SELECTION_RADIUS);
	m_axisBoxesHi[2] = vec3f(AXIS_SELECTION_RADIUS, AXIS_SELECTION_RADIUS,
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
	vec3f v;
	float theta;
	float r = 0.05f;

	//X
	v = ptEnd[0] + vec3f(0.1f, 0.0f, 0.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaX).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = 0.0f;
			v.y = r * sin(theta);
			v.z = r * cos(theta);

			v = v + ptEnd[0];
			glVertex3fv(v.ptr());
		}
	glEnd();

	//Y
	v = ptEnd[1] + vec3f(0.0f, 0.1f, 0.0f);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaY).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = r * cos(theta);
			v.y = 0.0f;
			v.z = r * sin(theta);
			v = v + ptEnd[1];
			glVertex3fv(v.ptr());
		}
	glEnd();

	//Z
	v = ptEnd[2] + vec3f(0.0f, 0.0f, 0.1f);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaZ).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = r * cos(theta);
			v.y = r * sin(theta);
			v.z = 0.0f;

			v = v + ptEnd[2];
			glVertex3fv(v.ptr());
		}
	glEnd();

	glPopAttrib();

	glEndList();
}

TranslateWidget::~TranslateWidget()
{
	if (glIsList(m_glList))
		glDeleteLists(m_glList, 1);
}

void TranslateWidget::draw() {
	glPushMatrix();
	glMultMatrixf(m_mtxTranform.cptr());
	glCallList(m_glList);
	glPopMatrix();

/*
	vec3f ptEnd[3];
	vec3f origin(0, 0, 0);
	ptEnd[0] = vec3f(1.0f, 0.0f, 0.0f);
	ptEnd[1] = vec3f(0.0f, 1.0f, 0.0f);
	ptEnd[2] = vec3f(0.0f, 0.0f, 1.0f);


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
	vec3f v;
	float theta;
	float r = 0.05f;

	//X
	v = ptEnd[0] + vec3f(0.1f, 0.0f, 0.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaX).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = 0.0f;
			v.y = r * sin(theta);
			v.z = r * cos(theta);

			v = v + ptEnd[0];
			glVertex3fv(v.ptr());
		}
	glEnd();

	//Y
	v = ptEnd[1] + vec3f(0.0f, 0.1f, 0.0f);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaY).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = r * cos(theta);
			v.y = 0.0f;
			v.z = r * sin(theta);
			v = v + ptEnd[1];
			glVertex3fv(v.ptr());
		}
	glEnd();

	//Z
	v = ptEnd[2] + vec3f(0.0, 0.0, 0.1);
	glBegin(GL_TRIANGLE_FAN);
		glColor4fv(maskColor(uiaZ).ptr());
		glVertex3fv(v.ptr());
		for (int i = 0; i <= 8; i++) {
			theta = static_cast<float>(i) * (TwoPi / 8.0f);
			v.x = r * cos(theta);
			v.y = r * sin(theta);
			v.z = 0.0f;

			v = v + ptEnd[2];
			glVertex3fv(v.ptr());
		}
	glEnd();

	glPopAttrib();
	*/
}

UITRANSFORMAXIS TranslateWidget::selectAxis(const vec3f& worldpos, const Ray& ray, float zNear, float zFar)
{
	AABB box;

	for (int i = 0; i < 3; i++)
	{
		box.set(m_axisBoxesLo[i], m_axisBoxesHi[i]);
		box.tranform(m_mtxTranform);
		box.translate(worldpos);

		if (box.intersect(ray, zNear, zFar)) {
			TheUITransform::Instance().axis = (UITRANSFORMAXIS) i;
			this->createWidget();
			return (UITRANSFORMAXIS) i;
		}
	}

	return uiaFree;
}

AbstractWidget* CreateAffineWidget(UITRANSFORMTYPE type)
{
	AbstractWidget* lpWidget = NULL;
	switch(type)
	{
	case(uitTranslate):
	{
		lpWidget = new TranslateWidget();
		break;
	}
	case(uitScale):
	{
		lpWidget = new ScaleWidget();
		break;
	}

	case(uitRotate):
	{
		lpWidget = new RotationWidget();
		break;
	}
	}
	TheUITransform::Instance().type = type;
	return lpWidget;
}
