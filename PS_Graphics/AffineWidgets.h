#ifndef AFFINEWIDGETS_H
#define AFFINEWIDGETS_H

#include "PS_Vector.h"
#include "PS_Quaternion.h"
#include "PS_Matrix.h"
#include "PS_Ray.h"
#include "loki/Singleton.h"
#include "PS_GLFuncs.h"
#include "SceneGraph.h"

using namespace Loki;
using namespace PS;
using namespace PS::MATH;

#define AXIS_SELECTION_RADIUS 0.2f
enum UITRANSFORMTYPE {uitTranslate, uitRotate, uitScale};
enum UITRANSFORMAXIS {uiaX, uiaY, uiaZ, uiaFree};

struct UITRANSFORM{
    UITRANSFORMTYPE type;
    int    axis;
    vec3f  translate;
    vec3f  scale;
    quat   rotate;
    U32		nStep;
    vec3f mouseDown;
    vec3f mouseMove;
};

typedef SingletonHolder<UITRANSFORM, CreateUsingNew, PhoenixSingleton> TheUITransform;

class AbstractWidget : public SceneNode {
public:    
    AbstractWidget() {
    	m_mtxTranform.identity();
    }
    virtual ~AbstractWidget() {}

    virtual void draw() = 0;
    virtual void createWidget() = 0;

    virtual UITRANSFORMAXIS selectAxis(int x, int y);

    virtual UITRANSFORMAXIS selectAxis(const vec3f& worldpos, const Ray& ray, float zNear, float zFar)
    {
        return uiaFree;
    }

    void  drawCubePolygon(int a, int b, int c, int d);
    vec3f maskDisplacement(vec3f v, UITRANSFORMAXIS axis);
    vec4f maskColor( UITRANSFORMAXIS axis );
    void maskColorSetGLFront( UITRANSFORMAXIS axis );

    mat44f getTransform() const {return m_mtxTranform;}
    void setTransform(const mat44f& mtx) {
    	m_mtxTranform = mtx;
    }
protected:
    mat44f m_mtxTranform;
};


/*!
  * Creates a rotation Widget
  */
class RotationWidget : public AbstractWidget
{
public:
    RotationWidget()
    {
    	m_mtxTranform.identity();
        this->createWidget();
    }

    virtual ~RotationWidget();


    void draw();
    void createWidget();

private:
    U32 m_glList;
};

/*!
 * Create scale Graph
 */
class ScaleWidget : public AbstractWidget
{
public:
    ScaleWidget()
    {
    	this->m_mtxTranform.identity();
        this->createWidget();
    }

    virtual ~ScaleWidget();



    void draw();
    void createWidget();
    UITRANSFORMAXIS selectAxis(const vec3f& worldpos, const Ray& ray, float zNear, float zFar);

private:
    U32 m_glList;
    vec3f m_axisBoxesLo[3];
    vec3f m_axisBoxesHi[3];
};



/*!
  * CMapTranslate Shows a translation map on GUI
  */
class TranslateWidget : public AbstractWidget
{
public:
    TranslateWidget()
    {
    	this->m_mtxTranform.identity();
        this->createWidget();
    }

    virtual ~TranslateWidget();


    void draw();
    void createWidget();
    UITRANSFORMAXIS selectAxis(const vec3f& worldpos, const Ray& ray, float zNear, float zFar);


private:
    U32 m_vbo;


    U32 m_glList;
    vec3f m_axisBoxesLo[3];
    vec3f m_axisBoxesHi[3];
};

AbstractWidget* CreateAffineWidget(UITRANSFORMTYPE type);

#endif
