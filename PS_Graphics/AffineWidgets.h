#ifndef AFFINEWIDGETS_H
#define AFFINEWIDGETS_H

#include "PS_Vector.h"
#include "PS_Quaternion.h"
#include "PS_Ray.h"
#include "loki/Singleton.h"

using namespace Loki;
using namespace PS;
using namespace PS::MATH;
using namespace PS::FUNCTIONALMATH;

#define AXIS_SELECTION_RADIUS 0.2f
enum UITRANSFORMTYPE {uitTranslate, uitRotate, uitScale};
enum UITRANSFORMAXIS {uiaX, uiaY, uiaZ, uiaFree, uiaCount};

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

class AbstractWidget{
public:    
    AbstractWidget() {}
    virtual ~AbstractWidget() {}

    virtual void draw() = 0;
    virtual void createWidget() = 0;
    virtual UITRANSFORMAXIS selectAxis(const Ray& ray, float zNear, float zFar)
    {
        return uiaFree;
    }

    void  drawCubePolygon(int a, int b, int c, int d);
    vec3f maskDisplacement(vec3f v, UITRANSFORMAXIS axis);
    vec4f maskColor( UITRANSFORMAXIS axis );
    void maskColorSetGLFront( UITRANSFORMAXIS axis );


    void setPos(const vec3f& pos) {m_pos = pos;}
    vec3f getPos() const {return m_pos;}

    void setLength(const vec3f& len) {m_length = len;}
    vec3f getLength() const {return m_length;}
protected:
    vec3f m_pos;
    vec3f m_length;
};


/*!
  * Creates a rotation Widget
  */
class RotationWidget : public AbstractWidget
{
public:
    RotationWidget()
    {
        m_pos = vec3f(0,0,0);
        m_length = vec3f(0.5f, 0.5f, 0.5f);
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
        m_pos = vec3f(0,0,0);
        m_length = vec3f(1,1,1);
        this->createWidget();
    }

    virtual ~ScaleWidget();



    void draw();
    void createWidget();
    UITRANSFORMAXIS selectAxis(const Ray& ray, float zNear, float zFar);
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
        m_pos = vec3f(0,0,0);
        m_length = vec3f(1,1,1);
        this->createWidget();
    }

    virtual ~TranslateWidget();


    void draw();
    void createWidget();
    UITRANSFORMAXIS selectAxis(const Ray& ray, float zNear, float zFar);


private:
    U32 m_vbo;


    U32 m_glList;
    vec3f m_axisBoxesLo[3];
    vec3f m_axisBoxesHi[3];
};

#endif
