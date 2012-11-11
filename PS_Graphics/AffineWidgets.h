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
enum UITRANSFORMAXIS {uiaX, uiaY, uiaZ, uiaFree};

struct UITRANSFORM{
    UITRANSFORMTYPE type;
    int    axis;
    vec3  translate;
    vec3  scale;
    quat   rotate;
    U32		nStep;
    vec3 mouseDown;
    vec3 mouseMove;
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
    vec3 maskDisplacement(vec3 v, UITRANSFORMAXIS axis);
    vec4 maskColor( UITRANSFORMAXIS axis );
    void maskColorSetGLFront( UITRANSFORMAXIS axis );


    void setPos(const vec3& pos) {m_pos = pos;}
    vec3 getPos() const {return m_pos;}

    void setLength(const vec3& len) {m_length = len;}
    vec3 getLength() const {return m_length;}
protected:
    vec3 m_pos;
    vec3 m_length;
};


/*!
  * Creates a rotation Widget
  */
class RotationWidget : public AbstractWidget
{
public:
    RotationWidget()
    {
        m_pos = vec3(0,0,0);
        m_length = vec3(0.5f, 0.5f, 0.5f);
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
        m_pos = vec3(0,0,0);
        m_length = vec3(1,1,1);
        this->createWidget();
    }

    virtual ~ScaleWidget();



    void draw();
    void createWidget();
    UITRANSFORMAXIS selectAxis(const Ray& ray, float zNear, float zFar);
private:
    U32 m_glList;
    vec3 m_axisBoxesLo[3];
    vec3 m_axisBoxesHi[3];
};



/*!
  * CMapTranslate Shows a translation map on GUI
  */
class TranslateWidget : public AbstractWidget
{
public:
    TranslateWidget()
    {
        m_pos = vec3(0,0,0);
        m_length = vec3(1,1,1);
        this->createWidget();
    }

    virtual ~TranslateWidget();


    void draw();
    void createWidget();
    UITRANSFORMAXIS selectAxis(const Ray& ray, float zNear, float zFar);


private:
    U32 m_vbo;


    U32 m_glList;
    vec3 m_axisBoxesLo[3];
    vec3 m_axisBoxesHi[3];
};

#endif
