#ifndef AFFINEWIDGETS_H
#define AFFINEWIDGETS_H

#include "PS_VectorMath.h"
#include "PS_Quaternion.h"
#include "PS_Ray.h"

//using namespace Loki;
using namespace PS;
using namespace PS::MATH;
using namespace PS::FUNCTIONALMATH;

#define AXIS_SELECTION_RADIUS 0.2f
enum UITRANSFORMTYPE {uitTranslate, uitRotate, uitScale};
enum UITRANSFORMAXIS {uiaX, uiaY, uiaZ, uiaFree};

struct UITRANSFORM{
    UITRANSFORMTYPE type;
    UITRANSFORMAXIS axis;
    svec3f	translate;
    svec3f  scale;
    quat	rotate;
    U32		nStep;
    svec3f mouseDown;
    svec3f mouseMove;
} TheUITransform;

//typedef SingletonHolder<UITRANSFORM, CreateUsingNew, PhoenixSingleton> TheUITransform;

class CUIWidget{
public:    
    CUIWidget() {}
    virtual ~CUIWidget() {}

    virtual void draw() = 0;
    virtual void createWidget() = 0;
    virtual UITRANSFORMAXIS selectAxis(const Ray& ray, float zNear, float zFar)
    {
        return uiaFree;
    }

    void  drawCubePolygon(int a, int b, int c, int d);
    svec3f maskDisplacement(svec3f v, UITRANSFORMAXIS axis);
    svec4f maskColor( UITRANSFORMAXIS axis );
    void maskColorSetGLFront( UITRANSFORMAXIS axis );


    void setPos(const svec3f& pos) {m_pos = pos;}
    svec3f getPos() const {return m_pos;}

    void setLength(const svec3f& len) {m_length = len;}
    svec3f getLength() const {return m_length;}
protected:
    svec3f m_pos;
    svec3f m_length;
};


/*!
  * Creates a rotation Widget
  */
class CMapRotate : public CUIWidget
{
public:
    CMapRotate()
    {
        m_pos = svec3f(0,0,0);
        m_length = svec3f(0.5f, 0.5f, 0.5f);
        this->createWidget();
    }

    virtual ~CMapRotate();



    void draw();
    void createWidget();

private:
    U32 m_glList;
};

/*!
 * Create scale Graph
 */
class CMapScale : public CUIWidget
{
public:
    CMapScale()
    {
        m_pos = svec3f(0,0,0);
        m_length = svec3f(1,1,1);
        this->createWidget();
    }

    virtual ~CMapScale();



    void draw();
    void createWidget();
    UITRANSFORMAXIS selectAxis(const Ray& ray, float zNear, float zFar);
private:
    U32 m_glList;
    svec3f m_axisBoxesLo[3];
    svec3f m_axisBoxesHi[3];
};



/*!
  * CMapTranslate Shows a translation map on GUI
  */
class CMapTranslate : public CUIWidget
{
public:
    CMapTranslate()
    {
        m_pos = svec3f(0,0,0);
        m_length = svec3f(1,1,1);
        this->createWidget();
    }

    virtual ~CMapTranslate();


    void draw();
    void createWidget();
    UITRANSFORMAXIS selectAxis(const Ray& ray, float zNear, float zFar);

private:
    U32 m_glList;
    svec3f m_axisBoxesLo[3];
    svec3f m_axisBoxesHi[3];
};

#endif
