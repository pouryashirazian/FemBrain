#include "testLib.h"
#include <iostream>
#include "PS_Matrix.h"
#include "PS_Vector.h"
//#include <QMatrix4x4>
#include <stdio.h>
using namespace std;
using namespace PS::MATH;

void VecPrint(const vec3f& v){
    printf("[%.2f, %.2f, %.2f] \n", v.x, v.y, v.z);
}

void VecPrint(const vec4f& v){
    printf("[%.2f, %.2f, %.2f, %.2f] \n", v.x, v.y, v.z, v.w);
}

void VecPrint(const vec4d& v){
    printf("[%.2f, %.2f, %.2f, %.2f] \n", v.x, v.y, v.z, v.w);
}
/*
void QMtxPrint(const QMatrix4x4& m)
{
    const double* data = m.data();
    for(int iRow=0; iRow<4; iRow++)
    {
        for(int iCol=0; iCol<4; iCol++)
        {
            //Print iRow
            printf("%.2f, ", data[iCol * 4 + iRow]);
        }
        printf("\n");
    }
}
*/

void runTests()
{    
    //Vector tests
    //Print Vectors
    printf("Testing vector lib: \n");
    vec3f v(1,2,0);
    VecPrint(v);

    printf("Set z coordinate to 3.\n");
    v.e[2] = 3;
    VecPrint(v);

    printf("Normalize vector.\n");
    v.normalize();
    VecPrint(v);

    vec3f a(1, 0, 0);
    vec3f b(0, 1, 0);
    printf("a and b are 3d vectors=\n");
    VecPrint(a); VecPrint(b);

    printf("a dot b = %.2f \n", vec3f::dot(a, b));

    vec3f c = vec3f::cross(a, b);
    printf("a cross b = c = ");
    VecPrint(c);

    printf("Reflect a over b = \n");
    VecPrint(vec3f::reflect(a, b));


    printf("Angle Degree between a and b = %.2f \n", vec3f::angleDeg(a, b));
    printf("Distance between a and b = %.2f \n", vec3f::distance(a, b));

    printf("Min Point between a, b, c = ");
    VecPrint(vec3f::minP(a, vec3f::minP(b, c)));

    printf("Max Point between a, b, c = ");
    VecPrint(vec3f::maxP(a, vec3f::maxP(b, c)));

    //Matrix Tests
    printf("Testing matrix lib: \n");
    double arrV[] = {0,1,2,3,
                   4,5,6,7,
                   8,9,10,11,
                   0,0,0,0};

    mat44d m(arrV);
    MtxPrint(m);
    for(int i=0; i<4; i++)
    {
        printf("Col %d = ", i);
        VecPrint(m.getCol(i));
    }

    for(int i=0; i<4; i++)
    {
        printf("Row %d = ", i);
        VecPrint(m.getRow(i));
    }

    printf("Diagonal = ");
    VecPrint(m.getDiag());

    printf("Transposed = \n");
    MtxPrint(m.transposed());

    /*
    printf("SubMatrix[0,0] = \n");
    MtxPrint(m.subMtx(0, 0));

    printf("SubMatrix[0,1] = \n");
    MtxPrint(m.subMtx(0, 1));

    printf("SubMatrix[1,0] = \n");
    MtxPrint(m.subMtx(1, 0));

    printf("SubMatrix[1,1] = \n");
    MtxPrint(m.subMtx(1, 1));
    */

    printf("Determinant = %.2f \n", m.determinant());

    printf("Inverted = \n");
    MtxPrint(m.inverted());


    /*
    printf("Ground Truth = \n");
    QMatrix4x4 gt(arrV);
    QMtxPrint(gt);
    printf("Testing vector lib: \n");

    printf("Ground Truth Determinant = %.2f\n", gt.determinant());

    printf("Ground Truth Inverted = \n");
    QMatrix4x4 gtInv = gt.inverted();
    QMtxPrint(gtInv);
	*/
}
