/*
 * PS_Quaternion.cpp
 *
 *  Created on: Oct 29, 2012
 *      Author: pourya
 */
#include "PS_Quaternion.h"

namespace PS{
namespace MATH{

void Quaternion::normalize()
{
    float	dist, square;

    square = q.x * q.x + q.y * q.y + q.z * q.z + w * w;
    if (square > 0.0f)
        dist = (float)(1.0f / sqrt(square));
    else
        dist = 1;

    q.x *= dist;
    q.y *= dist;
    q.z *= dist;
    w *= dist;
}

void Quaternion::fromMatrix(const mat44& mtx)
{
    float tr = mtx.e[0][0] + mtx.e[1][1] + mtx.e[2][2];
    // check the diagonal

    if (tr > 0.0f )
    {
        float s = (float) sqrt ( (double) (tr + 1.0f) );
        w = s * 0.5f;
        s = 0.5f / s;
        q.x = (mtx.e[1][2] - mtx.e[2][1]) * s;
        q.y = (mtx.e[2][0] - mtx.e[0][2]) * s;
        q.z = (mtx.e[0][1] - mtx.e[1][0]) * s;

    }
    else
    {
        // diagonal is negative
        int nxt[3] = {1, 2, 0};
        float  qa[4];

        int i = 0;

        if (mtx.e[1][1] > mtx.e[0][0]) i = 1;
        if (mtx.e[2][2] > mtx.e[i][i]) i = 2;

        int j = nxt[i];
        int k = nxt[j];

        float s = sqrt ( ((mtx.e[i][i] - (mtx.e[j][j] + mtx.e[k][k])) + 1.0f) );

        qa[i] = s * 0.5f;

        if (s != 0.0f ) s = 0.5f / s;

        qa[3] = (mtx.e[j][k] - mtx.e[k][j]) * s;
        qa[j] = (mtx.e[i][j] + mtx.e[j][i]) * s;
        qa[k] = (mtx.e[i][k] + mtx.e[k][i]) * s;

        q.x = qa[0];
        q.y = qa[1];
        q.z = qa[2];
        w   = qa[3];
    }
}

// quaternion must be normalized and matrix t in column-major format
void Quaternion::toMatrix(mat44& mtx) const
{
    float xx = q.x*q.x;
    float yy = q.y*q.y;
    float zz = q.z*q.z;
    float xy = q.x*q.y;
    float xz = q.x*q.z;
    float yz = q.y*q.z;
    float wx = w*q.x;
    float wy = w*q.y;
    float wz = w*q.z;

    mtx.e[0][0] = 1 - 2 * ( yy + zz );
    mtx.e[1][0] =     2 * ( xy - wz );
    mtx.e[2][0] =     2 * ( xz + wy );

    mtx.e[0][1] =     2 * ( xy + wz );
    mtx.e[1][1] = 1 - 2 * ( xx + zz );
    mtx.e[2][1] =     2 * ( yz - wx );

    mtx.e[0][2] =     2 * ( xz - wy );
    mtx.e[1][2] =     2 * ( yz + wx );
    mtx.e[2][2] = 1 - 2 * ( xx + yy );

    mtx.e[3][0] = mtx.e[3][1] = mtx.e[3][2] = 0.0f;
    mtx.e[0][3] = mtx.e[1][3] = mtx.e[2][3] = 0.0f;
    mtx.e[3][3] = 1.0f;
}

// build quaternion based on euler angles
void Quaternion::fromEuler(float roll,float pitch,float yaw)
{
    roll  *= 0.5f;
    pitch *= 0.5f;
    yaw   *= 0.5f;

    float cr = (float)cos(roll);
    float cp = (float)cos(pitch);
    float cy = (float)cos(yaw);

    float sr = (float)sin(roll);
    float sp = (float)sin(pitch);
    float sy = (float)sin(yaw);

    float cpcy = cp * cy;
    float spsy = sp * sy;
    float spcy = sp * cy;
    float cpsy = cp * sy;

    w   = cr * cpcy + sr * spsy;
    q.x = sr * cpcy - cr * spsy;
    q.y = cr * spcy + sr * cpsy;
    q.z = cr * cpsy - sr * spcy;
}


void Quaternion::toEuler( float& roll, float& pitch, float& yaw ) const
{
    float sint		= (2.0f * w * q.y) - (2.0f * q.x * q.z);
    float cost_temp = 1.0f - (sint * sint);
    float cost		= 0;

    if ( (float)fabs(cost_temp) > 0.001f )
    {
        cost = sqrt( cost_temp );
    }

    float sinv, cosv, sinf, cosf;
    if ( (float)fabs(cost) > 0.001f )
    {
        sinv = ((2.0f * q.y * q.z) + (2.0f * w * q.x)) / cost;
        cosv = (1.0f - (2.0f * q.x * q.x) - (2.0f * q.y * q.y)) / cost;
        sinf = ((2.0f * q.x * q.y) + (2.0f * w * q.z)) / cost;
        cosf = (1.0f - (2.0f * q.y * q.y) - (2.0f * q.z * q.z)) / cost;
    }
    else
    {
        sinv = (2.0f * w * q.x) - (2.0f * q.y * q.z);
        cosv = 1.0f - (2.0f * q.x * q.x) - (2.0f * q.z * q.z);
        sinf = 0;
        cosf = 1.0f;
    }

    // compute output rotations
    roll	= atan2( sinv, cosv );
    pitch	= atan2( sint, cost );
    yaw		= atan2( sinf, cosf );
}

}
}



