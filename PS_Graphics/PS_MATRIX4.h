/*
 * PS_MATRIX4.h
 *
 *  Created on: 2011-10-19
 *      Author: pourya
 */

#ifndef PS_MATRIX4_H_
#define PS_MATRIX4_H_

#include "PS_VectorMath.h"
#include <cstring>

namespace PS{
namespace FUNCTIONALMATH{


template<typename Type, int COLS, int ROWS>
struct MAT
{
	Type e[COLS][ROWS];

	MAT()	{ identity();}

	MAT(const Type* lpValues)
	{
		memcpy(e, lpValues, sizeof(Type)* COLS * ROWS);
	}

	MAT(const MAT& rhs)
	{
		memcpy(e, rhs.e, sizeof(Type)* COLS * ROWS);
	}

	const Type* cptr() {return &e[0][0];}

	void identity()
	{
		for(int i=0; i<COLS; i++)
		{
			for(int j=0; j<ROWS; j++)
			{
				if(i==j)
					e[i][j] = 1;
				else
					e[i][j] = 0;
			}
		}
	}

    inline MAT operator =(const MAT& rhs)
    {
        memcpy(e, rhs.e, sizeof (Type) * COLS * ROWS);
        return (*this);
    }
};

typedef MAT<float, 4, 4> MAT44;


inline bool mat4IsEqual(const MAT44& m1, const MAT44& m2)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
		{
			if(m1.e[i][j] != m2.e[i][j])
				return false;
		}
	return true;
}

inline bool mat4IsIdentity(const MAT44& m)
{
	MAT44 ident;
	return mat4IsEqual(ident, m);
}

inline MAT44 mat4Mult(const MAT44& m1, const MAT44& m2)
{
	/*
	MAT44 result;
	const float* pA = m1.e;
	const float* pB = m2.e;
	float* pM = result.e;

	memset(pM, 0, sizeof(float)*16);

	for(int i=0; i<4; i++ )
		for(int j=0; j<4; j++ )
			for(int k=0; k<4; k++ )
				pM[4*i+j] +=  pA[4*i+k] * pB[4*k+j];
				*/

	MAT44 result;
	memset(result.e, 0, sizeof(float)*16);
	for(int i=0; i<4; i++ )
		for(int j=0; j<4; j++ )
			for(int k=0; k<4; k++ )
				result.e[i][j] += m1.e[i][k] * m2.e[k][j];
	return result;
}

inline void mat4FromQuat(const svec4f& quat, MAT44& result)
{
	float xx = quat.x * quat.x;
	float yy = quat.y * quat.y;
	float zz = quat.z * quat.z;
	float xy = quat.x * quat.y;
	float xz = quat.x * quat.z;
	float yz = quat.y * quat.z;
	float wx = quat.w * quat.x;
	float wy = quat.w * quat.y;
	float wz = quat.w * quat.z;

	result.e[0][0] = 1 - 2 * ( yy + zz );
	result.e[1][0] =     2 * ( xy - wz );
	result.e[2][0] =     2 * ( xz + wy );

	result.e[0][1] =     2 * ( xy + wz );
	result.e[1][1] = 1 - 2 * ( xx + zz );
	result.e[2][1] =     2 * ( yz - wx );

	result.e[0][2] =     2 * ( xz - wy );
	result.e[1][2] =     2 * ( yz + wx );
	result.e[2][2] = 1 - 2 * ( xx + yy );

	result.e[3][0] = result.e[3][1] = result.e[3][2] = 0.0f;
	result.e[0][3] = result.e[1][3] = result.e[2][3] = 0.0f;
	result.e[3][3] = 1.0f;
}


inline float mat4Determinant(const MAT44& mat)
{
    svec3f p0(mat.e[0][0], mat.e[0][1], mat.e[0][2]);
    svec3f p1(mat.e[1][0], mat.e[1][1], mat.e[1][2]);
    svec3f p2(mat.e[2][0], mat.e[2][1], mat.e[2][2]);

    svec3f c12 = vcross3f(p1, p2);
    return vdot3f(p0, c12);
}

inline void mat4SubMat(const MAT44& input, int ki, int kj, MAT44& output)
{
    int row, col;
    int dstCol = 0, dstRow = 0;
    for(col = 0;col < 4;col++)
    {
        if(col == kj){
            continue;
        }
        for(dstRow = 0, row = 0;row < 4;row++)
        {
            if(row == ki){
            	continue;
            }
            output.e[dstCol][dstRow] = input.e[col][row];
            dstRow++;
        }
        dstCol++;
    }
}


inline void mat4Translate(const svec3f& t, MAT44& mat)
{
	mat.e[3][0] = t.x;
	mat.e[3][1] = t.y;
	mat.e[3][2] = t.z;
}

inline void mat4Scale(const svec3f& s, MAT44& mat)
{
	MAT44 work;
	work.e[0][0] = s.x;
	work.e[1][1] = s.y;
	work.e[2][2] = s.z;

	mat = mat4Mult(mat, work);
}

inline void mat4Invert(const MAT44& input, MAT44& invert)
{
	float determinant = mat4Determinant(input);
	if(determinant < 0.0001f)
		determinant = 1.0f;
	assert(determinant > 0.0001f);
	//assert( determinant > 0.0001f );
	determinant = 1.0f / determinant;
	for(int i = 0;i < 4;i++){
		for(int j = 0;j < 4;j++)
		{
			int sign = 1 - ((i + j) % 2) * 2;
			MAT44 subMat;
			mat4SubMat(input, i, j, subMat);
			float subDeterminant = mat4Determinant(subMat);
			invert.e[i][j] = (subDeterminant * sign) * determinant;
		}
	}
}

inline svec3f mat4Transform(const MAT44& input, const svec3f& v)
{
	svec3f row0 = svec3f(input.e[0][0], input.e[1][0], input.e[2][0]);
	svec3f row1 = svec3f(input.e[0][1], input.e[1][1], input.e[2][1]);
	svec3f row2 = svec3f(input.e[0][2], input.e[1][2], input.e[2][2]);
	svec3f col3 = svec3f(input.e[3][0], input.e[3][1], input.e[3][2]);

	svec3f t = svec3f(vdot3f(row0, v), vdot3f(row1, v), vdot3f(row2, v));
	t = vadd3f(t, col3);
	return t;
}

}
}

#endif /* PS_MATRIX4_H_ */
