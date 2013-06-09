#ifndef PS_MATRIX_H
#define PS_MATRIX_H

#include <string.h>
#include <iostream>
#include "PS_Vector.h"

namespace PS{
namespace MATH{

//Column-major order to match OpenGL.
template <typename T>
class Matrix
{
protected:
    static const int m_ctElements = 16;
    static const int m_ctRowElements = 4;
    static const int m_ctColElements = 4;

public:
    Matrix() { identity();}
    Matrix(const Matrix& rhs){
        this->copyFrom(rhs);
    }

    /*!
     * \brief Create a column major matrix based on the values passed in.
     * \param arrVals defines elements of matrix in row major format
     */
    explicit Matrix(T* arrVals){
        for(int row=0; row < m_ctRowElements; row++)
            for(int col=0; col < m_ctColElements; col++)
                e[col][row] = arrVals[row * m_ctRowElements + col];
        //memcpy(this->e, arrVals, m_ctElements * sizeof(T));
    }

    virtual ~Matrix() {}

    //Basic Ops
    int countElements() const {return m_ctElements;}
    void copyFrom(const Matrix& rhs);
    void zero();
    void identity();

    //Access to rows and cols and elements
    Vec4<T> getRow(int iRow) const;
    Vec4<T> getCol(int iCol) const;
    Vec4<T> getDiag() const;
    void setRow(int iRow, const Vec4<T>& row);
    void setCol(int iCol, const Vec4<T>& cold);
    void setDiag(const Vec4<T>& diag);

    Matrix<T> subMtx(int ki, int kj) const;

    T element(int iRow, int iCol) const;
    void setElement(int iRow, int iCol, T v);
    const T* cptr() const {return &e[0][0];}

    //Transpose and Invert
    void transpose();
    Matrix<T> transposed() const;

    //Invert
    Matrix<T> inverted(int* lpIsInvertible = 0) const;
    Matrix<T> invertTransposed() const;
    T determinant() const;
    T trace() const;

    //Transformations
    Vec4<T> map(const Vec4<T>& v) const;
    void scale(const Vec3<T>& s);
//    void rotate(const quat& q);
//    void rotate(float angleDeg, const vec3f& axis);
    void translate(const Vec3<T>& t);



    //Conditionals
    bool isIdentity() const;
    bool isDiagonal() const;
    bool isSymmetric() const;
    bool isAntiSymmetric() const;
    bool isOrthogonal() const;

    //3D Projection Matrices
    void ortho(T left, T right, T bottom, T top, T nearPlane, T farPlane);
    void frustum(T left, T right, T bottom, T top, T nearPlane, T farPlane);
    void perspective(T angle, T aspect, T nearPlane, T farPlane);
    void lookAt(const Vec3<T>& eye, const Vec3<T>& center, const Vec3<T>& up);


    //Static Ops
    static inline bool isEqual(const Matrix& a, const Matrix& b);
    static inline Matrix mul(const Matrix& a, const Matrix& b);
    static inline Matrix mul(const Matrix& a, T s);

    static inline Matrix mulEntrywise(const Matrix& a, const Matrix& b);
    static inline T mulFrobeniousInner(const Matrix& a, const Matrix& b);

    static inline Matrix add(const Matrix& a, const Matrix& b);
    static inline Matrix sub(const Matrix& a, const Matrix& b);

    //Determinant of a 2x2 matrix
    static inline T matrixDet2(const T m[4][4],
                               int col0, int col1,
                               int row0, int row1);

    //Determinant of a 3x3 matrix
    static inline T matrixDet3(const T m[4][4],
                               int col0, int col1, int col2,
                               int row0, int row1, int row2);

    //Determinant of a 4x4 matrix
    static inline T matrixDet4(const T m[4][4]);

    //Operators
    inline Matrix& operator=(const Matrix& rhs);
    inline bool operator==(const Matrix& rhs) const;
    inline Matrix operator*(const Matrix& rhs) const;
    inline Matrix operator*(T s) const;
    inline Matrix operator+(const Matrix& rhs) const;
    inline Matrix operator-(const Matrix& rhs) const;

    friend class Quaternion;
public:
    T e[m_ctColElements][m_ctRowElements];
};

template<typename T>
void Matrix<T>::copyFrom(const Matrix& rhs)
{
    memcpy(this->e, rhs.e, m_ctElements * sizeof(T));
}

template <typename T>
void Matrix<T>::zero()
{
    memset(e, 0, m_ctElements * sizeof(T));
}

template <typename T>
void Matrix<T>::identity()
{
    zero();
    for(int i=0; i < m_ctColElements; i++)
        this->e[i][i] = 1.0;
}

template <typename T>
Vec4<T> Matrix<T>::getRow(int iRow) const
{
    Vec4<T> row;
    for(int iCol=0; iCol < m_ctColElements; iCol++)
        row.setElement(iCol, this->e[iCol][iRow]);
    return row;
}

template <typename T>
Vec4<T> Matrix<T>::getCol(int iCol) const
{
    Vec4<T> col;
    for(int iRow=0; iRow < m_ctRowElements; iRow++)
        col.setElement(iRow, this->e[iCol][iRow]);
    return col;
}

template <typename T>
Vec4<T> Matrix<T>::getDiag() const
{
    Vec4<T> diag;
    for(int i=0; i<m_ctRowElements; i++)
        diag.setElement(i, this->e[i][i]);
    return diag;
}

template <typename T>
void Matrix<T>::setRow(int iRow, const Vec4<T>& row)
{
    for(int i=0; i<m_ctColElements; i++)
        this->e[i][iRow] = row.element(i);
}

template <typename T>
void Matrix<T>::setCol(int iCol, const Vec4<T>& col)
{
    for(int i=0; i<m_ctRowElements; i++)
        this->e[iCol][i] = col.element(i);
}

template <typename T>
void Matrix<T>::setDiag(const Vec4<T>& diag)
{
    for(int i=0; i<m_ctColElements; i++)
        this->e[i][i] = diag.element(i);
}


template <typename T>
Matrix<T> Matrix<T>::subMtx(int ki, int kj) const
{
    Matrix<T> output;
    int row;
    int dstCol = 0, dstRow = 0;
    for(int col = 0;col < m_ctColElements;col++)
    {
        if(col == kj){
            continue;
        }
        for(dstRow = 0, row = 0;row < m_ctRowElements; row++)
        {
            if(row == ki){
                continue;
            }
            output.e[dstCol][dstRow] = this->e[col][row];
            dstRow++;
        }
        dstCol++;
    }

    return output;
}

template <typename T>
T Matrix<T>::element(int iRow, int iCol) const
{
    return this->e[iCol][iRow];
}

template <typename T>
void Matrix<T>::setElement(int iRow, int iCol, T v)
{
    this->e[iCol][iRow] = v;
}

template <typename T>
void Matrix<T>::transpose()
{
    Matrix<T> temp;
    //Row
    for(int i=0; i < m_ctRowElements; i++)
        for(int j=0; j<m_ctColElements; j++)
            temp.e[i][j] = this->e[j][i];
    this->copyFrom(temp);
}

template <typename T>
Matrix<T> Matrix<T>::transposed() const
{
    Matrix<T> output = *this;
    output.transpose();
    return output;
}

template <typename T>
Matrix<T> Matrix<T>::inverted(int* lpIsInvertible) const
{
    Matrix<T> inv;
    T det = this->determinant();
    if (det == 0.0f)
    {
        if (lpIsInvertible)
            *lpIsInvertible = false;
        return inv;
    }
    det = 1.0f / det;

    inv.e[0][0] =  matrixDet3(e, 1, 2, 3, 1, 2, 3) * det;
    inv.e[0][1] = -matrixDet3(e, 0, 2, 3, 1, 2, 3) * det;
    inv.e[0][2] =  matrixDet3(e, 0, 1, 3, 1, 2, 3) * det;
    inv.e[0][3] = -matrixDet3(e, 0, 1, 2, 1, 2, 3) * det;
    inv.e[1][0] = -matrixDet3(e, 1, 2, 3, 0, 2, 3) * det;
    inv.e[1][1] =  matrixDet3(e, 0, 2, 3, 0, 2, 3) * det;
    inv.e[1][2] = -matrixDet3(e, 0, 1, 3, 0, 2, 3) * det;
    inv.e[1][3] =  matrixDet3(e, 0, 1, 2, 0, 2, 3) * det;
    inv.e[2][0] =  matrixDet3(e, 1, 2, 3, 0, 1, 3) * det;
    inv.e[2][1] = -matrixDet3(e, 0, 2, 3, 0, 1, 3) * det;
    inv.e[2][2] =  matrixDet3(e, 0, 1, 3, 0, 1, 3) * det;
    inv.e[2][3] = -matrixDet3(e, 0, 1, 2, 0, 1, 3) * det;
    inv.e[3][0] = -matrixDet3(e, 1, 2, 3, 0, 1, 2) * det;
    inv.e[3][1] =  matrixDet3(e, 0, 2, 3, 0, 1, 2) * det;
    inv.e[3][2] = -matrixDet3(e, 0, 1, 3, 0, 1, 2) * det;
    inv.e[3][3] =  matrixDet3(e, 0, 1, 2, 0, 1, 2) * det;
    //inv.flagBits = flagBits;

    if (lpIsInvertible)
        *lpIsInvertible = true;
    return inv;
}


template <typename T>
Matrix<T> Matrix<T>::invertTransposed() const
{
    Matrix<T> inv = this->inverted();
    return inv.transposed();
}

template <typename T>
inline T Matrix<T>::matrixDet2(const T m[4][4],
                    int col0, int col1,
                    int row0, int row1)
{
    return m[col0][row0] * m[col1][row1] - m[col0][row1] * m[col1][row0];
}


// Calculate the determinant of a 3x3 sub-matrix.
//     | A B C |
// M = | D E F |   det(M) = A * (EI - HF) - B * (DI - GF) + C * (DH - GE)
//     | G H I |
template <typename T>
inline T Matrix<T>::matrixDet3(const T m[4][4],
                               int col0, int col1, int col2,
                               int row0, int row1, int row2)
{
    return m[col0][row0] * matrixDet2(m, col1, col2, row1, row2)
            - m[col1][row0] * matrixDet2(m, col0, col2, row1, row2)
            + m[col2][row0] * matrixDet2(m, col0, col1, row1, row2);
}

// The 4x4 matrix inverse algorithm is based on that described at:
// http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q24
// Some optimization has been done to avoid making copies of 3x3
// sub-matrices and to unroll the loops.
template <typename T>
inline T Matrix<T>::matrixDet4(const T m[4][4])
{
    T det;
    det  = m[0][0] * matrixDet3(m, 1, 2, 3, 1, 2, 3);
    det -= m[1][0] * matrixDet3(m, 0, 2, 3, 1, 2, 3);
    det += m[2][0] * matrixDet3(m, 0, 1, 3, 1, 2, 3);
    det -= m[3][0] * matrixDet3(m, 0, 1, 2, 1, 2, 3);
    return det;
}

template <typename T>
T Matrix<T>::determinant() const
{
    return matrixDet4(e);
}

template <typename T>
T Matrix<T>::trace() const
{
    T sum = 0.0;
    for(int i=0; i<m_ctColElements; i++)
        sum += this->e[i][i];
    return sum;
}

//Map a vector using this matrix
template <typename T>
Vec4<T> Matrix<T>::map(const Vec4<T>& v) const
{
    Vec4<T> output;
    for(int i=0; i< m_ctColElements; i++)
        output.setElement(i, Vec4<T>::dot(this->getRow(i), v));
    return output;
}

template <typename T>
void Matrix<T>::scale(const Vec3<T>& s)
{
	Matrix<T> work;
	work.e[0][0] = s.x;
	work.e[1][1] = s.y;
	work.e[2][2] = s.z;

	*this = *this * work;
}
/*
template <typename T>
void Matrix<T>::rotate(const quat& q)
{
	Matrix<T> work;
	q.toMatrix(work);
	*this = *this * work;
}

template <typename T>
void Matrix<T>::rotate(float angleDeg, const vec3f& axis)
{
	Matrix<T> work;
	quat q;
	q.fromAngleAxis(angleDeg, axis);
	q.toMatrix(work);
	*this = *this * work;
}
*/

template <typename T>
void Matrix<T>::translate(const Vec3<T>& t)
{
	Matrix<T> work;
	work.e[3][0] = t.x;
	work.e[3][1] = t.y;
	work.e[3][2] = t.z;


	*this = mul(*this, work);
}

//Conditionals
template <typename T>
bool Matrix<T>::isIdentity() const
{
    Matrix<T> ident;
    return isEqual(ident, *this);
}

template <typename T>
bool Matrix<T>::isDiagonal() const
{
    for(int i=0; i<m_ctColElements; i++)
        for(int j=0; j<m_ctRowElements; j++)
        {
            if(i!=j && e[i][j] != 0.0f)
                return false;
        }
    return true;
}

template <typename T>
bool Matrix<T>::isSymmetric() const
{
    return isEqual(*this, this->transposed());
}

template <typename T>
bool Matrix<T>::isAntiSymmetric() const
{
    //a skew-symmetric (or antisymmetric or antimetric[1]) matrix is a square matrix A whose
    //transpose is also its negative; that is, it satisfies the equation A = −AT.
    return isEqual(*this, mul(this->transposed(), -1.0));
}

template <typename T>
bool Matrix<T>::isOrthogonal() const
{
    //an orthogonal matrix is a square matrix with real entries whose columns and rows are orthogonal unit vectors (i.e., orthonormal vectors).
    //Equivalently, a matrix Q is orthogonal if its transpose is equal to its inverse.
    return isEqual(this->transposed(), this->inverted());
}

//Friend Functions
template <typename T>
bool Matrix<T>::isEqual(const Matrix<T>& a, const Matrix<T>& b)
{
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
        {
            if(a.e[i][j] != b.e[i][j])
                return false;
        }

    return true;
}


template <typename T>
Matrix<T> Matrix<T>::mul(const Matrix<T>& a, const Matrix<T>& b)
{
    Matrix<T> output;
    output.zero();
    for(int iCol=0; iCol < 4; iCol++ )
        for(int iRow=0; iRow < 4; iRow++ )
            for(int k=0; k < 4; k++ )
                output.e[iCol][iRow] += a.e[k][iRow] * b.e[iCol][k];
    return output;
}

template <typename T>
Matrix<T> Matrix<T>::mul(const Matrix<T>& a, T s)
{
    Matrix<T> output;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
        {
            output[i][j] = s * a.e[i][j];
        }
    return output;
}

template <typename T>
Matrix<T> Matrix<T>::mulEntrywise(const Matrix<T>& a, const Matrix<T>& b)
{
    Matrix<T> output;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
        {
            output[i][j] = a.e[i][j] * b.e[i][j];
        }
    return output;
}

template <typename T>
T Matrix<T>::mulFrobeniousInner(const Matrix<T>& a, const Matrix<T>& b)
{
    //tr(aTb) = tr(abT)
    return mul(a.transposed(), b).trace();
}

template <typename T>
Matrix<T> Matrix<T>::add(const Matrix<T>& a, const Matrix<T>& b)
{
    Matrix<T> output;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
        {
            output[i][j] = a.e[i][j] + b.e[i][j];
        }
    return output;

}

template <typename T>
Matrix<T> Matrix<T>::sub(const Matrix<T>& a, const Matrix<T>& b)
{
    Matrix<T> output;
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
        {
            output[i][j] = a.e[i][j] - b.e[i][j];
        }
    return output;
}

/*!
    Multiplies this matrix by another that applies an orthographic
    projection for a window with lower-left corner (\a left, \a bottom),
    upper-right corner (\a right, \a top), and the specified \a nearPlane
    and \a farPlane clipping planes.

    \sa frustum(), perspective()
*/
template <typename T>
void Matrix<T>::ortho(T left, T right, T bottom, T top, T nearPlane, T farPlane)
{
    // Bail out if the projection volume is zero-sized.
    if (left == right || bottom == top || nearPlane == farPlane)
        return;

    // Construct the projection.
    T width = right - left;
    T invheight = top - bottom;
    T clip = farPlane - nearPlane;
    Matrix<T> m;
    m.e[0][0] = 2.0f / width;
    m.e[1][0] = 0.0f;
    m.e[2][0] = 0.0f;
    m.e[3][0] = -(left + right) / width;
    m.e[0][1] = 0.0f;
    m.e[1][1] = 2.0f / invheight;
    m.e[2][1] = 0.0f;
    m.e[3][1] = -(top + bottom) / invheight;
    m.e[0][2] = 0.0f;
    m.e[1][2] = 0.0f;
    m.e[2][2] = -2.0f / clip;
    m.e[3][2] = -(nearPlane + farPlane) / clip;
    m.e[0][3] = 0.0f;
    m.e[1][3] = 0.0f;
    m.e[2][3] = 0.0f;
    m.e[3][3] = 1.0f;
    //m.flagBits = Translation | Scale;

    // Apply the projection.
    *this = m;
}

/*!
    Multiplies this matrix by another that applies a perspective
    frustum projection for a window with lower-left corner (\a left, \a bottom),
    upper-right corner (\a right, \a top), and the specified \a nearPlane
    and \a farPlane clipping planes.

    \sa ortho(), perspective()
*/
template <typename T>
void Matrix<T>::frustum(T left, T right, T bottom, T top, T nearPlane, T farPlane)

{
    // Bail out if the projection volume is zero-sized.
    if (left == right || bottom == top || nearPlane == farPlane)
        return;

    // Construct the projection.
    Matrix<T> m;
    T width = right - left;
    T invheight = top - bottom;
    T clip = farPlane - nearPlane;
    m.e[0][0] = 2.0f * nearPlane / width;
    m.e[1][0] = 0.0f;
    m.e[2][0] = (left + right) / width;
    m.e[3][0] = 0.0f;
    m.e[0][1] = 0.0f;
    m.e[1][1] = 2.0f * nearPlane / invheight;
    m.e[2][1] = (top + bottom) / invheight;
    m.e[3][1] = 0.0f;
    m.e[0][2] = 0.0f;
    m.e[1][2] = 0.0f;
    m.e[2][2] = -(nearPlane + farPlane) / clip;
    m.e[3][2] = -2.0f * nearPlane * farPlane / clip;
    m.e[0][3] = 0.0f;
    m.e[1][3] = 0.0f;
    m.e[2][3] = -1.0f;
    m.e[3][3] = 0.0f;
    //m.flagBits = General;

    // Apply the projection.
    *this = m;
}

/*!
    Multiplies this matrix by another that applies a perspective
    projection.  The field of view will be \a angle degrees within
    a window with a given \a aspect ratio.  The projection will
    have the specified \a nearPlane and \a farPlane clipping planes.

    \sa ortho(), frustum()
*/
template <typename T>
void Matrix<T>::perspective(T angle, T aspect, T nearPlane, T farPlane)
{
    // Bail out if the projection volume is zero-sized.
    if (nearPlane == farPlane || aspect == 0.0f)
        return;

    // Construct the projection.
    Matrix<T> m;
    T radians = (angle / 2.0f) * Pi / 180.0f;
    T sine = sin(radians);
    if (sine == 0.0f)
        return;

    T cotan = cos(radians) / sine;
    T clip = farPlane - nearPlane;
    m.e[0][0] = cotan / aspect;
    m.e[1][0] = 0.0f;
    m.e[2][0] = 0.0f;
    m.e[3][0] = 0.0f;
    m.e[0][1] = 0.0f;
    m.e[1][1] = cotan;
    m.e[2][1] = 0.0f;
    m.e[3][1] = 0.0f;
    m.e[0][2] = 0.0f;
    m.e[1][2] = 0.0f;
    m.e[2][2] = -(nearPlane + farPlane) / clip;
    m.e[3][2] = -(2.0f * nearPlane * farPlane) / clip;
    m.e[0][3] = 0.0f;
    m.e[1][3] = 0.0f;
    m.e[2][3] = -1.0f;
    m.e[3][3] = 0.0f;
    //m.flagBits = General;

    // Apply the projection.
    *this = m;
}

/*!
    Multiplies this matrix by another that applies an \a eye position
    transformation.  The \a center value indicates the center of the
    view that the \a eye is looking at.  The \a up value indicates
    which direction should be considered up with respect to the \a eye.
*/
template <typename T>
void Matrix<T>::lookAt(const Vec3<T>& eye, const Vec3<T>& center, const Vec3<T>& up)
{
    Vec3<T> forward = (center - eye).normalized();
    Vec3<T> side = Vec3<T>::cross(forward, up).normalized();
    Vec3<T> upVector = Vec3<T>::cross(side, forward);

    Matrix<T> m;
    m.e[0][0] = side.x();
    m.e[1][0] = side.y();
    m.e[2][0] = side.z();
    m.e[3][0] = 0.0f;
    m.e[0][1] = upVector.x();
    m.e[1][1] = upVector.y();
    m.e[2][1] = upVector.z();
    m.e[3][1] = 0.0f;
    m.e[0][2] = -forward.x();
    m.e[1][2] = -forward.y();
    m.e[2][2] = -forward.z();
    m.e[3][2] = 0.0f;
    m.e[0][3] = 0.0f;
    m.e[1][3] = 0.0f;
    m.e[2][3] = 0.0f;
    m.e[3][3] = 1.0f;
    //m.flagBits = Rotation;

    *this = m;
    translate(-eye);
}



//Operators
template <typename T>
Matrix<T>& Matrix<T>::operator=(const Matrix& rhs)
{
    this->copyFrom(rhs);
    return (*this);
}

template <typename T>
bool Matrix<T>::operator==(const Matrix& rhs) const
{    
    return isEqual(*this, rhs);
}

template <typename T>
Matrix<T> Matrix<T>::operator*(const Matrix& rhs) const
{
    return mul(*this, rhs);
}

template <typename T>
Matrix<T> Matrix<T>::operator*(T s) const
{
    return mul(*this, s);
}

template <typename T>
Matrix<T> Matrix<T>::operator+(const Matrix& rhs) const
{
    return add(*this, rhs);
}

template <typename T>
Matrix<T> Matrix<T>::operator-(const Matrix& rhs) const
{
    return sub(*this, rhs);
}


typedef Matrix<float> mat44;
typedef Matrix<float> mat44f;
typedef Matrix<double> mat44d;

//Debugging Functions
void MtxPrint(const mat44& a);
void MtxPrint(const mat44d& a);
}
}
#endif // PS_MATRIX_H
