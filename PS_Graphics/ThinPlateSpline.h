/*
 * ThinPlateSpline.h
 *
 *  Created on: Mar 29, 2013
 *      Author: pourya
 */

#ifndef THINPLATESPLINE_H_
#define THINPLATESPLINE_H_

namespace PS {
namespace MATH {

/**
 *  @class  ThinPlateSpline2D
 *  @brief  class representing a thin plate spline.
 *  It is largely based on WildMagics (www.geometrictools.com) code, but
 *  uses the faster and header only math library Eigen due to 64 bit issues
 */

template<class T>
class ThinPlateSpline2D {
public:

	/**
	 *  constructor
	 *  @param  quantity the number of points in form (x,y,f(x,y))
	 *  @param  X the x coordinates
	 *  @param  Y the y coordinates
	 *  @param  F the f(x,y) values
	 *  @param  smooth the smoothing factor, has to be non-negative
	 */
	ThinPlateSpline2D(T3Int quantity,
			std::vector<tuft3::math::Vector2<T> > & points, std::vector<T> & F,
			T smooth) :
			mInitialized(false), mQuantity(quantity), mPoints(points), mA(
					mQuantity) //, mMin(), mMax(), mInvRange()
	{
		T3Int i, row, col;

		// Compute matrix A = E+smooth*I [NxN matrix].

		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> AMat(mQuantity,
				mQuantity);
		for (row = 0; row < mQuantity; ++row) {
			for (col = 0; col < mQuantity; ++col) {
				if (row == col) {
					AMat(row, col) = smooth;
				} else {
					tuft3::math::Vector2<T> delta = mPoints[row] - mPoints[col];
					T t = delta.getLength();
					AMat(row, col) = Kernel(t);
				}
			}
		}

		// Compute matrix B [Nx3 matrix].
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> BMat(mQuantity, 3);
		for (row = 0; row < mQuantity; ++row) {
			BMat(row, 0) = (T) 1;
			BMat(row, 1) = mPoints[row].x;
			BMat(row, 2) = mPoints[row].y;
		}
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> BMatTrans =
				BMat.transpose();

		// Compute A^{-1}.
		Eigen::FullPivLU < Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
				> piv(AMat);
		if (!piv.isInvertible()) {
			return;
		}
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> invAMat =
				AMat.inverse();

		// Compute P = B^t A^{-1}  [3xN matrix].
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PMat = BMatTrans
				* invAMat;

		// Compute Q = P B = B^t A^{-1} B  [3x3 matrix].
		Eigen::Matrix<T, 3, 3> QMat = PMat * BMat;

		// Compute Q^{-1}.
		Eigen::Matrix<T, 3, 3> invQMat;
		bool invertible;
		T determinant;
		QMat.computeInverseAndDetWithCheck(invQMat, determinant, invertible);
		if (!invertible) {
			return;
		}

		// Compute P*z.
		T prod[3];
		for (row = 0; row < 3; ++row) {
			prod[row] = (T) 0;
			for (i = 0; i < mQuantity; ++i) {
				prod[row] += PMat(row, i) * F[i];
			}
		}

		// Compute 'b' vector for smooth thin plate spline.
		for (row = 0; row < 3; ++row) {
			mB[row] = (T) 0;
			for (i = 0; i < 3; ++i) {
				mB[row] += invQMat(row, i) * prod[i];
			}
		}

		// Compute z-B*b.
		std::vector<T> tmp(mQuantity);
		for (row = 0; row < mQuantity; ++row) {
			tmp[row] = F[row];
			for (i = 0; i < 3; ++i) {
				tmp[row] -= BMat(row, i) * mB[i];
			}
		}

		// Compute 'a' vector for smooth thin plate spline.
		for (row = 0; row < mQuantity; ++row) {
			mA[row] = (T) 0;
			for (i = 0; i < mQuantity; ++i) {
				mA[row] += invAMat(row, i) * tmp[i];
			}
		}

		mInitialized = true;
	}

	/**
	 *  default destructor
	 */
	~ThinPlateSpline2D() {
		mPoints.clear();
		mA.clear();
	}

	/*
	 *  Check this after the constructor call to see if the thin plate spline
	 *  coefficients were successfully computed.  If so, then calls to
	 *  operator()(Real,Real) will work properly.
	 *  @return returns if initialisation worked
	 */
	bool IsInitialized() const {
		return mInitialized;
	}

	/**
	 *  Evaluate the thin plate spline interpolator.  If IsInitialized()
	 *  returns 'false', this operator will always return std::numeric_limits<T>::max().
	 *  @param  point the point
	 *  @return returns the interpolated value
	 */
	T operator()(tuft3::math::Vector2<T> point) {
		if (mInitialized) {
			// Map (x,y) to the unit square.
//                    x = (x - mXMin)*mXInvRange;
//                    y = (y - mYMin)*mYInvRange;

			T result = mB[0] + mB[1] * point.x + mB[2] * point.y;
			for (int i = 0; i < mQuantity; ++i) {

				tuft3::math::Vector2<T> delta = point - mPoints[i];
				T t = delta.getLength();
				result += mA[i] * Kernel(t);
			}
			return result;
		}

		return std::numeric_limits<T>::max();
	}

private:

	/**
	 *  static method for a filter kernel
	 *  @param  t the value to get the kernel value for
	 *  @return returns the corresponding kernel value
	 */
	static T Kernel(T t) {
		if (t > (T) 0) {
			T t2 = t * t;
			return t2 * log(t2);
		}
		return (T) 0;
	}

	bool mInitialized; /**< if the values are initialised. */
	T3Int mQuantity; /**< the number of values. */

	// Input data mapped to unit cube.
	std::vector<tuft3::math::Vector2<T> > mPoints;

	// Thin plate spline coefficients.
	std::vector<T> mA;
	T mB[3];
};

}
}

#endif /* THINPLATESPLINE_H_ */

