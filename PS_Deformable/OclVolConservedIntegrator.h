/*
 * OclVolConservedIntegrator.h
 *
 *  Created on: May 2, 2013
 *      Author: pourya
 */

#ifndef OCLVOLCONSERVEDINTEGRATOR_H_
#define OCLVOLCONSERVEDINTEGRATOR_H_

#include "../AA_VegaFem/integrator/implicitNewmarkSparse.h"
//#define VIENNACL_WITH_OPENCL
//
//#include "viennacl/scalar.hpp"
//#include "viennacl/vector.hpp"
//#include "viennacl/coordinate_matrix.hpp"
//#include "viennacl/compressed_matrix.hpp"
//#include "viennacl/ell_matrix.hpp"
//#include "viennacl/hyb_matrix.hpp"
//
//#include "viennacl/linalg/cg.hpp"
//#include "viennacl/linalg/bicgstab.hpp"
//#include "viennacl/linalg/gmres.hpp"
//
//#include "viennacl/linalg/ilu.hpp"
//#include "viennacl/linalg/ichol.hpp"
//#include "viennacl/linalg/jacobi_precond.hpp"
//#include "viennacl/linalg/row_scaling.hpp"
//
//#ifdef VIENNACL_WITH_OPENCL
//  #include "viennacl/linalg/mixed_precision_cg.hpp"
//#endif
//
//#include "viennacl/io/matrix_market.hpp"

typedef double ScalarType;

class OclVolConservedIntegrator: public ImplicitNewmarkSparse {
public:
	//OclVolConservedIntegrator() {}

	// constrainedDOFs is an integer array of degrees of freedom that are to be fixed to zero (e.g., to permanently fix a vertex in a deformable simulation)
	// constrainedDOFs are 0-indexed (separate DOFs for x,y,z), and must be pre-sorted (ascending)
	// numThreads applies only to the PARDISO and SPOOLES solvers; if numThreads > 0, the sparse linear solves are multi-threaded; default: 0 (use single-threading)

	OclVolConservedIntegrator(int r, double timestep,
			SparseMatrix * massMatrix, ForceModel * forceModel,
			int positiveDefiniteSolver = 0, int numConstrainedDOFs = 0,
			int * constrainedDOFs = NULL, double dampingMassCoef = 0.0,
			double dampingStiffnessCoef = 0.0, int maxIterations = 1,
			double epsilon = 1E-6, int numSolverThreads = 0);


	virtual ~OclVolConservedIntegrator();

	// sets q, and (optionally) qvel
	// returns 0
	virtual int SetState(double * q, double * qvel = NULL);
	virtual int DoTimestep();

protected:
//	viennacl::vector<ScalarType> vcl_q;
//	viennacl::vector<ScalarType> vcl_qvel;
//	viennacl::vector<ScalarType> vcl_qacc;

};

#endif /* OCLVOLCONSERVEDINTEGRATOR_H_ */
