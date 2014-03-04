/*
 * PS_VolumeConservingIntegrator.h
 *
 *  Created on: Nov 13, 2012
 *      Author: pourya
 */

#ifndef PS_VOLUMECONSERVINGINTEGRATOR_H_
#define PS_VOLUMECONSERVINGINTEGRATOR_H_

#include "vegafem/integrator/implicitNewmarkSparse.h"

class VolumeConservingIntegrator: public ImplicitNewmarkSparse {
public:
	//VolumeConservingIntegrator() {}

	// constrainedDOFs is an integer array of degrees of freedom that are to be fixed to zero (e.g., to permanently fix a vertex in a deformable simulation)
	// constrainedDOFs are 0-indexed (separate DOFs for x,y,z), and must be pre-sorted (ascending)
	// numThreads applies only to the PARDISO and SPOOLES solvers; if numThreads > 0, the sparse linear solves are multi-threaded; default: 0 (use single-threading)

	VolumeConservingIntegrator(int r, double timestep,
			SparseMatrix * massMatrix, ForceModel * forceModel,
			int positiveDefiniteSolver = 0, int numConstrainedDOFs = 0,
			int * constrainedDOFs = NULL, double dampingMassCoef = 0.0,
			double dampingStiffnessCoef = 0.0, int maxIterations = 1,
			double epsilon = 1E-6, int numSolverThreads = 0);


	virtual ~VolumeConservingIntegrator();

	// sets q, and (optionally) qvel
	// returns 0
	virtual int SetState(double * q, double * qvel = NULL);
	virtual int DoTimestep();

protected:
};

#endif /* PS_VOLUMECONSERVINGINTEGRATOR_H_ */
