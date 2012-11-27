/*
 * PS_VolumeConservingIntegrator.cpp
 *
 *  Created on: Nov 13, 2012
 *      Author: pourya
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../AA_VegaFem/include/matrixIO.h"
#include "../AA_VegaFem/include/performanceCounter.h"
#include "../AA_VegaFem/include/insertRows.h"
#include "../AA_VegaFem/integrator/implicitNewmarkSparse.h"
#include "PS_VolumeConservingIntegrator.h"

VolumeConservingIntegrator::VolumeConservingIntegrator(int r, double timestep,
		SparseMatrix * massMatrix_, ForceModel * forceModel_,
		int positiveDefiniteSolver_, int numConstrainedDOFs_,
		int * constrainedDOFs_, double dampingMassCoef,
		double dampingStiffnessCoef, int maxIterations, double epsilon,
		int numSolverThreads_):	ImplicitNewmarkSparse(r, timestep, massMatrix_, forceModel_,
				positiveDefiniteSolver_, numConstrainedDOFs_, constrainedDOFs_,
				dampingMassCoef, dampingStiffnessCoef, maxIterations, epsilon,
				0.25, 0.5, numSolverThreads_)
{

}



VolumeConservingIntegrator::~VolumeConservingIntegrator() {
}

// sets the state based on given q, qvel
// automatically computes acceleration assuming zero external force
int VolumeConservingIntegrator::SetState(double * q_, double * qvel_) {
	memcpy(q, q_, sizeof(double) * r);

	if (qvel_ != NULL)
		memcpy(qvel, qvel_, sizeof(double) * r);

	return 0;
}

int VolumeConservingIntegrator::DoTimestep() {
	int numIter = 0;

	//Error after the first step
	double error0 = 0;
	double errorQuotient;

	// store current amplitudes and set initial guesses for qaccel, qvel
	for (int i = 0; i < r; i++) {
		qaccel_1[i] = qaccel[i] = 0;
		q_1[i] = q[i];
		qvel_1[i] = qvel[i];
	}

	do {
		int i;

		/*
		 printf("q:\n");
		 for(int i=0; i<r; i++)
		 printf("%G ", q[i]);
		 printf("\n");

		 printf("Internal forces:\n");
		 for(int i=0; i<r; i++)
		 printf("%G ", internalForces[i]);
		 printf("\n");
		 */

		PerformanceCounter counterForceAssemblyTime;
		forceModel->GetForceAndMatrix(q, internalForces, tangentStiffnessMatrix);
		counterForceAssemblyTime.StopCounter();
		forceAssemblyTime = counterForceAssemblyTime.GetElapsedTime();

		//tangentStiffnessMatrix->Print();
		//tangentStiffnessMatrix->Save("K");

		//Scale internal forces
		for (i = 0; i < r; i++)
			internalForces[i] *= internalForceScalingFactor;

		*tangentStiffnessMatrix *= internalForceScalingFactor;

		memset(qresidual, 0, sizeof(double) * r);

		if (useStaticSolver) {
			// fint + K * qdelta = fext

			// add externalForces, internalForces
			for (i = 0; i < r; i++) {
				qresidual[i] = externalForces[i] - internalForces[i];
				qdelta[i] = qresidual[i];
			}
		} else {
			tangentStiffnessMatrix->ScalarMultiply(dampingStiffnessCoef,
					rayleighDampingMatrix);
			rayleighDampingMatrix->AddSubMatrix(dampingMassCoef, *massMatrix);

			// build effective stiffness:
			// Keff = M + h D + h^2 * K
			// compute force residual, store it into aux variable qresidual
			// qresidual = h * (-D qdot - fint + fext - h * K * qdot))

			//add mass matrix and damping matrix to tangentStiffnessMatrix
			*tangentStiffnessMatrix *= timestep;

			*tangentStiffnessMatrix += *rayleighDampingMatrix;
			tangentStiffnessMatrix->AddSubMatrix(1.0, *dampingMatrix, 1); // at this point, tangentStiffnessMatrix = h * K + D
			tangentStiffnessMatrix->MultiplyVector(qvel, qresidual);
			*tangentStiffnessMatrix *= timestep;
			tangentStiffnessMatrix->AddSubMatrix(1.0, *massMatrix);

			// add externalForces, internalForces
			for (i = 0; i < r; i++) {
				qresidual[i] += internalForces[i] - externalForces[i];
				qresidual[i] *= -timestep;
				qdelta[i] = qresidual[i];
			}
		}

		/*
		 printf("internal forces:\n");
		 for(int i=0; i<r; i++)
		 printf("%G ", internalForces[i]);
		 printf("\n");

		 printf("external forces:\n");
		 for(int i=0; i<r; i++)
		 printf("%G ", externalForces[i]);
		 printf("\n");

		 printf("residual:\n");
		 for(int i=0; i<r; i++)
		 printf("%G ", -qresidual[i]);
		 printf("\n");
		 */

		double error = 0;
		for (i = 0; i < r; i++)
			error += qresidual[i] * qresidual[i];

		// on the first iteration, compute initial error
		if (numIter == 0) {
			error0 = error;
			errorQuotient = 1.0;
		} else {
			// rel error wrt to initial error before performing this iteration
			errorQuotient = error / error0;
		}

		if (errorQuotient < epsilon * epsilon)
			break;

		//tangentStiffnessMatrix->Save("Keff");
		RemoveRows(r, bufferConstrained, qdelta, numConstrainedDOFs,
				constrainedDOFs);
		systemMatrix->AssignSuperMatrix(tangentStiffnessMatrix);

		// solve: systemMatrix * qdelta = qresidual

		PerformanceCounter counterSystemSolveTime;
		memset(buffer, 0, sizeof(double) * r);

#ifdef SPOOLES
		int info;
		if (numSolverThreads > 1)
		{
			SPOOLESSolverMT * solver = new SPOOLESSolverMT(systemMatrix, numSolverThreads);
			info = solver->SolveLinearSystem(buffer, bufferConstrained);
			delete(solver);
		}
		else
		{
			SPOOLESSolver * solver = new SPOOLESSolver(systemMatrix);
			info = solver->SolveLinearSystem(buffer, bufferConstrained);
			delete(solver);
		}
		char solverString[16] = "SPOOLES";
#endif

#ifdef PARDISO
		int info = pardisoSolver->ComputeCholeskyDecomposition(systemMatrix);
		if (info == 0)
		info = pardisoSolver->SolveLinearSystem(buffer, bufferConstrained);
		char solverString[16] = "PARDISO";
#endif

#ifdef PCG
		int info =
				jacobiPreconditionedCGSolver->SolveLinearSystemWithJacobiPreconditioner(
						buffer, bufferConstrained, 1e-6, 10000);
		if (info > 0)
			info = 0;
		char solverString[16] = "PCG";
#endif

		if (info != 0) {
			printf(
					"Error: %s sparse solver returned non-zero exit status %d.\n",
					solverString, (int) info);
			exit(-1);
			return 1;
		}

		counterSystemSolveTime.StopCounter();
		systemSolveTime = counterSystemSolveTime.GetElapsedTime();

		InsertRows(r, buffer, qdelta, numConstrainedDOFs, constrainedDOFs);

		/*
		 printf("qdelta:\n");
		 for(int i=0; i<r; i++)
		 printf("%G ", qdelta[i]);
		 printf("\n");
		 exit(1);
		 */
		// update state
		if (useStaticSolver) {
			for (i = 0; i < r; i++) {
				q[i] += qdelta[i];
				qvel[i] = (q[i] - q_1[i]) / timestep;
			}
		} else {
			for (i = 0; i < r; i++) {
				qvel[i] += qdelta[i];
				q[i] += timestep * qvel[i];
			}
		}

		for (int i = 0; i < numConstrainedDOFs; i++)
			q[constrainedDOFs[i]] = qvel[constrainedDOFs[i]] =
					qaccel[constrainedDOFs[i]] = 0.0;

		numIter++;
	} while (numIter < maxIterations);

	/*
	 printf("q:\n");
	 for(int i=0; i<r; i++)
	 printf("%G ", q[i]);
	 printf("\n");

	 printf("qvel:\n");
	 for(int i=0; i<r; i++)
	 printf("%G ", qvel[i]);
	 printf("\n");
	 */

	//printf("Num iterations performed: %d\n",numIter);
	//if ((numIter >= maxIterations) && (maxIterations > 1))
	//{
	//printf("Warning: method did not converge in max number of iterations.\n");
	//}
	return 0;
}

