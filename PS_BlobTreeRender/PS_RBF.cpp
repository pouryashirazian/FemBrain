/*
 * PS_RBF.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: pourya
 */
#include "PS_RBF.h"
#include "PS_Graphics/PS_DebugUtils.h"
#undef Success

//Include from 3rdparty source code directory instead of installation
#include  "3rdparty/eigen/Eigen/Dense"


using Eigen::MatrixXd;
using namespace PS::DEBUG;

namespace PS {
namespace HPC {

FastRBF::FastRBF() {
	m_ctCenters = 0;
	m_lpGPUPoly = NULL;
	m_offSurfaceLen = DEFAULT_OFFSURFACE_LEN;
}

FastRBF::FastRBF(GPUPoly* lpGPUPoly) {
	m_ctCenters = 0;
	m_lpGPUPoly = lpGPUPoly;
	m_offSurfaceLen = DEFAULT_OFFSURFACE_LEN;
	this->prepareInterpolation();
}

FastRBF::~FastRBF() {
	m_lpGPUPoly = NULL;
}

bool FastRBF::prepareInterpolation() {
	if(!m_lpGPUPoly)
		return false;


	//Read-back vertices and normals
	U32 ctVertices;
	vector<float> vSurfXYZ;
	vector<float> normals;
	m_lpGPUPoly->readBackNormals(ctVertices, vSurfXYZ, normals);

	U32 ctOffsurfaceVertices = 2*ctVertices;
	vector<float> vOffSurfXYZF;
	vOffSurfXYZF.resize(2 * ctVertices * 4);
	for(U32 i=0; i < ctVertices; i++) {
		vec3f ptOut = vec3f(&vSurfXYZ[i * 3]) + vec3f(&normals[i * 3]) * m_offSurfaceLen;
		vec3f ptIn = vec3f(&vSurfXYZ[i * 3]) - vec3f(&normals[i * 3]) * m_offSurfaceLen;

		vOffSurfXYZF[i * 8] = ptOut.x;
		vOffSurfXYZF[i * 8 + 1] = ptOut.y;
		vOffSurfXYZF[i * 8 + 2] = ptOut.z;
		vOffSurfXYZF[i * 8 + 3] = 1.0;

		vOffSurfXYZF[i * 8 + 4] = ptIn.x;
		vOffSurfXYZF[i * 8 + 5] = ptIn.y;
		vOffSurfXYZF[i * 8 + 6] = ptIn.z;
		vOffSurfXYZF[i * 8 + 7] = 1.0;
	}

	m_lpGPUPoly->computeFieldArray(ctVertices * 2, 4, vOffSurfXYZF);

//	U32 ctOffsurfaceVertices;
//	vector<float> vOffSurfXYZF;
//	m_lpGPUPoly->computeOffSurfacePointsAndFields(2, 0.1f, ctOffsurfaceVertices, vOffSurfXYZF);
//	printf("\nVertices: %d, OFFSURF Size: %d\n", ctOffsurfaceVertices, vOffSurfXYZF.size());
//	PrintArrayF(&vOffSurfXYZF[0], vOffSurfXYZF.size());

//	vec3f ptOut = vec3f(&vOffSurfXYZF[0]);
//	vec3f ptIn = vec3f(&vOffSurfXYZF[4]);
//	vec3f ptSurf = vec3f(&vSurfXYZ[0]);
//	printf("\n Dist1: %.2f, Dist2: %.2f \n", vec3f::distance(ptSurf, ptOut), vec3f::distance(ptSurf, ptIn));
//	printf("\n Out and In Vertices: \n");
//	PrintArrayF(&vOffSurfXYZF[0], 8);
//	printf("\n Surface Vertex: \n");
//	PrintArrayF(&vSurfXYZ[0], 3);

	m_centers.reserve(ctVertices * 2 * 3);
	m_fields.reserve(ctVertices*2);

	//Every other vertex will have its off-surface points
	for(U32 i=0; i<ctVertices; i++) {
		m_centers.push_back(vSurfXYZ[i * 3]);
		m_centers.push_back(vSurfXYZ[i * 3 + 1]);
		m_centers.push_back(vSurfXYZ[i * 3 + 2]);
		m_fields.push_back(ISO_VALUE - ISO_VALUE);
		if(i%2 == 0) {
			m_centers.push_back(vOffSurfXYZF[i*8]);
			m_centers.push_back(vOffSurfXYZF[i*8 + 1]);
			m_centers.push_back(vOffSurfXYZF[i*8 + 2]);
			m_fields.push_back(vOffSurfXYZF[i*8 + 3] - ISO_VALUE);

			m_centers.push_back(vOffSurfXYZF[i*8 + 4]);
			m_centers.push_back(vOffSurfXYZF[i*8 + 5]);
			m_centers.push_back(vOffSurfXYZF[i*8 + 6]);
			m_fields.push_back(vOffSurfXYZF[i*8 + 7] - ISO_VALUE);
		}
	}

//	PrintArrayF(&m_centers[0], m_centers.size());
//	PrintArrayF(&m_fields[0], 100);

	//Number of interpolation m_centers
	m_ctCenters = m_centers.size() / 3;
	MatrixXd A(m_ctCenters, m_ctCenters);

	vec3f xi, xj;
	U32 ctElements = 0;
	//Row
	for(U32 i=0; i<m_ctCenters; i++) {
		xi = vec3f(&m_centers[i*3]);

		//Col
		for(U32 j=0; j<m_ctCenters; j++) {
			xj = vec3f(&m_centers[j*3]);
			A(i, j) = vec3f::distance(xi, xj);
			//A(j, i) = A(i, j);
			ctElements ++;
		}
	}

	MatrixXd F(m_ctCenters, 1);
	for(U32 i=0; i<m_ctCenters; i++) {
		F(i, 0) = m_fields[i];
	}
//	std::cout << "A Matrix: " << m_ctCenters << " x " << m_ctCenters << endl;
//	std::cout << A << endl;
	std::cout << "F Matrix: " << m_ctCenters << " x 1" << endl;
	std::cout << F << endl;

	//
	MatrixXd Lamda = A.colPivHouseholderQr().solve(F);
	m_lambda.resize(Lamda.rows());
	for(U32 i=0; i < Lamda.rows(); i++)
		m_lambda[i] = Lamda(i, 0);
//	PrintArrayF(&m_lambda[0], m_lambda.size());

	m_solutionError = (A*Lamda - F).norm() / F.norm();
	printf("Error expected to be: %f\n", m_solutionError);

	testWithVoxelGridSamples();
	float f = fieldRBF(vec3f(0,0,0));

	return true;
}

bool FastRBF::testWithVoxelGridSamples() {
	vec4u dim;
	vector<float> arrXYZF;
	bool bres = m_lpGPUPoly->readBackVoxelGridSamples(dim, arrXYZF);
	if(!bres)
		return false;

	float error = 0.0f;
	for(U32 i=0; i < dim.w; i++)
	{
		vec4f xyzf = vec4f(&arrXYZF[i * 4]);

		float interpolatedField = fieldRBF(xyzf.getVec3());
		error += Absolutef(interpolatedField - xyzf.w);
	}

	return (error < 0.5);
}


float FastRBF::fieldRBF(const vec3f& v) {
	if(m_ctCenters == 0)
		return 0.0f;

	float sum = 0.0f;
	for(U32 i=0; i < m_ctCenters; i++)
		sum += m_lambda[i] * vec3f::distance(v, vec3f(&m_centers[i * 3]));

	//Return field value in the original format
	return (sum + ISO_VALUE);
}

vec3f FastRBF::gradientRBF(const vec3f& v) {

	return vec3f(0,0,0);
}

}
}

