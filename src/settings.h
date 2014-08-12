/*
 * settings.h
 *
 *  Created on: Mar 8, 2014
 *      Author: pourya
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <vector>
#include "base/Vec.h"

using namespace PS;
using namespace PS::MATH;
using namespace PS::FILESTRINGUTILS;

//Defines
#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 800
#define FOVY 45.0
#define ZNEAR 0.01
#define ZFAR 100.0

#define DEFAULT_FORCE_COEFF 600000
#define ANIMATION_TIME_SAMPLE_INTERVAL 5

#define INDEX_GPU_INFO 	   0
#define INDEX_CAMERA_INFO 1
#define INDEX_HAPTIC_INFO 2
#define INDEX_ANIMATION_INFO 3
#define INDEX_MESH_INFO 4



enum HAPTICMODES {hmDynamic, hmSceneEdit};
enum DRAWISOSURFACE {disNone, disWireFrame, disFull, disCount};

//Application Settings
class AppSettings{
public:
	//Set to default values in constructor
	AppSettings() {
		//Set the simulation file path
		this->strSimFilePath = ChangeFileExt(ExtractFilePath(GetExePath()), AnsiStr(".sim"));
		this->appWidth = WINDOW_WIDTH;
		this->appHeight = WINDOW_HEIGHT;
		this->hapticForceCoeff = DEFAULT_FORCE_COEFF;
		this->hapticNeighborhoodPropagationRadius = DEFAULT_FORCE_NEIGHBORHOOD_SIZE;
		this->groundLevel = 0.0f;
		this->drawIsoSurface = disFull;
		this->drawTetMesh = disFull;

		this->selectFixedNodes = false;
		this->panCamera = false;
		this->drawAABB = true;
		this->drawNormals = true;
		this->drawVoxels = true;


		this->drawAffineWidgets = true;
		this->drawGround = true;
		this->drawAvatar = true;

		this->logSql = true;
		this->gravity = true;
		this->ctSolverThreads = task_scheduler_init::default_num_threads();

		//Avatar
		this->avatarThickness = vec3f(0.5);
		this->avatarPos = vec3f(0.0f, 3.0f, 0.0f);
		this->avatarAxis = 1;

		this->hapticMode = 0;
		this->cellsize = DEFAULT_CELL_SIZE;
	}

public:
	AnsiStr strSimFilePath;
	AnsiStr strModelFilePath;
	int  drawIsoSurface;
	int  drawTetMesh;
	bool drawAvatar;
	bool drawGround;
	bool drawVoxels;
	bool panCamera;
	bool drawAABB;
	bool drawNormals;

	bool drawAffineWidgets;
	bool logSql;
	bool gravity;
	bool selectFixedNodes;

	//AppSettings
	int appWidth;
	int appHeight;
	int hapticMode;

	float cellsize;
	float groundLevel;
	//vec3d worldAvatarPos;
	//vec3d worldDragStart;
	//vec3d worldDragEnd;
	int avatarAxis;
	vec3f avatarThickness;
	vec3f avatarPos;
	vec2i screenDragStart;
	vec2i screenDragEnd;

	//Fixed Vertices
	vector<int> vFixedNodes;

	//Propagate force to neighborhood
	int hapticForceCoeff;
	int hapticNeighborhoodPropagationRadius;

	//Other timings
	double msAnimApplyDisplacements;
	double msPolyTriangleMesh;
	double msPolyTetrahedraMesh;
	double msRBFCreation;
	int ctSolverThreads;
};


#endif /* SETTINGS_H_ */
