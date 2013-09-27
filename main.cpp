#include <GL/glew.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <map>

//#include "PS_BlobTreeRender/_CellConfigTableCompact.h";
#include <tbb/task_scheduler_init.h>
#include "PS_Base/PS_MathBase.h"

//#include <GL/glew.h>
#include <GL/freeglut.h>

#include "PS_Base/PS_Logger.h"
#include "PS_Base/PS_FileDirectory.h"
#include "PS_BlobTreeRender/PS_OclPolygonizer.h"
#include "PS_BlobTreeRender/PS_RBF.h"

#include "PS_Graphics/ShaderManager.h"
#include "PS_Graphics/PS_ArcBallCamera.h"
#include "PS_Graphics/PS_SketchConfig.h"
#include "PS_Graphics/AffineWidgets.h"
#include "PS_Graphics/PS_Vector.h"
#include "PS_Graphics/OclRayTracer.h"
#include "PS_Graphics/MeshRenderer.h"
#include "PS_Graphics/PS_DebugUtils.h"
#include "PS_Graphics/SceneGraph.h"
#include "PS_Graphics/CLMeshBuffer.h"
#include "PS_Graphics/Lerping.h"


#include "PS_Deformable/PS_Deformable.h"
#include "PS_Deformable/PS_VegWriter.h"
#include "PS_Deformable/Avatar.h"
#include "PS_Deformable/TetGenExporter.h"
//#include "PS_Deformable/MincReader.h"
#include "PS_Deformable/MassSpringSystem.h"
#include "volumetricMeshLoader.h"
#include "generateSurfaceMesh.h"
#include "PS_Graphics/PS_Particles.h"

using namespace std;
using namespace PS;
using namespace PS::FILESTRINGUTILS;
using namespace PS::HPC;

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 800
#define FOVY 45.0
#define ZNEAR 0.01
#define ZFAR 100.0

#define DEFAULT_FORCE_COEFF 600000
//#define ANIMATION_TIMER_MILLISEC 33
#define ANIMATION_TIMER_MILLISEC 1000/60
#define ANIMATION_TIME_SAMPLE_INTERVAL 5


#define INDEX_GPU_INFO 	   0
#define INDEX_CAMERA_INFO 1
#define INDEX_HAPTIC_INFO 2
#define INDEX_ANIMATION_INFO 3
#define INDEX_MESH_INFO 4



enum HAPTICMODES {hmDynamic, hmSceneEdit};
enum DRAWISOSURFACE {disNone, disWireFrame, disFull, disTetMesh, disCount};

//Application Settings
class AppSettings{
public:

	//Set to default values in constructor
	AppSettings() {
		//Set the simulation file path
		this->strSimFilePath = (string)(ChangeFileExt(ExtractFilePath(GetExePath()), DAnsiStr(".sim")).c_str());
		this->appWidth = WINDOW_WIDTH;
		this->appHeight = WINDOW_HEIGHT;
		this->hapticNeighborhoodPropagationRadius = DEFAULT_FORCE_NEIGHBORHOOD_SIZE;
		this->collisionDist = 0;
		this->groundLevel = 0.0f;
		this->drawIsoSurface = disFull;
		this->bEditConstrainedNodes = false;
		this->bPanCamera = false;
		this->bDrawAABB = true;
		this->bDrawNormals = true;

		this->bDrawTetMesh = true;
		this->bDrawAffineWidgets = true;
		this->bDrawGround = true;
		this->bDrawAvatar = true;

		this->bLogSql = true;
		this->bGravity = true;
		this->idxCollisionFace = -1;
		this->animTimerInterval = ANIMATION_TIMER_MILLISEC;
		this->ctAnimFrame = 0;
		this->ctAnimLogger = 0;
		this->ctLogsCollected = 0;
		this->ctSolverThreads = task_scheduler_init::default_num_threads();

		//Animation
		this->msAnimationTimeAcc = 0;
		this->animFPS = 0;


		this->hapticMode = 0;
		this->hapticForceCoeff = DEFAULT_FORCE_COEFF;
		this->cellsize = DEFAULT_CELL_SIZE;
	}

public:
	string strSimFilePath;
	string strModelFilePath;
	int  drawIsoSurface;
	bool bDrawAvatar;
	bool bDrawGround;
	bool bPanCamera;
	bool bDrawAABB;
	bool bDrawNormals;
	bool bDrawTetMesh;
	bool bDrawAffineWidgets;
	bool bLogSql;
	bool bGravity;
	bool bEditConstrainedNodes;

	int idxCollisionFace;
	double collisionDist;
	vec3d collisionClosestPoint;
	double hapticForceCoeff;


	//AppSettings
	int appWidth;
	int appHeight;
	int hapticMode;

	float cellsize;
	float groundLevel;
	//vec3d worldAvatarPos;
	//vec3d worldDragStart;
	//vec3d worldDragEnd;
	vec2i screenDragStart;
	vec2i screenDragEnd;

	//Fixed Vertices
	vector<int> vFixedVertices;

	//Propagate force to neighborhood
	int hapticNeighborhoodPropagationRadius;

	//Stats
	U64  ctLogsCollected;
	U64  ctAnimFrame;
	U64  ctAnimLogger;
	U32  animTimerInterval;
	double msAnimationFrameTime;
	double msAnimationTimeAcc;

	double msAnimApplyDisplacements;
	double msPolyTriangleMesh;
	double msPolyTetrahedraMesh;
	double msRBFCreation;
	int ctSolverThreads;
	int animFPS;
};

void AdvanceHapticPosition();


std::map<int, vec3d> g_hashVertices;

//Visual Cues
AvatarCube* g_lpAvatarCube = NULL;
//AvatarScalpel* g_lpAvatarScalpel = NULL;

AbstractWidget*	g_lpAffineWidget = NULL;
GLMeshBuffer* g_lpGroundMatrix = NULL;
GLMeshBuffer* g_lpSceneBox = NULL;
GLMeshBuffer* g_lpDrawNormals = NULL;

//SpringDumble* g_lpMassSpring = NULL;
//Particles* g_lpParticles = NULL;

//SimulationObjects
PS::ArcBallCamera 	g_arcBallCam;
PS::HPC::FastRBF*	g_lpFastRBF = NULL;
Deformable* 		g_lpDeformable = NULL;

//Info Lines
std::vector<std::string> g_infoLines;
AppSettings g_appSettings;
U32 g_uiHardEdgesShader;
U32 g_uiPhongShader;


////////////////////////////////////////////////
//Function Prototype
void Close();
void Draw();
void Resize(int w, int h);
void TimeStep(int t);
void MousePress(int button, int state, int x, int y);
void MouseMove(int x, int y);
void MousePassiveMove(int x, int y);
void MouseWheel(int button, int dir, int x, int y);

void ApplyDeformations(U32 dof, double* displacements);

void NormalKey(unsigned char key, int x, int y);
void SpecialKey(int key, int x, int y);

void DrawText(const char* chrText, int x, int y);
string GetGPUInfo();

//Settings
bool LoadSettings(const std::string& strSimFP);
bool SaveSettings(const std::string& strSimFP);
////////////////////////////////////////////////
//Vertex Shader Code
const char * g_lpVertexShaderCode = 
	"varying vec3 N;"
	"varying vec3 V; "
	"void main(void) {"
	"gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;"
	"gl_FrontColor = gl_Color;"
	"N = normalize(gl_NormalMatrix * gl_Normal);"
	"V = vec3(gl_ModelViewMatrix * gl_Vertex); }";

//Fragment Shader Code
const char* g_lpFragShaderCode =
	"varying vec3 N;"
	"varying vec3 V;"
	"void main(void) {"
	"vec3 L = normalize(gl_LightSource[0].position.xyz - V);"
	"vec3 E = normalize(-V);"
	"vec3 R = normalize(-reflect(L, N));"
	"vec4 Iamb = 0.5 * gl_LightSource[0].ambient * gl_Color;"
	"vec4 Idif = (gl_LightSource[0].diffuse * gl_Color) * max(dot(N,L), 0.0);"
	"vec4 Ispec = (gl_LightSource[0].specular * (vec4(0.8, 0.8, 0.8, 0.8) + 0.2 * gl_Color)) * pow(max(dot(R, E), 0.0), 32.0);"
	"gl_FragColor = gl_FrontLightModelProduct.sceneColor + Iamb + Idif + Ispec;	}";

////////////////////////////////////////////////////////////////////////////////////////

void AdvanceHapticPosition() {
//	if(g_lpDeformable->isHapticInProgress() && g_appSettings.hapticMode == hmDynamic) {
//		vec2f v = g_hapticPosLerp.value();
//		printf("Hx= %.3f, Hy= %.3f\n", v.x, v.y);
//	}
}

void Draw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Render
	g_arcBallCam.look();

	//Draw Box
	if(g_lpSceneBox) {
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glDisable(GL_CULL_FACE);
		g_lpSceneBox->draw();

		glEnable(GL_CULL_FACE);
		glPopAttrib();
	}

	//Draw Ground Level
	if(g_lpGroundMatrix && g_appSettings.bDrawGround) {
		glPushMatrix();
		glTranslatef(0.0f, g_appSettings.groundLevel, 0.0f);
		g_lpGroundMatrix->draw();
		glPopMatrix();
	}

	//1.Draw RBF
	if(g_lpFastRBF && (g_appSettings.drawIsoSurface != disNone)) {
		g_lpFastRBF->setWireFrameMode(g_appSettings.drawIsoSurface == disWireFrame);

		//Draw AABB and Normals
		if(g_appSettings.bDrawAABB) {
			g_lpFastRBF->drawBBox();
		}

		//Draw Model Normals
		if(g_lpDrawNormals && (g_appSettings.bDrawNormals))
			g_lpDrawNormals->draw();

		//Draw Collision
		g_lpFastRBF->drawCollision();

		glEnable(GL_LIGHTING);
			g_lpFastRBF->draw();
		glDisable(GL_LIGHTING);
	}


	//2.Draw Deformable Mesh and cuts
	if(g_lpDeformable) {
		if(g_appSettings.bDrawTetMesh)
			g_lpDeformable->draw();

		//Draw Cutting
		g_lpDeformable->drawCuttingArea();
	}

	//3.Draw Avatar
	if(g_lpAvatarCube && g_appSettings.bDrawAvatar)
	{
		vec3f wpos = TheUITransform::Instance().translate;
		glPushMatrix();
			glTranslatef(wpos.x, wpos.y, wpos.z);

			//Draw Avatar Filled
			glEnable(GL_LIGHTING);
			g_lpAvatarCube->setShaderEffectProgram(g_uiPhongShader);
			g_lpAvatarCube->setWireFrameMode(false);
			g_lpAvatarCube->draw();
			glDisable(GL_LIGHTING);

			//Draw Edges
			g_lpAvatarCube->setShaderEffectProgram(g_uiHardEdgesShader);
			g_lpAvatarCube->setWireFrameMode(true);
			g_lpAvatarCube->draw();

			if (g_appSettings.bDrawAffineWidgets)
			{
				glDisable(GL_DEPTH_TEST);
				g_lpAffineWidget->draw();
				glEnable(GL_DEPTH_TEST);
			}
		glPopMatrix();
	}


	//4.Draw Haptic Line
	if(g_lpDeformable->isHapticInProgress() && g_appSettings.bDrawAffineWidgets)
	{
		GLint vp[4];
		glGetIntegerv(GL_VIEWPORT, vp);
		vec2i s1 = g_appSettings.screenDragStart;
		vec2i s2 = g_appSettings.screenDragEnd;

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0.0, vp[2], vp[3], 0.0, -1.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glColor3f(1,0,0);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glLineWidth(1.0);

		glBegin(GL_LINES);
			glVertex2f(s1.x, s1.y);
			glVertex2f(s2.x, s2.y);
		glEnd();
		glPopAttrib();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	}

	//Write Info
	{
		char chrMsg[1024];
		sprintf(chrMsg,"Camera [Roll=%.1f, Tilt=%.1f, PanX=%.2f, PanY=%.2f]",
				g_arcBallCam.getRoll(),
				g_arcBallCam.getTilt(),
				g_arcBallCam.getPan().x,
				g_arcBallCam.getPan().y);
		g_infoLines[INDEX_CAMERA_INFO] = string(chrMsg);

		//Frame Count, Frame Time and FPS, Log Count
		sprintf(chrMsg, "ANIMATION FRAME# %08llu, TIME# %.2f, FPS# %d, LOGS# %08llu",
				g_appSettings.ctAnimFrame,
				g_appSettings.msAnimationFrameTime,
				g_appSettings.animFPS,
				g_appSettings.ctLogsCollected);
		g_infoLines[INDEX_ANIMATION_INFO] = string(chrMsg);

		if(g_lpFastRBF && g_lpDeformable) {
			sprintf(chrMsg, "MESH CELLSIZE:%.3f, ISOSURF: V# %d, TRI# %d, VOLUME: V# %d, TET# %d",
					g_appSettings.cellsize,
					g_lpFastRBF->countVertices(),
					g_lpFastRBF->countTriangles(),
					g_lpDeformable->getTetMesh()->getNumVertices(),
					g_lpDeformable->getTetMesh()->getNumElements());
			g_infoLines[INDEX_MESH_INFO] = string(chrMsg);
		}
	}

	//Write Model Info
	{
		for(size_t i=0; i<g_infoLines.size(); i++)
			DrawText(g_infoLines[i].c_str(), 10, 20 + i * 15);
	}

	glutSwapBuffers();
}



void DrawText(const char* chrText, int x, int y)
{
	GLint vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, vp[2], vp[3], 0, -1, 1);


	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	float clFont[] = { 0, 0, 1, 1 };
	DrawString(chrText, x,  y, clFont, GLUT_BITMAP_8_BY_13);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

void Resize(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(FOVY, (double)w/(double)h, ZNEAR, ZFAR);
	//glOrtho(-2.0f, 2.0f, -2.0f, 2.0f, -2.0f, 2.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}



void MousePress(int button, int state, int x, int y)
{
	// Wheel reports as button 3(scroll up) and button 4(scroll down)
	if ((button == 3) || (button == 4)) // It's a wheel event
	{
		// Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
		if (state == GLUT_UP)
			return; // Disregard redundant GLUT_UP events

		if(button == 3)
			g_arcBallCam.setZoom(g_arcBallCam.getZoom() - 0.5);
		else
			g_arcBallCam.setZoom(g_arcBallCam.getZoom() + 0.5);

		return;
	}


	//Set Values
	g_appSettings.screenDragStart = vec2i(x, y);
	g_appSettings.screenDragEnd = vec2i(x, y);

	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			//g_lpMassSpring->force();

			if(g_lpAffineWidget->selectAxis(x, y) != uiaFree)
			{
				LogInfoArg1("Affine Widget changed axis to: %d", TheUITransform::Instance().axis);
			}
			else //Check if the user intended to pick a model vertex
			{
				vec3d wpos;
				int stencilValue = ScreenToWorldReadStencil(x, y, wpos);
				if (stencilValue == 1) {
					vec3d closestVertex;
					int idxVertex = g_lpDeformable->pickVertex(wpos, closestVertex);
					g_lpDeformable->setPulledVertex(idxVertex);
					LogInfoArg1("Selected Vertex Index = %d ", idxVertex);

					//Edit Fixed Vertices
					if(g_appSettings.bEditConstrainedNodes) {
						g_appSettings.vFixedVertices.push_back(idxVertex);
						LogInfoArg1("Added last selection to fixed vertices. count = %d", g_appSettings.vFixedVertices.size());
					}
				}
			}
		}
	}
	else if (button == GLUT_RIGHT_BUTTON) {
		if (state == GLUT_DOWN) {
			if (g_lpDeformable->isHapticInProgress()) {
				g_lpDeformable->hapticEnd();
				g_hashVertices.clear();
			}
			else if (g_appSettings.hapticMode == hmDynamic) {
				g_lpDeformable->hapticStart(0);
			}
		}
	} else if (button == GLUT_MIDDLE_BUTTON) {

	}

	//Camera
	g_arcBallCam.mousePress(button, state, x, y);

	//Update selection
	glutPostRedisplay();
}


/*!
 * Passive Move for mouse
 */
void MousePassiveMove(int x, int y)
{
	if(!(g_lpDeformable->isHapticInProgress() && g_appSettings.hapticMode == hmDynamic))
		return;

	float dx = x - g_appSettings.screenDragStart.x;
	float dy = g_appSettings.screenDragStart.y - y;

	dx *= 0.001f;
	dy *= 0.001f;
	g_appSettings.screenDragStart = vec2i(x, y);

	string strAxis;
	vec3f worldAvatarPos = TheUITransform::Instance().translate;

	switch (TheUITransform::Instance().axis) {
	case uiaX:
		worldAvatarPos.x += dx;
		strAxis = "X";
		break;
	case uiaY:
		worldAvatarPos.y += dy;
		strAxis = "Y";
		break;
	case uiaZ:
		worldAvatarPos.z += dx;
		strAxis = "Z";
		break;

	case uiaFree:
		worldAvatarPos = worldAvatarPos + vec3f(dx, dy, 0.0);
		strAxis = "FREE";
		break;
	}

	//World Avatar Pos
	TheUITransform::Instance().translate = worldAvatarPos;
	vec3d wpos = vec3d(worldAvatarPos.x, worldAvatarPos.y, worldAvatarPos.z);

	char buffer[1024];
	sprintf(buffer,
			"HAPTIC DELTA=(%.4f, %.4f), AVATAR=(%.4f, %0.4f, %.4f), AXIS=%s PRESS F4 To Change.",
			dx, dy, wpos.x, wpos.y, wpos.z, strAxis.c_str());
	g_infoLines[INDEX_HAPTIC_INFO] = string(buffer);

	//Avatar corners
	vec3d lower = g_lpAvatarCube->lower() + wpos;
	vec3d upper = g_lpAvatarCube->upper() + wpos;

	//Avatar Box
	AABB aabbAvatar(vec3(lower.x, lower.y, lower.z),
			vec3(upper.x, upper.y, upper.z));

	//1.If boxes donot intersect return
	if (!g_lpFastRBF->bbox().intersect(aabbAvatar)) {
		g_appSettings.idxCollisionFace = -1;
		g_hashVertices.clear();
		g_lpDeformable->cleanupCuttingStructures();
		return;
	}

	//Cutting
	vec3d extent  = (upper - lower);
	extent.y = 0.0;
	g_lpDeformable->performCuts(lower, lower + extent);


	//2.Compute Collision using RBF Interpolation Function
	/*
	 vector<vec3f> avatarVertices;
	 vector<bool> flags;
	 vector<float> penetrations;
	 int idxMaxPenetration;
	 aabbAvatar.getVertices(avatarVertices);
	 g_lpFastRBF->resetCollision();
	 int ctIntersected = g_lpFastRBF->intersects(avatarVertices, flags, penetrations, idxMaxPenetration);
	 if(ctIntersected == 0) {
	 return;
	 }
	 */

	//List all the vertices in the model impacted
	{
		vector<vec3d> arrVertices;
		vector<int> arrIndices;
		g_lpDeformable->pickVertices(lower, upper, arrVertices, arrIndices);

		//Add new vertices
		for (U32 i = 0; i < arrIndices.size(); i++) {
			//If it does not have the vertex then add it. If we have it then the original vertex is used.
			if (g_hashVertices.find(arrIndices[i]) == g_hashVertices.end())
				g_hashVertices.insert(
						std::pair<int, vec3d>(arrIndices[i], arrVertices[i]));
		}
	}

	//If no vertices impacted then return. If we return here then gaps cannot be filled.
	if (g_hashVertices.size() == 0)
		return;

	//Input Arrays
	vec3d n[6];
	vec3d s[6];
	//X. Left and Right
	n[0] = vec3d(-1, 0, 0);
	n[1] = vec3d(1, 0, 0);

	//Y. Bottom and Top
	n[2] = vec3d(0, -1, 0);
	n[3] = vec3d(0, 1, 0);

	//Z. Back and Front
	n[4] = vec3d(0, 0, -1);
	n[5] = vec3d(0, 0, 1);

	//Sample Point to use
	s[0] = lower;
	s[1] = upper;
	s[2] = lower;
	s[3] = upper;
	s[4] = lower;
	s[5] = upper;

	//Detect Collision Face
	//Check against six faces of Avatar to find the intersection
	//Previously Detected Face? If no then detect now
	if (g_appSettings.idxCollisionFace < 0) {
		//g_appSettings.hapticForceCoeff = DEFAULT_FORCE_COEFF;
		double minDot = GetMaxLimit<double>();
		int idxMin = 0;

		//Iterate over vertices in collision
		for (map<int, vec3d>::iterator it = g_hashVertices.begin();
				it != g_hashVertices.end(); ++it) {
			vec3d p = it->second;
			for (int j = 0; j < 6; j++) {
				double dot = vec3d::dot(s[j] - p, n[j]);
				if (dot < minDot) {
					minDot = dot;
					idxMin = j;
					g_appSettings.collisionClosestPoint = p;
				}

				g_appSettings.idxCollisionFace = idxMin;
				g_appSettings.collisionDist = minDot;
			}
		}
	} else {
		double minDot = GetMaxLimit<double>();
		int idxFace = g_appSettings.idxCollisionFace;
		//Iterate over vertices in collision
		for (map<int, vec3d>::iterator it = g_hashVertices.begin();
				it != g_hashVertices.end(); ++it) {
			double dot = vec3d::dot(s[idxFace] - it->second, n[idxFace]);
			if (dot < minDot) {
				minDot = dot;
				g_appSettings.collisionDist = minDot;
				g_appSettings.collisionClosestPoint = it->second;
			}
		}
	}

	//Compute Displacement
	vector<vec3d> arrForces;
	vector<int> arrIndices;
	int idxCFace = g_appSettings.idxCollisionFace;
	for (std::map<int, vec3d>::iterator it = g_hashVertices.begin();
			it != g_hashVertices.end(); ++it) {
		vec3d v = it->second;

		//1000000
		double dot = vec3d::dot(s[idxCFace] - v, n[idxCFace])
				* g_appSettings.hapticForceCoeff;
		dot = MATHMAX(dot, 0);

		/*
		 string arrFaces[] = { "LEFT", "RIGHT", "BOTTOM", "TOP", "NEAR",
		 "FAR" };
		 printf("Face[%d] = %s, dot = %.4f, VERTEX USED: [%.4f, %.4f, %.4f], COEFF: %.2f \n",
		 idxCFace, arrFaces[idxCFace].c_str(), dot, v.x, v.y, v.z,
		 g_appSettings.hapticForceCoeff);
		 */

		arrIndices.push_back(it->first);
		arrForces.push_back(n[idxCFace] * dot);
	}

	//Apply displacements/forces to the model
	g_lpDeformable->hapticSetCurrentForces(arrIndices, arrForces);


	glutPostRedisplay();
}

void MouseMove(int x, int y)
{
	if(g_appSettings.hapticMode == hmDynamic)
		g_arcBallCam.mouseMove(x, y);
	else{
		double d[3];
		d[0] = x - g_appSettings.screenDragStart.x;
		d[1] = g_appSettings.screenDragEnd.y - y;
		d[2] = d[0];
		for(int i=0;i<3;i++)
			d[i] *= 0.001;

		printf("WP = [%.4f, %.4f, %.4f] \n", d[0], d[1], d[2]);

		//Scale
		if(TheUITransform::Instance().type == uitScale)
		{
			int axis = TheUITransform::Instance().axis;
			vec3d inc(0,0,0);
			if(axis < uiaFree)
				inc.setElement(axis, d[axis]);
			else
				inc = vec3d(d);
			inc = inc * 0.5;
			vec3d lo = g_lpAvatarCube->lower() - inc;
			vec3d hi = g_lpAvatarCube->upper() + inc;
			lo = vec3d::minP(lo, hi);
			hi = vec3d::maxP(lo, hi);
			vec3d sides = hi - lo;
			LogInfoArg3("New Avatar Size is [%.3f, %.3f, %.3f]", sides.x, sides.y, sides.z);

			//Rebuild avatar
			SAFE_DELETE(g_lpAvatarCube);
			g_lpAvatarCube = new AvatarCube(lo, hi);
		}
	}
	glutPostRedisplay();
}

void MouseWheel(int button, int dir, int x, int y)
{
	g_arcBallCam.mouseWheel(button, dir, x, y);
	glutPostRedisplay();
}

void ApplyDeformations(U32 dof, double* displacements) {
	tbb::tick_count tsStart = tbb::tick_count::now();
	if(g_lpFastRBF)
		g_lpFastRBF->applyFemDisplacements(dof, displacements);

	g_appSettings.msAnimApplyDisplacements = (tbb::tick_count::now() - tsStart).seconds() * 1000.0;
}

void Close()
{
	//Flush events to db and text log files
	TheDataBaseLogger::Instance().flushAndWait();
	PS::TheEventLogger::Instance().flush();

	//Cleanup
	cout << "Cleanup Memory objects" << endl;

	SAFE_DELETE(g_lpDrawNormals);
	SAFE_DELETE(g_lpFastRBF);
	SAFE_DELETE(g_lpDeformable);
	SAFE_DELETE(g_lpAvatarCube);

}

string QueryOGL(GLenum name)
{
	string strOut = "N/A";
	const char* lpRes = reinterpret_cast<const char*>(glGetString(name));
	if(lpRes == NULL)
		return strOut;
	else
	{
		strOut = string(lpRes);
		return strOut;
	}
}

string GetGPUInfo()
{
	string strVendorName = QueryOGL(GL_VENDOR);
	string strRenderer   = QueryOGL(GL_RENDERER);
	string strVersion	 = QueryOGL(GL_VERSION);
	string strExtensions = QueryOGL(GL_EXTENSIONS);
//	cout << "GPU VENDOR: " << strVendorName << endl;
//	cout << "GPU RENDERER: " << strRenderer << endl;
//	cout << "GPU VERSION: " << strVersion << endl;
	
	LogInfoArg1("GPU VENDOR: %s", strVendorName.c_str());
	LogInfoArg1("GPU RENDERER: %s", strRenderer.c_str());
	LogInfoArg1("GPU VERSION: %s", strVersion.c_str());
	LogInfoArg1("GLSL Version: %s", QueryOGL(GL_SHADING_LANGUAGE_VERSION).c_str());


	if(strcmp(strVendorName.c_str(), "Intel") == 0)
	{
		cout << "WARNING: Integrated GPU is being used!" << endl;
		LogWarning("Non-Discrete GPU selected for rendering!");
	}
	//cout << "GPU EXTENSIONS: " << strExtensions << endl;
	return  string("GPU: ") + strVendorName + ", " + strRenderer + ", " + strVersion;
}



void NormalKey(unsigned char key, int x, int y)
{
	mat44f mtx;
	mtx.scale(vec3f(0.4, 0.4, 0.4));
	switch(key)
	{
	case('+'):{
		g_appSettings.cellsize += 0.01;
		LogInfoArg1("Changed cellsize to: %.2f", g_appSettings.cellsize);
//		g_lpBlobRender->runPolygonizer(g_appSettings.cellsize);
		break;
	}
	case('-'):{
		g_appSettings.cellsize -= 0.01;
		LogInfoArg1("Changed cellsize to: %.2f", g_appSettings.cellsize);
//		g_lpBlobRender->runPolygonizer(g_appSettings.cellsize);
		break;
	}

	case('a'): {
		g_appSettings.bDrawAvatar = !g_appSettings.bDrawAvatar;
		LogInfoArg1("Draw avatar: %d", g_appSettings.bDrawAvatar);
		glutPostRedisplay();
		break;
	}

	case('f'): {
		LogInfo("Begin selecting fixed nodes now!");
		g_appSettings.bEditConstrainedNodes = true;
		g_appSettings.vFixedVertices.resize(0);
		break;
	}
	case('g'):{
		SAFE_DELETE(g_lpAffineWidget);
		g_lpAffineWidget = CreateAffineWidget(uitTranslate);
		g_lpAffineWidget->setTransform(mtx);
		g_appSettings.hapticMode = hmDynamic;
		break;
	}

	case('s'):{
		SAFE_DELETE(g_lpAffineWidget);
		g_lpAffineWidget = CreateAffineWidget(uitScale);
		g_lpAffineWidget->setTransform(mtx);
		g_appSettings.hapticMode = hmSceneEdit;
		break;
	}

	case('r'):{
		SAFE_DELETE(g_lpAffineWidget);
		g_lpAffineWidget = CreateAffineWidget(uitRotate);
		g_lpAffineWidget->setTransform(mtx);
		g_appSettings.hapticMode = hmSceneEdit;
		break;
	}

	case('x'):{
		SAFE_DELETE(g_lpAffineWidget);
		TheUITransform::Instance().axis = uiaX;
		g_lpAffineWidget = CreateAffineWidget(TheUITransform::Instance().type);
		g_lpAffineWidget->setTransform(mtx);
		break;
	}
	case('y'):{
		SAFE_DELETE(g_lpAffineWidget);
		TheUITransform::Instance().axis = uiaY;
		g_lpAffineWidget = CreateAffineWidget(TheUITransform::Instance().type);
		g_lpAffineWidget->setTransform(mtx);
		break;
	}
	case('z'):{
		SAFE_DELETE(g_lpAffineWidget);
		TheUITransform::Instance().axis = uiaZ;
		g_lpAffineWidget = CreateAffineWidget(TheUITransform::Instance().type);
		g_lpAffineWidget->setTransform(mtx);

		break;
	}
	case('d'): {
		g_appSettings.bDrawGround = !g_appSettings.bDrawGround;
		LogInfoArg1("Draw ground checkerboard set to: %d", g_appSettings.bDrawGround);
		glutPostRedisplay();
		break;
	}
	case('n'): {
		g_appSettings.bDrawNormals = !g_appSettings.bDrawNormals;
		LogInfoArg1("Draw Model Normals set to: %d", g_appSettings.bDrawNormals);
		glutPostRedisplay();
		break;
	}

	case('['):{
		g_arcBallCam.setZoom(g_arcBallCam.getZoom() + 0.5);
		glutPostRedisplay();
		break;
	}
	case(']'):{
		g_arcBallCam.setZoom(g_arcBallCam.getZoom() - 0.5);
		glutPostRedisplay();
		break;
	}

	}
}

void SpecialKey(int key, int x, int y)
{
	switch(key)
	{
		case(GLUT_KEY_F1):
		{
			g_appSettings.bDrawTetMesh = !g_appSettings.bDrawTetMesh;
			LogInfoArg1("Draw tetmesh: %d", g_appSettings.bDrawTetMesh);
			break;
		}

		case(GLUT_KEY_F2):
		{
			g_appSettings.drawIsoSurface = (g_appSettings.drawIsoSurface + 1) % disCount;
			LogInfoArg1("Draw isosurf mode: %d", g_appSettings.drawIsoSurface);
			break;
		}

		case(GLUT_KEY_F3):
		{
			g_appSettings.bDrawAABB = !g_appSettings.bDrawAABB;
			LogInfoArg1("Draw the AABB: %d", g_appSettings.bDrawAABB);
			break;
		}

		case(GLUT_KEY_F4):
		{
			//Set UIAxis
			TheUITransform::Instance().axis = (TheUITransform::Instance().axis + 1) % 4;
			LogInfoArg1("Change haptic axis to %d", TheUITransform::Instance().axis);
			g_lpAffineWidget->createWidget();

			break;
		}

		case(GLUT_KEY_F5):
		{
			int axis = TheUITransform::Instance().axis;
			vec3d lo = g_lpAvatarCube->lower();
			vec3d hi = g_lpAvatarCube->upper();

			vec3d inc(0,0,0);
			if(axis < uiaFree)
				inc.setElement(axis, -0.1);
			else
				inc = vec3d(-0.1, -0.1, -0.1);
			inc = inc * 0.5;
			SAFE_DELETE(g_lpAvatarCube);
			g_lpAvatarCube = new AvatarCube(lo - inc, hi + inc);
			break;
		}

		case(GLUT_KEY_F6):
		{
			int axis = TheUITransform::Instance().axis;
			vec3d lo = g_lpAvatarCube->lower();
			vec3d hi = g_lpAvatarCube->upper();
			vec3d inc(0,0,0);
			if(axis < uiaFree)
				inc.setElement(axis, 0.1);
			else
				inc = vec3d(0.1, 0.1, 0.1);
			inc = inc * 0.5;
			SAFE_DELETE(g_lpAvatarCube);
			g_lpAvatarCube = new AvatarCube(lo - inc, hi + inc);
			break;
		}

		case(GLUT_KEY_F7):
		{
			int radius = MATHMAX(g_lpDeformable->getHapticForceRadius() - 1, 1);
			g_lpDeformable->setHapticForceRadius(radius);
			LogInfoArg1("Decrease haptic force radius: %d", radius);
			break;
		}

		case(GLUT_KEY_F8):
		{
			int radius = MATHMIN(g_lpDeformable->getHapticForceRadius() + 1, 10);
			g_lpDeformable->setHapticForceRadius(radius);
			LogInfoArg1("Increase haptic force radius: %d", radius);
			break;
		}

		case(GLUT_KEY_F9):
		{
			g_appSettings.bDrawAffineWidgets = !g_appSettings.bDrawAffineWidgets;
			LogInfoArg1("Draw affine widgets: %d", g_appSettings.bDrawAffineWidgets);
			break;
		}

		case(GLUT_KEY_F11):
		{
			//Saving Settings and Exit
			SaveSettings(g_appSettings.strSimFilePath);

			LogInfo("Exiting.");

			glutLeaveMainLoop();
			break;
		}

	}

	glutPostRedisplay();
}


void TimeStep(int t)
{
	tbb::tick_count tkStart = tbb::tick_count::now();

	g_appSettings.msAnimationTimeAcc += (tbb::tick_count::now() - tkStart).seconds() * 1000.0;
	g_appSettings.ctAnimFrame ++;

	//Advance timestep in scenegraph
	//TheSceneGraph::Instance().timestep();

	//Sample For Logs and Info
	if(g_appSettings.ctAnimFrame - g_appSettings.ctAnimLogger > ANIMATION_TIME_SAMPLE_INTERVAL) {

		//Store counter
		g_appSettings.ctAnimLogger = g_appSettings.ctAnimFrame;

		//Compute Time
		g_appSettings.msAnimationFrameTime = g_appSettings.msAnimationTimeAcc / (double)ANIMATION_TIME_SAMPLE_INTERVAL;
		g_appSettings.msAnimationTimeAcc = 0.0;
		g_appSettings.animFPS = 1000 / (g_appSettings.msAnimationFrameTime > 1.0 ? (int)g_appSettings.msAnimationFrameTime : 1);

		//Log Database
		if(g_appSettings.bLogSql)
		{
			if(g_lpDeformable->isVolumeChanged())
			{
				DBLogger::Record rec;
				g_lpDeformable->statFillRecord(rec);

				//StatInfo
				rec.ctSolverThreads = g_appSettings.ctSolverThreads;
				rec.animFPS = g_appSettings.animFPS;
				rec.cellsize = g_appSettings.cellsize;
				rec.msAnimTotalFrame = g_appSettings.msAnimationFrameTime;
				rec.msAnimSysSolver  = g_lpDeformable->getSolverTime() * 1000.0;
				rec.msAnimApplyDisplacements = g_appSettings.msAnimApplyDisplacements;
				rec.msPolyTriangleMesh 		 = g_appSettings.msPolyTriangleMesh;
				rec.msPolyTetrahedraMesh 	 = g_appSettings.msPolyTetrahedraMesh;
				rec.msRBFCreation 			 = g_appSettings.msRBFCreation;
				rec.msRBFEvaluation = 0.0;
				TheDataBaseLogger::Instance().append(rec);

				g_appSettings.ctLogsCollected ++;
			}
		}

	}

	//Animation Frame Prepared now display
	glutPostRedisplay();

	//Set Timer for next frame
	glutTimerFunc(g_appSettings.animTimerInterval, TimeStep, g_appSettings.ctAnimFrame);
}

bool LoadSettings(const std::string& strSimFP)
{
	if(!PS::FILESTRINGUTILS::FileExists(DAnsiStr(strSimFP.c_str()))) {
		LogErrorArg1("SIM file not found at: %s", strSimFP.c_str());
		return false;
	}

	//Loading Settings
	LogInfo("Loading Settings from the ini file.");
	CSketchConfig cfg(DAnsiStr(strSimFP.c_str()), CSketchConfig::fmRead);

	//READ MODEL FILE
	bool isRelativePath = cfg.readBool("MODEL", "RELATIVEPATH", true);
	string strSimFileDir = (string)(PS::FILESTRINGUTILS::ExtractFilePath(DAnsiStr(strSimFP.c_str())).c_str());
	if(isRelativePath)
		g_appSettings.strModelFilePath = strSimFileDir + (string)(cfg.readString("MODEL", "INPUTFILE").c_str());
	else
		g_appSettings.strModelFilePath = (string)(cfg.readString("MODEL", "INPUTFILE").c_str());


	int ctFixed = 	cfg.readInt("MODEL", "FIXEDVERTICESCOUNT", 0);
	if(ctFixed > 0) {
		if(!cfg.readIntArray("MODEL", "FIXEDVERTICES", ctFixed, g_appSettings.vFixedVertices))
			LogError("Unable to read specified number of fixed vertices!");
	}

	//Translation Widget
	TheUITransform::Instance().axis = uiaX;
	g_lpAffineWidget = new TranslateWidget();
	mat44f mtx;
	mtx.scale(vec3f(0.4f, 0.4f, 0.4f));
	g_lpAffineWidget->setTransform(mtx);

	//System settings
	g_appSettings.groundLevel = cfg.readInt("SYSTEM", "GROUNDLEVEL", 0.0f);
	g_appSettings.hapticForceCoeff = cfg.readInt("SYSTEM", "FORCECOEFF", DEFAULT_FORCE_COEFF);
	g_appSettings.bLogSql = cfg.readBool("SYSTEM", "LOGSQL", g_appSettings.bLogSql);
	g_appSettings.cellsize = cfg.readFloat("SYSTEM", "CELLSIZE", DEFAULT_CELL_SIZE);
	g_appSettings.hapticNeighborhoodPropagationRadius = cfg.readInt("AVATAR", "RADIUS");
	g_appSettings.bGravity = cfg.readBool("SYSTEM", "GRAVITY", true);

	//Avatar
	TheUITransform::Instance().translate = cfg.readVec3f("AVATAR", "POS");
	TheUITransform::Instance().axis = cfg.readInt("AVATAR", "AXIS");
	vec3f thickness = cfg.readVec3f("AVATAR", "THICKNESS");
	vec3d thicknessd = vec3d(thickness.x, thickness.y, thickness.z);
	g_lpAvatarCube = new AvatarCube(thicknessd * (-0.5), thicknessd * 0.5);

	//Loading Camera
	if(cfg.hasSection("CAMERA") >= 0)
	{
		g_arcBallCam.setRoll(cfg.readFloat("CAMERA", "ROLL"));
		g_arcBallCam.setTilt(cfg.readFloat("CAMERA", "TILT"));
		g_arcBallCam.setZoom(cfg.readFloat("CAMERA", "ZOOM"));
		g_arcBallCam.setCenter(cfg.readVec3f("CAMERA", "CENTER"));
		g_arcBallCam.setOrigin(cfg.readVec3f("CAMERA", "ORIGIN"));
		g_arcBallCam.setPan(cfg.readVec2f("CAMERA", "PAN"));
	}

	//DISPLAY SETTINGS
	g_appSettings.bDrawAABB = cfg.readBool("DISPLAY", "AABB", true);
	g_appSettings.bDrawAffineWidgets = cfg.readBool("DISPLAY", "AFFINEWIDGETS", true);
	g_appSettings.bDrawGround = cfg.readBool("DISPLAY", "GROUND", true);
	g_appSettings.bDrawNormals = cfg.readBool("DISPLAY", "NORMALS", true);
	g_appSettings.bDrawTetMesh = cfg.readBool("DISPLAY", "TETMESH", true);
	g_appSettings.drawIsoSurface = cfg.readInt("DISPLAY", "ISOSURF", disFull);

	//DISPLAY INFO
	g_infoLines.push_back(GetGPUInfo());
	g_infoLines.push_back(string("CAMERA"));
	g_infoLines.push_back(string("HAPTIC"));
	g_infoLines.push_back(string("ANIMATION"));
	g_infoLines.push_back(string("MESH"));
	return true;
}

bool SaveSettings(const std::string& strSimFP)
{
	LogInfo("Saving settings to ini file");
	CSketchConfig cfg(DAnsiStr(strSimFP.c_str()), CSketchConfig::fmReadWrite);
	cfg.writeFloat("CAMERA", "ROLL", g_arcBallCam.getRoll());
	cfg.writeFloat("CAMERA", "TILT", g_arcBallCam.getTilt());
	cfg.writeFloat("CAMERA", "ZOOM", g_arcBallCam.getZoom());
	cfg.writeVec3f("CAMERA", "CENTER", g_arcBallCam.getCenter());
	cfg.writeVec3f("CAMERA", "ORIGIN", g_arcBallCam.getOrigin());
	cfg.writeVec2f("CAMERA", "PAN", g_arcBallCam.getPan());

	//Write Cellsize
	cfg.writeFloat("SYSTEM", "CELLSIZE", g_appSettings.cellsize);
	cfg.writeBool("SYSTEM", "GRAVITY", g_appSettings.bDrawGround);

	//Avatar
	vec3d sides = g_lpAvatarCube->upper() - g_lpAvatarCube->lower();
	cfg.writeVec3f("AVATAR", "POS", TheUITransform::Instance().translate);
	cfg.writeVec3f("AVATAR", "THICKNESS", vec3f(sides.x, sides.y, sides.z));
	cfg.writeInt("AVATAR", "AXIS", TheUITransform::Instance().axis);

	//DISPLAY SETTINGS
	cfg.writeBool("DISPLAY", "AABB", g_appSettings.bDrawAABB);
	cfg.writeBool("DISPLAY", "AFFINEWIDGETS", g_appSettings.bDrawAffineWidgets);
	cfg.writeBool("DISPLAY", "GROUND", g_appSettings.bDrawGround);
	cfg.writeBool("DISPLAY", "NORMALS", g_appSettings.bDrawNormals);
	cfg.writeBool("DISPLAY", "TETMESH", g_appSettings.bDrawTetMesh);
	cfg.writeInt("DISPLAY", "ISOSURF", g_appSettings.drawIsoSurface);

	//MODEL PROPS
	if(g_appSettings.bEditConstrainedNodes) {
		cfg.writeInt("MODEL", "FIXEDVERTICESCOUNT", g_appSettings.vFixedVertices.size());
		cfg.writeIntArray("MODEL", "FIXEDVERTICES", g_appSettings.vFixedVertices);
	}

	return true;
}


//Main Loop of Application
int main(int argc, char* argv[])
{
	//Init Task Schedular
	tbb::task_scheduler_init init(task_scheduler_init::default_num_threads());

	printf("***************************************************************************************\n");
	printf("FEMBrain - GPU-Accelerated Animation of Implicit Surfaces using Finite Element Methods.\n");
	printf("[Pourya Shirazian] Email: pouryash@cs.uvic.ca\n");


	//SIM files define the simulation parameters
	//Parse Input Args
	if(argc > 0) {
		int iArg = 0;
		while(iArg < argc) {
			string strArg = string(argv[iArg]);
			if(strArg == "-f") {
				string strSimFP = string(argv[++iArg]);
				g_appSettings.strSimFilePath = string(ExtractFilePath(GetExePath()).c_str()) + strSimFP;
				LogInfoArg1("Simulation filepath selected is %s", strSimFP.c_str());
			}
			else if(strArg == "-t") {
				g_appSettings.ctSolverThreads = atoi(argv[++iArg]);
				LogInfoArg1("Number of solver threads %d", g_appSettings.ctSolverThreads);
			}

			else if(strArg == "-h") {
				printf("Usage: FemBrain -f [SIM FILE] \n");
				printf("-f [SIM FILE] Input file defining the simulation scene.\n");
				exit(0);
			}

			iArg++;
		}
	}

	//Setup the event logger
	//PS::TheEventLogger::Instance().setWriteFlags(PS_LOG_WRITE_EVENTTYPE | PS_LOG_WRITE_TIMESTAMP | PS_LOG_WRITE_SOURCE | PS_LOG_WRITE_TO_SCREEN);
	PS::TheEventLogger::Instance().setWriteFlags(PS_LOG_WRITE_EVENTTYPE | PS_LOG_WRITE_SOURCE | PS_LOG_WRITE_TO_SCREEN);
	LogInfo("Starting FemBrain");
	
	//Initialize app
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("Deformable Tissue Modeling - PhD Project - Pourya Shirazian");
	glutDisplayFunc(Draw);
	glutReshapeFunc(Resize);
	glutMouseFunc(MousePress);
	glutPassiveMotionFunc(MousePassiveMove);
	glutMotionFunc(MouseMove);
	glutMouseWheelFunc(MouseWheel);

	glutKeyboardFunc(NormalKey);
	glutSpecialFunc(SpecialKey);
	glutCloseFunc(Close);
	glutTimerFunc(g_appSettings.animTimerInterval, TimeStep, 0);

	//Print GPU INFO
	GetGPUInfo();

	//Setup Shading Environment
	static const GLfloat lightColor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	static const GLfloat lightPos[4] = { 0.0f, 9.0f, 0.0f, 1.0f };

	//Setup Light0 Position and Color
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightColor);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightColor);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

	//Turn on Light 0
	glEnable(GL_LIGHT0);
	//Enable Lighting
	glEnable(GL_LIGHTING);

	//Enable features we want to use from OpenGL
	glShadeModel(GL_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_LINE_SMOOTH);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_STENCIL_TEST);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

	//glClearColor(0.45f, 0.45f, 0.45f, 1.0f);
	glClearColor(1.0, 1.0, 1.0, 1.0);

	//Compiling shaders
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		//Problem: glewInit failed, something is seriously wrong.
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(1);
	}

	//Build Shaders for drawing the mesh
	DAnsiStr strShaderRoot = ExtractOneLevelUp(ExtractFilePath(GetExePath())) + "data/shaders/";
	DAnsiStr strMeshesRoot = ExtractOneLevelUp(ExtractFilePath(GetExePath())) + "data/models/mesh/";
	DAnsiStr strVPath = strShaderRoot + "avataredges.vsh";
	DAnsiStr strFPath = strShaderRoot + "avataredges.fsh";
	g_uiPhongShader = TheShaderManager::Instance().add(g_lpVertexShaderCode, g_lpFragShaderCode, "phong");
	g_uiHardEdgesShader = TheShaderManager::Instance().addFromFile(strVPath.cptr(), strFPath.cptr());

	//Load Settings
	if(!LoadSettings(g_appSettings.strSimFilePath))
		exit(0);


	//Ground and Room
	TheSceneGraph::Instance().addGroundMatrix(32, 32, 0.2f);
	g_lpGroundMatrix = reinterpret_cast<GLMeshBuffer*>(TheSceneGraph::Instance().last());
	TheSceneGraph::Instance().addSceneBox(AABB(vec3f(-10,-10,-16), vec3f(10,10,16)));
	g_lpSceneBox = reinterpret_cast<GLMeshBuffer*>(TheSceneGraph::Instance().last());
	g_lpSceneBox->setShaderEffectProgram(TheShaderManager::Instance().get("phong"));
	g_lpAvatarCube->setShaderEffectProgram(TheShaderManager::Instance().get("phong"));

	//Check the model file extension
	DAnsiStr strFileExt = ExtractFileExt(DAnsiStr(g_appSettings.strModelFilePath.c_str()));

	//Mesh
	U32 ctVertices = 0;
	U32 ctTriangles = 0;
	vector<float> vertices;
	vector<U32> elements;

	if(strFileExt == DAnsiStr("obj")) {
		LogInfoArg1("Loading input mesh from obj file at: %s", g_appSettings.strModelFilePath.c_str());
		Mesh* lpMesh = new Mesh(g_appSettings.strModelFilePath.c_str());
		AABB aabb = lpMesh->computeBoundingBox();
		//Move to origin
		lpMesh->move(aabb.center() * -1);

		//No triangle polygonization time
		g_appSettings.msPolyTriangleMesh = 0;

		tbb::tick_count tsStart = tbb::tick_count::now();
		g_lpFastRBF = new FastRBF(lpMesh);
		g_appSettings.msRBFCreation = (tbb::tick_count::now() - tsStart).seconds() * 1000.0;

		g_lpDrawNormals = g_lpFastRBF->prepareMeshBufferNormals();

		//Readback mesh
		MeshNode* lpNode = lpMesh->getNode(0);
		if(lpNode != NULL)
			lpNode->readbackMeshV3T3(ctVertices, vertices, ctTriangles, elements);


		SAFE_DELETE(lpMesh);
	}
	else
	{
		LogInfoArg1("Loading input BlobTree file at: %s", g_appSettings.strModelFilePath.c_str());
		GPUPoly* lpBlobRender = new GPUPoly();
		if(!lpBlobRender->readModel(g_appSettings.strModelFilePath.c_str())) {
			LogErrorArg1("Unable to read Blob file at: %s", g_appSettings.strModelFilePath.c_str());
			SAFE_DELETE(lpBlobRender);
			exit(-1);
		}

		LogInfoArg1("Polygonizing BlobTree file with cellsize: %.2f", g_appSettings.cellsize);
		tbb::tick_count tsStart = tbb::tick_count::now();
		lpBlobRender->runPolygonizer(g_appSettings.cellsize);
		g_appSettings.msPolyTriangleMesh = (tbb::tick_count::now() - tsStart).seconds() * 1000.0;

		//Save Mesh
		//CLMeshBuffer::StoreAsObjMesh("CubeStored.obj", lpBlobRender->computeDevice(), lpBlobRender);

		//Create the RBF representation
		tsStart = tbb::tick_count::now();
		g_lpFastRBF = new FastRBF(lpBlobRender);
		g_appSettings.msRBFCreation = (tbb::tick_count::now() - tsStart).seconds() * 1000.0;


		g_lpFastRBF->setShaderEffectProgram(g_uiPhongShader);
		g_lpDrawNormals = g_lpFastRBF->prepareMeshBufferNormals();

		//Read Back Polygonized Mesh
		if(!lpBlobRender->readbackMeshV3T3(ctVertices, vertices, ctTriangles, elements))
			LogError("Unable to read mesh from RBF in the format of V3T3");


		SAFE_DELETE(lpBlobRender);
	}


	{
		DAnsiStr strModelFP = PS::FILESTRINGUTILS::ChangeFileExt(DAnsiStr(g_appSettings.strModelFilePath.c_str()), "");
		DAnsiStr strModelTitleOnly = PS::FILESTRINGUTILS::ExtractFileTitleOnly(strModelFP);
		DAnsiStr strModelMeshV3T3 = strModelFP + "TriMeshV3T3";
		DAnsiStr strModelTetMesh = strModelFP + "TetMesh";
		DAnsiStr strModelTetMeshNodes = strModelFP + "TetMesh.node";
		DAnsiStr strModelTetMeshVegFile = strModelTetMesh + ".veg";
		DAnsiStr strModelTetMeshObjFile = strModelTetMesh + ".obj";


		//Produce Quality Tetrahedral Mesh using TetGen for now
		tbb::tick_count tsStart = tbb::tick_count::now();
		TetGenExporter::tesselate(ctVertices, &vertices[0], ctTriangles, &elements[0],
								  strModelMeshV3T3.cptr(),
								  strModelTetMesh.cptr());
		g_appSettings.msPolyTetrahedraMesh = (tbb::tick_count::now() - tsStart).seconds() * 1000.0;

		//Export a veg file for the Fem Engine
		VegWriter::WriteVegFile(strModelTetMeshNodes.cptr());


		//Create Deformable Model from: Triangle Mesh and Tetrahedra Mesh

		//Obj mesh is useful though for viewing the boundary tetrahedra produced by tetgen
		//Produce special obj mesh with the same vertices and surface triangles
		VolumetricMesh * mesh = VolumetricMeshLoader::load(strModelTetMeshVegFile.ptr());
		GenerateSurfaceMesh generateSurfaceMesh;
		ObjMesh * objMesh = generateSurfaceMesh.ComputeMesh(mesh, true);
		objMesh->save(string(strModelTetMeshObjFile.cptr()));

		//Setup Deformable object
		g_lpDeformable = new Deformable(strModelTetMeshVegFile.cptr(),
										strModelTetMeshObjFile.cptr(),
										g_appSettings.vFixedVertices,
										g_appSettings.ctSolverThreads,
										strModelTitleOnly.cptr());

		//Deformations will be applied using opencl kernel
		LogInfoArg1("GRAVITY is: %d", g_appSettings.bGravity);
		g_lpDeformable->setGravity(g_appSettings.bGravity);
		g_lpDeformable->setDeformCallback(ApplyDeformations);
		g_lpDeformable->setHapticForceRadius(g_appSettings.hapticNeighborhoodPropagationRadius);
		g_lpDeformable->setName("tissue");

		//Add to scene graph
		TheSceneGraph::Instance().add(g_lpDeformable);
	}

	//Particles
	/*
	g_lpParticles = new Particles();
	g_lpParticles->setName("blood");
	TheSceneGraph::Instance().add(g_lpParticles);
	*/


	//Draw the first time
	glutPostRedisplay();

	//Run App
	glutMainLoop();

	return 0;
}

