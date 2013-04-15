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
#include "PS_Graphics/PS_GLSurface.h"
#include "PS_Graphics/PS_SketchConfig.h"
#include "PS_Graphics/AffineWidgets.h"
#include "PS_Graphics/PS_Vector.h"
#include "PS_Graphics/OclRayTracer.h"
#include "PS_Graphics/GroundMartrix.h"
#include "PS_Graphics/SceneBox.h"
#include "PS_Graphics/MeshRenderer.h"
#include "PS_Graphics/PS_DebugUtils.h"


#include "PS_Deformable/PS_Deformable.h"
#include "PS_Deformable/PS_VegWriter.h"
#include "PS_Deformable/Avatar.h"
#include "PS_Deformable/TetGenExporter.h"
//#include "PS_Deformable/MincReader.h"

#include "volumetricMeshLoader.h"
#include "generateSurfaceMesh.h"


using namespace std;
using namespace PS;
using namespace PS::FILESTRINGUTILS;
using namespace PS::HPC;

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 800
#define FOVY 45.0
#define ZNEAR 0.01
#define ZFAR 100.0
#define DEFAULT_TIMER_MILLIS 33
#define DEFAULT_FORCE_COEFF 600000

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
		this->strSimFilePath = (string)(ChangeFileExt(ExtractFilePath(GetExePath()), DAnsiStr(".sim")).c_str());
		this->appWidth = WINDOW_WIDTH;
		this->appHeight = WINDOW_HEIGHT;
		this->hapticNeighborhoodPropagationRadius = DEFAULT_FORCE_NEIGHBORHOOD_SIZE;
		this->currentCollisionFaceModelDist = this->initialCollisionFaceModelDist = 0;
		this->groundLevel = 0.0f;
		this->drawIsoSurface = disFull;
		this->bPanCamera = false;
		this->bDrawAABB = true;
		this->bDrawTetMesh = true;
		this->bDrawAffineWidgets = true;
		this->bDrawGround = true;

		this->enablePhysics = true;
		this->bLogSql = true;
		this->idxCollisionFace = -1;
		this->timerInterval = DEFAULT_TIMER_MILLIS;
		this->ctAnimFrame = 0;
		this->ctAnimLogger = 0;
		this->ctLogsCollected = 0;
		this->hapticMode = 0;
		this->hapticForceCoeff = DEFAULT_FORCE_COEFF;
		this->cellsize = DEFAULT_CELL_SIZE;
	}

public:
	string strSimFilePath;
	string strModelFilePath;
	int  drawIsoSurface;
	bool bDrawGround;
	bool bPanCamera;
	bool bDrawAABB;
	bool bDrawTetMesh;
	bool bDrawAffineWidgets;
	bool bLogSql;
	bool enablePhysics;

	int idxCollisionFace;
	double initialCollisionFaceModelDist;
	double currentCollisionFaceModelDist;
	double hapticForceCoeff;

	U32  timerInterval;
	U64  ctAnimFrame;
	U64  ctAnimLogger;
	U64  ctLogsCollected;

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
};

std::map<int, vec3d> g_hashVertices;

//Visual Cues
AvatarCube* g_lpAvatarCube = NULL;
AbstractWidget*	g_lpAffineWidget = NULL;
GroundMatrix* g_lpGroundMatrix = NULL;
SceneBox* g_lpSceneBox = NULL;
GLMeshBuffer* g_lpDrawNormals = NULL;

//SimulationObjects
PS::ArcBallCamera 	g_arcBallCam;
PS::HPC::GPUPoly* 	g_lpBlobRender = NULL;
PS::HPC::FastRBF*	g_lpFastRBF = NULL;
Deformable* 		g_lpDeformable = NULL;

//Info Lines
std::vector<std::string> g_infoLines;

GLSurface* g_lpSurface = NULL;
AppSettings g_appSettings;


////////////////////////////////////////////////
//Function Prototype
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

	//Draw Deformable Mesh
	if(g_lpDeformable && g_appSettings.bDrawTetMesh)
		g_lpDeformable->draw();

	if(g_lpFastRBF && (g_appSettings.drawIsoSurface != disNone)) {
		g_lpFastRBF->setWireFrameMode(g_appSettings.drawIsoSurface == disWireFrame);

		//Draw AABB and Normals
		if(g_appSettings.bDrawAABB) {
			g_lpFastRBF->drawBBox();
			if(g_lpDrawNormals)
				g_lpDrawNormals->draw();
		}

		glEnable(GL_LIGHTING);
			g_lpFastRBF->draw();
		glDisable(GL_LIGHTING);
	}

	//Draw Polygonizer output
	/*
	if(g_lpBlobRender && (g_appSettings.drawIsoSurface != disNone))
	{
		g_lpBlobRender->setWireFrameMode(g_appSettings.drawIsoSurface == disWireFrame);

		//Draw AABB and Normals
		if(g_appSettings.bDrawAABB) {
			g_lpBlobRender->drawBBox();

			if(g_lpDrawNormals)
				g_lpDrawNormals->draw();
		}

		glEnable(GL_LIGHTING);
			g_lpBlobRender->draw();
		glDisable(GL_LIGHTING);
	}
	*/


	//Draw Interaction Avatar
	if(g_lpAvatarCube)
	{
		vec3f wpos = TheUITransform::Instance().translate;
		glPushMatrix();
			glTranslatef(wpos.x, wpos.y, wpos.z);
			g_lpAvatarCube->draw();

			if (g_appSettings.bDrawAffineWidgets)
			{
				glDisable(GL_DEPTH_TEST);
				g_lpAffineWidget->draw();
				glEnable(GL_DEPTH_TEST);
			}
		glPopMatrix();
	}


	//Draw Haptic Line
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

		sprintf(chrMsg, "ANIMATION FRAME# %08llu, LOGS# %08llu, ", g_appSettings.ctAnimFrame, g_appSettings.ctLogsCollected);
		g_infoLines[INDEX_ANIMATION_INFO] = string(chrMsg);

		/*
		if(g_lpBlobRender && g_lpDeformable) {
			sprintf(chrMsg, "MESH CELLSIZE:%.3f, ISOSURF: V# %d, TRI# %d, VOLUME: V# %d, TET# %d",
					g_appSettings.cellsize,
					g_lpBlobRender->countVertices(),
					g_lpBlobRender->countTriangles(),
					g_lpDeformable->getTetMesh()->getNumVertices(),
					g_lpDeformable->getTetMesh()->getNumElements());
			g_infoLines[INDEX_MESH_INFO] = string(chrMsg);
		}
		*/
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


	g_appSettings.screenDragStart = vec2i(x, y);
	g_appSettings.screenDragEnd = vec2i(x, y);

	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
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
				}
			}
		}
	}
	else if (button == GLUT_RIGHT_BUTTON) {
		if (state == GLUT_DOWN) {
			if (g_lpDeformable->isHapticInProgress())
				g_lpDeformable->hapticEnd();
			else if (g_appSettings.hapticMode == hmDynamic) {
				g_lpDeformable->hapticStart(0);
				g_hashVertices.clear();
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
	if(g_lpDeformable->isHapticInProgress() && g_appSettings.hapticMode == hmDynamic)
	{
		float dx = x - g_appSettings.screenDragStart.x;
		float dy = g_appSettings.screenDragStart.y - y;
		dx *= 0.001f;
		dy *= 0.001f;
		g_appSettings.screenDragStart = vec2i(x, y);
		string strAxis;
		vec3f worldAvatarPos = TheUITransform::Instance().translate;

		switch(TheUITransform::Instance().axis)
		{
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
		sprintf(buffer, "HAPTIC DELTA=(%.4f, %.4f), AVATAR=(%.4f, %0.4f, %.4f), AXIS=%s PRESS F4 To Change.",
				 dx, dy, wpos.x, wpos.y, wpos.z, strAxis.c_str());
		g_infoLines[INDEX_HAPTIC_INFO] = string(buffer);

		vec3d lower = g_lpAvatarCube->lower() + wpos;
		vec3d upper = g_lpAvatarCube->upper() + wpos;
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

		//List all the vertices in the model impacted
		{
			vector<vec3d> arrVertices;
			vector<int> arrIndices;
			g_lpDeformable->pickVertices(lower, upper, arrVertices, arrIndices);
			for(U32 i=0; i<arrIndices.size(); i++)
			{
				printf("Collision Index = %d \n", arrIndices[i]);
				//If it does not have the vertex then add it. If we have it then the original vertex is used.
				if(g_hashVertices.find(arrIndices[i]) == g_hashVertices.end())
					g_hashVertices.insert(std::pair<int, vec3d>(arrIndices[i], arrVertices[i]));
				//else
					//g_hashVertices[arrIndices[i]] = arrVertices[i];
			}
		}

		AABB aabbAvatar(vec3(lower.x, lower.y, lower.z), vec3(upper.x, upper.y, upper.z));
		//Collided Now
		if(g_lpDeformable->aabb().intersect(aabbAvatar))
		{
			printf("INTERSECTS\n");

			//Detect Collision Face
			//Check against six faces of Avatar to find the intersection
			//Previously Detected Face? If no then detect now
			if(g_appSettings.idxCollisionFace < 0)
			{
				//g_appSettings.hapticForceCoeff = DEFAULT_FORCE_COEFF;
				double minDot = GetMaxLimit<double>();
				int idxMin = 0;

				//Iterate over vertices in collision
				for(map<int, vec3d>::iterator it = g_hashVertices.begin(); it != g_hashVertices.end(); ++it)
				{
					vec3d p = it->second;
					for(int j=0; j<6; j++)
					{
						double dot = vec3d::dot(s[j] - p, n[j]);
						if(dot < minDot)
						{
							minDot = dot;
							idxMin = j;
						}

						g_appSettings.idxCollisionFace = idxMin;
						g_appSettings.initialCollisionFaceModelDist = minDot;
					}
				}
			}
			else
			{
				double minDot = GetMaxLimit<double>();
				int idxFace = g_appSettings.idxCollisionFace;
				//Iterate over vertices in collision
				for(map<int, vec3d>::iterator it = g_hashVertices.begin(); it != g_hashVertices.end(); ++it)
				{
					double dot = vec3d::dot(s[idxFace] - it->second, n[idxFace]);
					if(dot < minDot)
					{
						minDot = dot;
						g_appSettings.currentCollisionFaceModelDist = minDot;
					}
				}
			}


			//Compute Displacement
			vector<vec3d> arrForces;
			vector<int> arrIndices;
			int idxFace = g_appSettings.idxCollisionFace;
			for(std::map<int, vec3d>::iterator it = g_hashVertices.begin(); it != g_hashVertices.end(); ++it)
			{
				vec3d v = it->second;
/*
				if(g_appSettings.currentCollisionFaceModelDist > g_appSettings.initialCollisionFaceModelDist)
					g_appSettings.hapticForceCoeff -= 1000;
				else
					g_appSettings.hapticForceCoeff += 1000;
*/

				//1000000
				double dot = vec3d::dot(s[idxFace] - v, n[idxFace]) * g_appSettings.hapticForceCoeff;
				//double dot = vec3d::dot(s[idxFace] - v, n[idxFace]);
				string arrFaces [] = {"LEFT", "RIGHT", "BOTTOM", "TOP", "NEAR", "FAR"};
				printf("Face[%d] = %s, dot = %.4f, VERTEX USED: [%.4f, %.4f, %.4f], COEFF: %.2f \n",
						idxFace, arrFaces[idxFace].c_str(), dot, v.x, v.y, v.z, g_appSettings.hapticForceCoeff);
				arrForces.push_back(n[idxFace] * dot);
				arrIndices.push_back(it->first);
			}

			//Apply displacements/forces to the model
			g_lpDeformable->hapticSetCurrentForces(arrIndices, arrForces);

		}
		else
			g_appSettings.idxCollisionFace = -1;
		glutPostRedisplay();
	}
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
			vec3d lo = g_lpAvatarCube->lower();
			vec3d hi = g_lpAvatarCube->upper();

			vec3d inc(0,0,0);
			if(axis < uiaFree)
				inc.setElement(axis, d[axis]);
			else
				inc = vec3d(d);
			inc = inc * 0.5;
			SAFE_DELETE(g_lpAvatarCube);
			g_lpAvatarCube = new AvatarCube(lo - inc, hi + inc);

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
//	if(g_lpBlobRender)
//		g_lpBlobRender->applyFemDisplacements(dof, displacements);
	if(g_lpFastRBF)
		g_lpFastRBF->applyFemDisplacements(dof, displacements);
}

void Close()
{
	//Flush events to db and text log files
	TheDataBaseLogger::Instance().flush();
	PS::TheEventLogger::Instance().flush();

	//Cleanup
	cout << "Cleanup Memory objects" << endl;

	SAFE_DELETE(g_lpDrawNormals);
	SAFE_DELETE(g_lpSurface);
	SAFE_DELETE(g_lpFastRBF);
	SAFE_DELETE(g_lpDeformable);
	SAFE_DELETE(g_lpAvatarCube);
	SAFE_DELETE(g_lpGroundMatrix);
	SAFE_DELETE(g_lpSceneBox);
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
	cout << "GPU VENDOR: " << strVendorName << endl;
	cout << "GPU RENDERER: " << strRenderer << endl;
	cout << "GPU VERSION: " << strVersion << endl;
	
	LogInfoArg1("GPU VENDOR: %s", strVendorName.c_str());
	LogInfoArg1("GPU RENDERER: %s", strRenderer.c_str());
	LogInfoArg1("GPU VERSION: %s", strVersion.c_str());


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
		g_lpBlobRender->runPolygonizer(g_appSettings.cellsize);
		break;
	}
	case('-'):{
		g_appSettings.cellsize -= 0.01;
		LogInfoArg1("Changed cellsize to: %.2f", g_appSettings.cellsize);
		g_lpBlobRender->runPolygonizer(g_appSettings.cellsize);
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
	g_lpDeformable->timestep();
	g_appSettings.ctAnimFrame ++;

	//Log Database
	if(g_appSettings.bLogSql && (g_appSettings.ctAnimFrame - g_appSettings.ctAnimLogger > 5))
	{
		if(g_lpDeformable->isVolumeChanged())
		{
			DBLogger::Record rec;
			g_lpDeformable->statFillRecord(rec);
			TheDataBaseLogger::Instance().append(rec);
			g_appSettings.ctAnimLogger = g_appSettings.ctAnimFrame;
			g_appSettings.ctLogsCollected ++;
		}
	}

	glutTimerFunc(g_appSettings.timerInterval, TimeStep, 0);
}

bool LoadSettings(const std::string& strSimFP)
{
	if(!PS::FILESTRINGUTILS::FileExists(DAnsiStr(strSimFP.c_str()))) {
		LogErrorArg1("SIM file not found at: %s", strSimFP.c_str());
		return false;
	}


	LogInfo("Loading Settings from the ini file.");
	CSketchConfig cfg(DAnsiStr(strSimFP.c_str()), CSketchConfig::fmRead);

	//MODEL
	g_appSettings.strModelFilePath = (string)(cfg.readString("MODEL", "INPUTFILE").c_str());
	int ctFixed = 	cfg.readInt("MODEL", "FIXEDVERTICESCOUNT", 0);
	bool bres = cfg.readIntArray("MODEL", "FIXEDVERTICES", ctFixed, g_appSettings.vFixedVertices);
	if(!bres) LogError("Unable to read specified number of fixed vertices!");

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

	//Avatar
	TheUITransform::Instance().translate = cfg.readVec3f("AVATAR", "POS");
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

	//Avatar
	vec3d sides = g_lpAvatarCube->upper() - g_lpAvatarCube->lower();
	cfg.writeVec3f("AVATAR", "POS", TheUITransform::Instance().translate);
	cfg.writeVec3f("AVATAR", "THICKNESS", vec3f(sides.x, sides.y, sides.z));

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
				g_appSettings.strSimFilePath = string(ExtractFilePath(GetExePath()).c_str()) + string(argv[++iArg]);
				LogInfoArg1("Simulation filepath selected is %s", g_appSettings.strSimFilePath.c_str());
			}
			else if(strArg == "-h") {
				printf("Usage: FemBrain -f [SIM FILE] \n");
				printf("-f [SIM FILE] Input file defining the simulation scene.\n");
				exit(0);
			}

			iArg++;
		}
	}

	//Read Minc File
	//MincReader* lpMinc = new MincReader("/home/pourya/Desktop/Projects/FemBrain/AA_Models/mri/brainweb/minc/subject04_csf_v.mnc");
//	MincReader* lpMinc = new MincReader("/home/pourya/Desktop/Projects/FemBrain/AA_Models/mri/icbm/icbm_avg_152_t2_tal_lin.mnc");
//	SAFE_DELETE(lpMinc);


	//VegWriter::WriteVegFile("/home/pourya/Desktop/Models/pyramid/pyramid.1.node");
	//Setup the event logger
	PS::TheEventLogger::Instance().setWriteFlags(PS_LOG_WRITE_EVENTTYPE | PS_LOG_WRITE_TIMESTAMP | PS_LOG_WRITE_SOURCE | PS_LOG_WRITE_TO_SCREEN);
	LogInfo("Starting FemBrain");
	
	//Initialize app
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
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
	glutTimerFunc(g_appSettings.timerInterval, TimeStep, 0);

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
	TheShaderManager::Instance().add(g_lpVertexShaderCode, g_lpFragShaderCode, "phong");

	//Load Settings
	if(!LoadSettings(g_appSettings.strSimFilePath))
		exit(0);

	//Ground and Room
	g_lpGroundMatrix = new GroundMatrix(32, 32, 0.2);
	g_lpSceneBox = new SceneBox(vec3f(-10,-10,-16), vec3f(10,10,16));
	g_lpSceneBox->setShaderEffectProgram(TheShaderManager::Instance().get("phong"));

	//DAnsiStr strFPModel = ExtractFilePath(GetExePath());
	//strFPModel = ExtractOneLevelUp(strFPModel) + "AA_Models/flatdisk.scene";
	//strFPModel = ExtractOneLevelUp(strFPModel) + "AA_Models/testDisc3.scene";
	//strFPModel = ExtractOneLevelUp(strFPModel) + "AA_Models/CylinderWithHoles.scene";
	//strFPModel = ExtractOneLevelUp(strFPModel) + "AA_Models/peanut.scene";
	//Process BlobTree, Produce TetMesh
	DAnsiStr strFileExt = ExtractFileExt(DAnsiStr(g_appSettings.strModelFilePath.c_str()));

	//Mesh
	U32 ctVertices, ctElements;
	vector<U32> elements;
	vector<float> vertices;

	if(strFileExt == DAnsiStr("obj")) {
		Mesh* lpMesh = new Mesh(g_appSettings.strModelFilePath.c_str());
		lpMesh->fitToBBox(AABB(vec3f(0,0,0), vec3f(8, 8, 8)));
		g_lpFastRBF = new FastRBF(lpMesh);
	}
	else
	{
		GPUPoly* lpBlobRender = new GPUPoly();
		if(!lpBlobRender->readModel(g_appSettings.strModelFilePath.c_str())) {
			LogErrorArg1("Unable to read Blob file at: %s", g_appSettings.strModelFilePath.c_str());
			SAFE_DELETE(lpBlobRender);
			exit(-1);
		}
		lpBlobRender->runPolygonizer(g_appSettings.cellsize);
		g_lpFastRBF = new FastRBF(lpBlobRender);

		//Read Back Polygonized Mesh
		if(!lpBlobRender->readBackMesh(ctVertices, vertices, ctElements, elements))
			LogError("Unable to read mesh from RBF in the format of V3T3");
		PS::DEBUG::PrintArrayF(&vertices[0], 200);

		SAFE_DELETE(lpBlobRender);
	}

	//Prepares MeshBuffer for drawing normals
	g_lpDrawNormals = g_lpFastRBF->prepareMeshBufferNormals();


	{

		//Produce Quality Tetrahedral Mesh
		TetGenExporter::tesselate(ctVertices, &vertices[0], ctElements, &elements[0], "IsoSurfMeshIn", "TetMeshOut");

		//Export a veg file for the Fem Engine
		VegWriter::WriteVegFile("TetMeshOut.node");


		//Create Deformable Model from: Triangle Mesh and Tetrahedra Mesh
		DAnsiStr strVegFile = "TetMeshOut.veg";
		DAnsiStr strObjFile = "TetMeshOut.obj";

		//Obj mesh is useful though for viewing the boundary tetrahedra produced by tetgen
		//Produce special obj mesh with the same vertices and surface triangles
		VolumetricMesh * mesh = VolumetricMeshLoader::load(strVegFile.ptr());
		GenerateSurfaceMesh generateSurfaceMesh;
		ObjMesh * objMesh = generateSurfaceMesh.ComputeMesh(mesh, true);
		objMesh->save(string(strObjFile.cptr()));

		//Setup Deformable object
		g_lpDeformable = new Deformable(strVegFile.cptr(),
										strObjFile.cptr(),
										g_appSettings.vFixedVertices);

		//Deformations will be applied using opencl kernel
		g_lpDeformable->setDeformCallback(ApplyDeformations);
		g_lpDeformable->setHapticForceRadius(g_appSettings.hapticNeighborhoodPropagationRadius);
	}

	/*
	PS::HPC::RayTracer* lpTracer = new RayTracer(128, 128);
	lpTracer->run();
	SAFE_DELETE(lpTracer);
	 */

	//Surface
	/*
    glDisable(GL_TEXTURE_2D);
	g_lpSurface = new GLSurface(WINDOW_WIDTH, WINDOW_HEIGHT);
	g_lpSurface->attach();
    g_lpSurface->testDrawTriangle();
    g_lpSurface->detach();
    */
   // g_lpSurface->saveAsPPM("/home/pourya/Desktop/110.ppm");


	//Draw the first time
	glutPostRedisplay();

	//Run App
	glutMainLoop();

	return 0;
}

