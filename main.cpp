#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>

#include <string>
#include <vector>
#include <fstream>

#include "PS_Base/PS_MathBase.h"
#include "PS_Base/PS_Logger.h"
#include "PS_Base/PS_FileDirectory.h"
#include "PS_BlobTreeRender/PS_OclPolygonizer.h"
#include "PS_Graphics/PS_ArcBallCamera.h"
#include "PS_Graphics/PS_GLFuncs.h"
#include "PS_Graphics/PS_GLSurface.h"
#include "PS_Graphics/PS_SketchConfig.h"
#include "PS_Deformable/PS_Deformable.h"
#include "PS_Deformable/PS_VegWriter.h"

using namespace std;
using namespace PS;
using namespace PS::FILESTRINGUTILS;
using namespace PS::HPC;

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define FOVY 45.0
#define ZNEAR 0.01
#define ZFAR 100.0
#define DEFAULT_TIMER_MILLIS 33

#define INDEX_CAMERA_INFO 0
#define INDEX_HAPTIC_INFO 1
#define INDEX_GPU_INFO 	   2


//Application Settings
class AppSettings{
public:

	//Set to default values in constructor
	AppSettings(){
		this->bDrawWireFrame = false;
		this->bPanCamera = false;
		this->bShowElements = false;
		this->millis = DEFAULT_TIMER_MILLIS;
		this->axis = 0;
	}

public:
	bool bPanCamera;
	bool bDrawWireFrame;
	bool bShowElements;
	U32   millis;
	int appWidth;
	int appHeight;

	int axis;
	svec3f worldDragStart;
	svec3f worldDragEnd;
	svec2i screenDragStart;
	svec2i screenDragEnd;
};

//Global Variables
PS::ArcBallCamera g_arcBallCam;
PS::HPC::GPUPoly* g_lpBlobRender = NULL;
Deformable* g_lpDeformable = NULL;

//Info Lines
std::vector<std::string> g_infoLines;

GLSurface* g_lpSurface = NULL;
AppSettings g_appSettings;
GLuint g_uiShader;


////////////////////////////////////////////////
//Function Prototype
void Draw();
void Resize(int w, int h);
void TimeStep(int t);
void MousePress(int button, int state, int x, int y);
void MouseMove(int x, int y);
void MouseWheel(int button, int dir, int x, int y);
bool ScreenToWorld(const svec3f& screenP, svec3f& worldP);

void Keyboard(int key, int x, int y);
void DrawText(const char* chrText, int x, int y);
string GetGPUInfo();

//Settings
void LoadSettings();
void SaveSettings();
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

	//glTranslatef(0,0,10);
	/*
	glTranslatef(g_appSettings.pan.x, g_appSettings.pan.y, 0.0f);
	svec3f p = g_arcBallCam.getCoordinates();
	svec3f c = g_arcBallCam.getCenter();
	gluLookAt(p.x, p.y, p.z, c.x, c.y, c.z, 0.0f, 1.0f, 0.0f);
	*/
	//Use Shader Effect
	/*
	glUseProgram(g_uiShader);
	if(g_lpBlobRender)
	{
		g_lpBlobRender->drawBBox();
		g_lpBlobRender->drawMesh(g_appSettings.bDrawWireFrame);
	}
	glUseProgram(0);
	*/

	//Draw Deformable Object
	g_lpDeformable->draw();

	//Draw Haptic Line
	if(g_lpDeformable->isHapticInProgress())
	{
		GLint vp[4];
		glGetIntegerv(GL_VIEWPORT, vp);
		//svec3f s1 = g_appSettings.worldDragStart;
		//svec3f s2 = g_appSettings.worldDragEnd;
		svec2i s1 = g_appSettings.screenDragStart;
		svec2i s2 = g_appSettings.screenDragEnd;

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

	//Write Camera Info
	{
		char chrMsg[1024];
		sprintf(chrMsg,"Camera [Roll=%.1f, Tilt=%.1f, PanX=%.2f, PanY=%.2f]",
				g_arcBallCam.getRoll(),
				g_arcBallCam.getTilt(),
				g_arcBallCam.getPan().x,
				g_arcBallCam.getPan().y);
		g_infoLines[INDEX_CAMERA_INFO] = string(chrMsg);
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
	if(button == GLUT_RIGHT_BUTTON)
	{
		if(state == GLUT_DOWN)
		{
			GLdouble model[16];
			glGetDoublev (GL_MODELVIEW_MATRIX, model);

			GLdouble proj[16];
			glGetDoublev (GL_PROJECTION_MATRIX, proj);

			GLint view[4];
			glGetIntegerv (GL_VIEWPORT, view);

			int winX = x;
			int winY = view[3] - 1 - y;

			float zValue;
			glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &zValue);

			GLubyte stencilValue;
			glReadPixels(winX, winY, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, &stencilValue);

			GLdouble worldX, worldY, worldZ;
			gluUnProject(winX, winY, zValue, model, proj, view,
						   &worldX, &worldY, &worldZ);

			if (stencilValue == 1) {
				if(g_lpDeformable->pickFreeVertex(worldX, worldY, worldZ))
				{
					g_appSettings.worldDragStart = svec3f(worldX, worldY, worldZ);
					g_appSettings.screenDragStart = svec2i(x, y);
					g_lpDeformable->hapticStart();
				}
			}
		}
		else
			g_lpDeformable->hapticEnd();
	}

	//Camera
	g_arcBallCam.mousePress(button, state, x, y);

	//Update selection
	glutPostRedisplay();
}

void MouseMove(int x, int y)
{
	if(g_lpDeformable->isHapticInProgress())
	{
		svec3f ptScreen(x - g_appSettings.screenDragStart.x,
						y - g_appSettings.screenDragStart.y, 0);
		svec3f ptWorld;

		string strAxis = "X";
		if(g_appSettings.axis == 0)
			strAxis = "X";
		else if(g_appSettings.axis == 1)
			strAxis = "Y";
		else
			strAxis = "Z";

		char buffer[1024];
		sprintf(buffer, "HAPTIC LEN=%.2f, AXIS=%s", vlength3f(ptScreen), strAxis.c_str());
		g_infoLines[INDEX_HAPTIC_INFO] = string(buffer);

		//ArcBall Camera
		g_arcBallCam.screenToWorld_OrientationOnly3D(ptScreen, ptWorld);

		double arrExt[3];
		arrExt[0] = ptWorld.x;
		arrExt[1] = ptWorld.y;
		arrExt[2] = ptWorld.z;

		g_lpDeformable->hapticSetCurrentForce(arrExt);
		g_appSettings.worldDragEnd = ptWorld;
		g_appSettings.screenDragEnd = svec2i(x, y);
	}

	g_arcBallCam.mouseMove(x, y);
	glutPostRedisplay();
}

void MouseWheel(int button, int dir, int x, int y)
{
	g_arcBallCam.mouseWheel(button, dir, x, y);
	glutPostRedisplay();
}

void Close()
{
	//Cleanup
	cout << "Cleanup Memory objects" << endl;
	SAFE_DELETE(g_lpSurface);
	SAFE_DELETE(g_lpBlobRender);
	SAFE_DELETE(g_lpDeformable);

	PS::TheEventLogger::Instance().flush();
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

bool ScreenToWorld(const svec3f& screenP, svec3f& worldP)
{
    GLdouble ox, oy, oz;
    GLdouble mv[16];
    GLdouble pr[16];
    GLint vp[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, mv);
    glGetDoublev(GL_PROJECTION_MATRIX, pr);
    glGetIntegerv(GL_VIEWPORT, vp);
    if(gluUnProject(screenP.x, screenP.y, screenP.z, mv, pr, vp, &ox, &oy, &oz) == GL_TRUE)
    {
        worldP = svec3f(ox, oy, oz);
        return true;
    }

    return false;
}

void Keyboard(int key, int x, int y)
{
	switch(key)
	{
		case(GLUT_KEY_F2):
		{
			g_appSettings.bDrawWireFrame = !g_appSettings.bDrawWireFrame;
			LogInfoArg1("Draw wireframe mode: %d", g_appSettings.bDrawWireFrame);
			break;
		}

		case(GLUT_KEY_F3):
		{

			break;
		}

		case(GLUT_KEY_F4):
		{
			g_appSettings.axis = (g_appSettings.axis + 1) % 3;
			LogInfoArg1("Change haptic axis to %d", g_appSettings.axis);
			break;
		}

		case(GLUT_KEY_F11):
		{
			//Saving Settings and Exit
			SaveSettings();

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
	glutTimerFunc(g_appSettings.millis, TimeStep, 0);
}

void LoadSettings()
{
	LogInfo("Loading Settings from the ini file.");

	CSketchConfig cfg(ChangeFileExt(GetExePath(), ".ini"), CSketchConfig::fmRead);
	DAnsiStr strVegFile = cfg.readString("MODEL", "VEGFILE", "");
	DAnsiStr strObjFile = cfg.readString("MODEL", "OBJFILE", "");
	DAnsiStr strBlobFile = cfg.readString("MODEL", "BLOBTREEFILE", "");
	int ctFixed = 	cfg.readInt("MODEL", "FIXEDVERTICESCOUNT", 0);
	vector<int> vFixedVertices;
	bool bres = cfg.readIntArray("MODEL", "FIXEDVERTICES", ctFixed, vFixedVertices);
	if(!bres)
		LogError("Unable to read specified number of fixed vertices!");
	DAnsiStr strBlobTreeFile = cfg.readString("MODEL", "BLOBTREEFILE", "");

	//Create Deformable Model
	g_lpDeformable = new Deformable(strVegFile.cptr(),
									strObjFile.cptr(),
									vFixedVertices);

	/*
	g_lpBlobRender = new GPUPoly();
	g_lpBlobRender->readModel(strFPModel.cptr());
	g_lpBlobRender->runTandem(0.1);
	*/

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
	g_infoLines.push_back(string("CAMERA"));
	g_infoLines.push_back(string("HAPTIC"));
	g_infoLines.push_back(GetGPUInfo());

}

void SaveSettings()
{
	LogInfo("Saving settings to ini file");
	CSketchConfig cfg(ChangeFileExt(GetExePath(), ".ini"), CSketchConfig::fmReadWrite);
	cfg.writeFloat("CAMERA", "ROLL", g_arcBallCam.getRoll());
	cfg.writeFloat("CAMERA", "TILT", g_arcBallCam.getTilt());
	cfg.writeFloat("CAMERA", "ZOOM", g_arcBallCam.getCurrentZoom());
	cfg.writeVec3f("CAMERA", "CENTER", g_arcBallCam.getCenter());
	cfg.writeVec3f("CAMERA", "ORIGIN", g_arcBallCam.getOrigin());
	cfg.writeVec2f("CAMERA", "PAN", g_arcBallCam.getPan());
}


//Main Loop of Application
int main(int argc, char* argv[])
{
	//VegWriter::WriteVegFile("/home/pourya/Desktop/disc/disc.1.node");

	//Setup the event logger
	PS::TheEventLogger::Instance().setWriteFlags(PS_LOG_WRITE_EVENTTYPE | PS_LOG_WRITE_TIMESTAMP | PS_LOG_WRITE_SOURCE | PS_LOG_WRITE_TO_SCREEN);
	LogInfo("Starting FemMain Application");
	
	//Initialize app
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("Deformable Tissue Modeling - PhD Project - Pourya Shirazian");
	glutDisplayFunc(Draw);
	glutReshapeFunc(Resize);
	glutMouseFunc(MousePress);
	glutMotionFunc(MouseMove);
	glutMouseWheelFunc(MouseWheel);

	glutSpecialFunc(Keyboard);
	glutCloseFunc(Close);
	glutTimerFunc(g_appSettings.millis, TimeStep, 0);

	//Print GPU INFO
	GetGPUInfo();

	//Setup Shading Environment
	static const GLfloat lightColor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	static const GLfloat lightPos[4] = { 5.0f, 5.0f, 10.0f, 0.0f };

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
	CompileShaderCode(g_lpVertexShaderCode, g_lpFragShaderCode, g_uiShader);

	//Load Settings
	LoadSettings();

	//Surface
	/*
    glDisable(GL_TEXTURE_2D);
	g_lpSurface = new GLSurface(WINDOW_WIDTH, WINDOW_HEIGHT);
	g_lpSurface->attach();
    g_lpSurface->testDrawTriangle();
    g_lpSurface->detach();
    */
   // g_lpSurface->saveAsPPM("/home/pourya/Desktop/110.ppm");


	glutPostRedisplay();

	

	//Run App
	glutMainLoop();

	return 0;
}
