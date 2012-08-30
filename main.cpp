#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include "PS_MathBase.h"
#include "PS_Logger.h"
#include <string>
#include <fstream>
#include "PS_OclPolygonizer.h"
#include "PS_Particles.h"
#include "PS_ArcBallCamera.h"
#include "PS_Base/PS_FileDirectory.h"
#include "PS_BlobTreeRender/PS_OclPolygonizer.h"
#include "PS_Graphics/PS_GLFuncs.h"

using namespace std;
using namespace PS;
using namespace PS::FILESTRINGUTILS;
using namespace PS::HPC;

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768
#define FOVY 60.0
#define ZNEAR 0.1
#define ZFAR 3000

class AppSettings{
public:
	AppSettings(){
		this->bPanCamera = false;
		this->pan = svec2f(0.0f, 0.0f); 
	}

public:
	bool bPanCamera;
	svec2f pan;
};

//Global Variables
PS::CArcBallCamera g_arcBallCam;
PS::HPC::Particles* g_lpParticles = NULL;
PS::HPC::GPUPoly* g_lpBlobRender = NULL;
AppSettings g_appSettings;
GLuint g_uiShader;

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
void Display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Render
	glTranslatef(g_appSettings.pan.x, g_appSettings.pan.y, 0.0f);
	svec3f p = g_arcBallCam.getCoordinates();
	svec3f c = g_arcBallCam.getCenter();
	gluLookAt(p.x, p.y, p.z, c.x, c.y, c.z, 0.0f, 1.0f, 0.0f);
	
	//Use Shader Effect
	glUseProgram(g_uiShader);
	if(g_lpBlobRender)
	{
		g_lpBlobRender->drawBBox();
		g_lpBlobRender->drawMesh(false);
	}

	glUseProgram(0);

	glutSwapBuffers();
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
	if(state == GLUT_DOWN)
		g_arcBallCam.mousePress((PS::CArcBallCamera::MOUSEBUTTONSTATE)button, x, y);
	else
		g_arcBallCam.mousePress(PS::CArcBallCamera::mbMiddle, x, y);
}

void MouseMove(int x, int y)
{
	if(g_appSettings.bPanCamera)
	{
		g_appSettings.pan.x += (x - g_arcBallCam.getLastPos().x) * 0.03f;
		g_appSettings.pan.y += (g_arcBallCam.getLastPos().y - y) * 0.03f;
		g_arcBallCam.setLastPos(svec2i(x, y));
	}
	else
		g_arcBallCam.mouseMove(x, y);
	glutPostRedisplay();
}

void Close()
{
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

bool GetGPUInfo()
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
	return true;
}


//Main Loop of Application
int main(int argc, char* argv[])
{
	//Setup the event logger
	PS::TheEventLogger::Instance().setWriteFlags(PS_LOG_WRITE_EVENTTYPE | PS_LOG_WRITE_TIMESTAMP | PS_LOG_WRITE_SOURCE);
	
	//Initialize app
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("FEM Hydrocephalus Surgery Simulation");
	glutDisplayFunc(Display);
	glutReshapeFunc(Resize);
	glutMouseFunc(MousePress);
	glutMotionFunc(MouseMove);
///	glutSpecialFunc(Keyboard);
	glutCloseFunc(Close);

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
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glClearColor(0.45f, 0.45f, 0.45f, 1.0f);

	//Compiling shaders
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		//Problem: glewInit failed, something is seriously wrong.
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(1);
	}
	CompileShaderCode(g_lpVertexShaderCode, g_lpFragShaderCode, g_uiShader);
	


	//Run OCL TEST
	//PS::HPC::Run_SphereDistKernel();
	//g_lpParticles = new ::Particles(1024);
	DAnsiStr strFPModel;
	{
		strFPModel = ExtractOneLevelUp(ExtractFilePath(GetExePath()));
		strFPModel += "AA_Models/sphere.txt";
	}

	g_lpBlobRender = new GPUPoly();
	g_lpBlobRender->readModel(strFPModel.cptr());
	g_lpBlobRender->runTandem(0.1);
	glutPostRedisplay();

	

	//Run App
	glutMainLoop();

	return 0;
}
