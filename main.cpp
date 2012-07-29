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

using namespace std;
using namespace PS;
using namespace PS::FILESTRINGUTILS;
using namespace PS::HPC;

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

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


void Display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.4, 0.4, 0.4, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//Render
	/*
	glTranslatef(g_appSettings.pan.x, g_appSettings.pan.y, 0.0f);
	svec3f p = g_arcBallCam.getCoordinates();
	svec3f c = g_arcBallCam.getCenter();
	gluLookAt(p.x, p.y, p.z, c.x, c.y, c.z, 0.0f, 1.0f, 0.0f);
	*/
	if(g_lpBlobRender)
		g_lpBlobRender->drawMesh();

	glutSwapBuffers();
}

void Resize(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//gluPerspective(60.0f, (double)w/(double)h, 0.1, 3000.0);
	glOrtho(-2.0f, 2.0f, -2.0f, 2.0f, -2.0f, 2.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void MousePress(int button, int state, int x, int y)
{
	/*
	if(state == GLUT_DOWN)
		g_arcBallCam.mousePress((PS::CArcBallCamera::MOUSEBUTTONSTATE)button, x, y);
	else
		g_arcBallCam.mousePress(PS::CArcBallCamera::mbMiddle, x, y);
		*/
}

void MouseMove(int x, int y)
{
	/*
	if(g_appSettings.bPanCamera)
	{
		g_appSettings.pan.x += (x - g_arcBallCam.getLastPos().x) * 0.03f;
		g_appSettings.pan.y += (g_arcBallCam.getLastPos().y - y) * 0.03f;
		g_arcBallCam.setLastPos(svec2i(x, y));
	}
	else
		g_arcBallCam.mouseMove(x, y);
	glutPostRedisplay();
	*/
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

	//Compiling shaders
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		//Problem: glewInit failed, something is seriously wrong.
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(1);
	}

	//Print GPU INFO
	GetGPUInfo();

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
	g_lpBlobRender->run();


	

	//Run App
	glutMainLoop();

	return 0;
}
