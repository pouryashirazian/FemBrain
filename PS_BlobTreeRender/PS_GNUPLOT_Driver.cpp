/*
 * PS_GNUPLOT_Driver.cpp
 *
 *  Created on: 2011-10-18
 *      Author: pourya
 */

#include <string.h>
#include <limits.h>
#include "PS_GNUPLOT_Driver.h"
#include "../PS_Base/PS_FileDirectory.h"
using namespace PS;
using namespace PS::FILESTRINGUTILS;


GNUPlotDriver::GNUPlotDriver(const string& strDB, int iStartXP, int iEndXP)
{
	m_strSQLDB = strDB;
	m_minX       = INT_MAX;
	m_maxX       = INT_MIN;
	m_minY 		 = FLT_MAX;
	m_maxY       = FLT_MIN;

	m_idxStart   = iStartXP;
	m_idxEnd     = iEndXP;
	m_strLabelX  = "Number of cores";
	m_strLabelY  = "Polygonization time [ms]";
	m_strTitle   = "Rendering performance graph - Polygonization time vs. Number of cores";
	m_strBGColor = "white";
	m_strDataColor = "blue";
	m_lpGNUPlot = NULL;

	m_bRaise = true;
	m_bPersist = true;
}

GNUPlotDriver::~GNUPlotDriver()
{
	if(m_lpGNUPlot)
	{
		fclose(m_lpGNUPlot);
	}

}

int GNUPlotDriver::sqlite_Callback(void* lpDriver, int argc, char **argv, char **azColName)
{
	int ctThreads = 0;
	float cellSize = 0;
	float arrTimes[3];
	arrTimes[0] = 0.0;
	arrTimes[1] = 0.0;
	arrTimes[2] = 0.0;

	for(int i=0; i<argc; i++)
	{
		printf("%s = %s\t", azColName[i], argv[i] ? argv[i] : "NULL");
		if(strcmp(azColName[i], "ctThreads") == 0)
			ctThreads = atoi(argv[i]);
		else if(strcmp(azColName[i], "cellSize") == 0)
			cellSize = atof(argv[i]);
		else if(strcmp(azColName[i], "msPolyTotal") == 0)
			arrTimes[0] = atof(argv[i]);
		else if(strcmp(azColName[i], "msPolyFieldEvals") == 0)
			arrTimes[1] = atof(argv[i]);
		else if(strcmp(azColName[i], "msPolyTriangulate") == 0)
			arrTimes[2] = atof(argv[i]);

	}
	printf("\n");

	if(lpDriver)
		reinterpret_cast<GNUPlotDriver*>(lpDriver)->addDataPoint(ctThreads, cellSize, arrTimes);
	return 0;
}

void GNUPlotDriver::addDataPoint(int x, float y, float z)
{
	m_minX = (x < m_minX)? x : m_minX;
	m_maxX = (x > m_maxX)? x : m_maxX;

	m_minY = (y < m_minY)? y : m_minY;
	m_maxY = (y > m_maxX)? y : m_maxY;

	m_strData << x << "," << y << "," << z << endl;
}

void GNUPlotDriver::addDataPoint(int ctThreads, float cellSize, float arrTimes[3])
{
	m_strData << ctThreads << "," << cellSize << "," << arrTimes[0] << "," << arrTimes[1] << "," << arrTimes[2] << endl;
}

bool GNUPlotDriver::createGraph()
{
	sqlite3* db;
	char* lpSqlError = 0;

	int rc = sqlite3_open(m_strSQLDB.c_str(), &db);
	if(rc)
	{
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
		sqlite3_close(db);
		return false;
	}
/*
		const char *strSQL = "INSERT INTO tblPerfLog (xpModelName, xpTime, ctPrims, ctOps, cellSize, ctGroupsTotal, ctMPUTotal, ctMPUIntersected, ctTotalFieldEvals, "
						"ctLatestMPUFieldEvals, ctFVEPT, ctMeshFaces, ctMeshVertices, msPolyTotal, msPolyFieldEvals, msPolyTriangulate, "
						"ctThreads, ctSIMDLength, szWorkItemMem, szTotalMemUsage, szLastLevelCache) values"
						"(@xp_model_name, @xp_time, @ct_prims, @ct_ops, @cell_size, @ct_groups_total, @ct_mpu_total, @ct_mpu_intersected, @ct_total_fieldevals, "
						"@ct_lastestmpu_fieldevals, @ct_fvept, @ct_mesh_faces, @ct_mesh_vertices, @time_total, @time_fieldevals, $time_triangulate, "
						"@ct_threads, @ct_simd_length, @sz_work_item_mem, @sz_total_mem_usage, @sz_last_level_cache);";

*/
	char buffer[1024];

	sprintf(buffer, "SELECT xpID, msPolyTotal, msPolyFieldEvals, msPolyTriangulate, ctThreads, ctSIMDLength, cellSize FROM tblPerfLog WHERE xpID >= %d and xpID <= %d", m_idxStart, m_idxEnd);

	rc = sqlite3_exec(db, buffer, sqlite_Callback, this, &lpSqlError);
	if(rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL error: %s\n", lpSqlError);
		sqlite3_free(lpSqlError);
		return false;
	}

	string strDatFN;
	{
		ofstream myfile;
		GetExePath(buffer, 1024);
		strcat(buffer, "Log.dat");

		strDatFN = string(buffer);
		myfile.open (buffer);
		myfile << m_strData.str() << endl;
		myfile.close();
	}

#ifdef PS_OS_LINUX
    try{
		if(!m_lpGNUPlot)
		{
		  string com = (string) "gnuplot ";
		  m_lpGNUPlot = popen(com.c_str(),"w");
		}


		sprintf(buffer, "plot \"ParsipCmdLinuxLog.dat\" using 1:2", strDatFN.c_str());
		fprintf(m_lpGNUPlot, "%s", buffer);
    }

	catch (ios::failure const &problem)
	{
		printf("gnuplot_driver: %s\n", problem.what());
	}
#endif

	return true;

}
