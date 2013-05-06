/*
 * DBLogger.cpp
 *
 *  Created on: Nov 19, 2012
 *      Author: pourya
 */

#include "DBLogger.h"

#include "sqlite/CppSQLite3.h"
#include "../PS_Base/PS_Logger.h"
#include "../PS_Base/PS_FileDirectory.h"

using namespace PS;
using namespace PS::FILESTRINGUTILS;

//Global result to notify caller of the result of running the task
tbb::concurrent_queue<TaskLogInsertResult> g_resultQ;

DBLogger::DBLogger()
{
	m_isTaskInProgress = false;
	DAnsiStr strFP = ExtractFilePath(GetExePath()) + "fembrain_logs.db";
	m_strDB = string(strFP.c_str());

	this->createTable();
	m_lastXPID = getLastXPID();
}

DBLogger::~DBLogger()
{
	this->flush();
}

bool DBLogger::insert(const Record& record)
{
	sqlite3* lpDB = NULL;
	int rc = sqlite3_open(m_strDB.c_str(), &lpDB);
	if(rc)
	{
		LogErrorArg1("Can't open database: %s", sqlite3_errmsg(lpDB));
		sqlite3_close(lpDB);
		lpDB = NULL;
	}

	if(!lpDB)
		return false;
	char* lpSqlError = 0;
	const char *strSQL = "INSERT INTO tblPerfLog (xpID, xpModelName, xpTime, xpForceModel, xpIntegrator, totalVolume, restVolume, ctVertices, ctElements, elementType, poissonRatio, youngModulo, "
						  "ctSolverThreads, ctAnimFPS, msAnimTotalFrame, msAnimSysSolver, msAnimApplyDisplacements, msPolyTriangleMesh, msPolyTetrahedraMesh, msRBFCreation, msRBFEvaluation) values"
						"(@xp_id, @xp_model_name, @xp_time, @xp_force_model, @xp_integrator, @total_volume, @rest_volume, @ct_vertices, @ct_elements, @element_type, @poisson_ratio, @young_modulo, "
						"@solver_threads, @anim_fps, @ms_anim_totalframe, @ms_anim_syssolver, @ms_anim_applydisplacements, @ms_poly_triangletime, @ms_poly_tetrahedratime, @ms_rbf_creation, "
						"@ms_rbf_evaluation);";
	sqlite3_stmt* statement;
	sqlite3_prepare_v2(lpDB, strSQL, -1, &statement, NULL);

	int idxParam = sqlite3_bind_parameter_index(statement, "@xp_id");
	sqlite3_bind_int(statement, idxParam, getLastXPID() + 1);

	//Bind All Parameters for Insert Statement
	idxParam = sqlite3_bind_parameter_index(statement, "@xp_model_name");
	sqlite3_bind_text(statement, idxParam, record.xpModelName.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@xp_time");
	sqlite3_bind_text(statement, idxParam, record.xpTime.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@xp_force_model");
	sqlite3_bind_text(statement, idxParam, record.xpForceModel.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@xp_integrator");
	sqlite3_bind_text(statement, idxParam, record.xpIntegrator.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@total_volume");
	sqlite3_bind_double(statement, idxParam, record.totalVolume);

	idxParam = sqlite3_bind_parameter_index(statement, "@rest_volume");
	sqlite3_bind_double(statement, idxParam, record.restVolume);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_vertices");
	sqlite3_bind_int(statement, idxParam, record.ctVertices);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_elements");
	sqlite3_bind_int(statement, idxParam, record.ctElements);

	idxParam = sqlite3_bind_parameter_index(statement, "@element_type");
	sqlite3_bind_text(statement, idxParam, record.xpElementType.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@poisson_ratio");
	sqlite3_bind_double(statement, idxParam, record.poissonRatio);

	idxParam = sqlite3_bind_parameter_index(statement, "@young_modulo");
	sqlite3_bind_double(statement, idxParam, record.youngModulo);

	//Write statistics
	idxParam = sqlite3_bind_parameter_index(statement, "@solver_threads");
	sqlite3_bind_double(statement, idxParam, record.ctSolverThreads);

	idxParam = sqlite3_bind_parameter_index(statement, "@anim_fps");
	sqlite3_bind_double(statement, idxParam, record.animFPS);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_anim_totalframe");
	sqlite3_bind_double(statement, idxParam, record.msAnimTotalFrame);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_anim_syssolver");
	sqlite3_bind_double(statement, idxParam, record.msAnimSysSolver);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_anim_applydisplacements");
	sqlite3_bind_double(statement, idxParam, record.msAnimApplyDisplacements);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_poly_triangletime");
	sqlite3_bind_double(statement, idxParam, record.msPolyTriangleMesh);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_poly_tetrahedratime");
	sqlite3_bind_double(statement, idxParam, record.msPolyTetrahedraMesh);

	//ms_rbf_creation, @ms_rbf_evaluation
	idxParam = sqlite3_bind_parameter_index(statement, "@ms_rbf_creation");
	sqlite3_bind_double(statement, idxParam, record.msRBFCreation);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_rbf_evaluation");
	sqlite3_bind_double(statement, idxParam, record.msRBFEvaluation);

	rc = sqlite3_step(statement);
	if(rc != SQLITE_DONE)
	{
		LogErrorArg1("SQL error: %s", sqlite3_errmsg(lpDB));
		sqlite3_free(lpSqlError);
	}

	//Free Statement
	sqlite3_finalize(statement);

	//Close DB
	sqlite3_close(lpDB);

	return true;
}

int DBLogger::getLastXPID() const
{
	CppSQLite3DB db;
	db.open(m_strDB.c_str());

	CppSQLite3Query q = db.execQuery("select MAX(xpID) from tblPerfLog");
	while(!q.eof())
	{
		if(strcmp(q.fieldName(0), "MAX(xpID)") == 0)
			return q.getIntField(0, -1);
		q.nextRow();
	}

	return 0;
}


int sqlite_Callback(void *NotUsed, int argc, char **argv, char **azColName)
{
	int i;
	for(i=0; i<argc; i++){
      printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
	}
	printf("\n");
	return 0;
}



bool DBLogger::isLogTableExist() const
{
	sqlite3* db;
	char* lpSqlError = 0;
	int rc = sqlite3_open(m_strDB.c_str(), &db);
	if(rc)
	{
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
		sqlite3_close(db);
		return false;
	}

	const char *strSQL = "SELECT CASE WHEN tbl_name = 'tblPerfLog' THEN 1 ELSE 0 END FROM sqlite_master WHERE tbl_name = 'tblPerfLog' AND type = 'table'";
	rc = sqlite3_exec(db, strSQL, sqlite_Callback, 0, &lpSqlError);
	if(rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL error: %s\n", lpSqlError);
		sqlite3_free(lpSqlError);
		return false;
	}
	return true;
}

bool DBLogger::createTable()
{
	sqlite3* db;
	char* lpSqlError = 0;
	int rc = sqlite3_open(m_strDB.c_str(), &db);
	if(rc)
	{
		LogErrorArg1("Can't open database: %s", sqlite3_errmsg(db));
		sqlite3_close(db);
		return false;
	}

	const char *strSQL1 = "CREATE TABLE IF NOT EXISTS tblPerfLog(xpID INTEGER PRIMARY KEY AUTOINCREMENT, xpModelName VARCHAR(30), xpTime DATETIME, "
						 "xpForceModel VARCHAR(128), xpIntegrator VARCHAR(30), totalVolume double, restVolume double, ctVertices int, ctElements int, "
						 "elementType VARCHAR(16), poissonRatio double, youngModulo double, ctSolverThreads int, ctAnimFPS int, msAnimTotalFrame double, "
						 "msAnimSysSolver double, msAnimApplyDisplacements double, msPolyTriangleMesh double, msPolyTetrahedraMesh double, "
						 "msRBFCreation double, msRBFEvaluation double);";
	rc = sqlite3_exec(db, strSQL1, sqlite_Callback, 0, &lpSqlError);
	if(rc != SQLITE_OK)
	{
		LogErrorArg1("SQL error: %s\n", lpSqlError);
		sqlite3_free(lpSqlError);
		return false;
	}

	return true;
}

void DBLogger::flush()
{
	if(m_vRecords.size() == 0)
		return;

	m_isTaskInProgress = true;
	Functor<void> FOnCompleted(this, &DBLogger::insertCompleted);
	DBInsertionTask* t = new( tbb::task::allocate_root()) DBInsertionTask(m_strDB, m_vRecords, FOnCompleted);
	tbb::task::enqueue(*t);

	m_vRecords.resize(0);
}

void DBLogger::flushAndWait() {
	flush();
	while(m_isTaskInProgress) {
		//LogInfo("Waiting for db flush thread to finish!");
		sleep(1);
	}
}

void DBLogger::insertCompleted() {
	TaskLogInsertResult result;
	g_resultQ.try_pop(result);
	LogInfoArg1("Insertion task completed! Written records: %d\n", result.ctWrittenRecords);
	m_isTaskInProgress = false;
}

string DBLogger::timestamp()
{
	const int TIME_STRING_LENGTH = 20;
	char chrTimeStamp[TIME_STRING_LENGTH];
	{
		time_t rawtime;
		struct tm* currentTime;
		time(&rawtime);
		currentTime = localtime(&rawtime);
		strftime(chrTimeStamp, TIME_STRING_LENGTH, "%Y-%m-%d %H:%M:%S", currentTime);
	}
	return string(chrTimeStamp);
}
////////////////////////////////////////////////////////////////////////////////////////
DBInsertionTask::DBInsertionTask(const string& strDB,
		  const std::vector<DBLogger::Record>& vRecords,
		  const FOnInsertionCompleted& fOnCompleted) {

	m_strDB = strDB;
	m_vRecords.assign(vRecords.begin(), vRecords.end());
	m_fOnCompleted = fOnCompleted;
}


/*!
 * Perform all record insertions
 */
tbb::task* DBInsertionTask::execute() {

	TaskLogInsertResult result;
	result.ctWrittenRecords = 0;

	//Open DB
	//printf("DBPATH: %s\n", m_strDB.c_str());

	sqlite3* lpDB = NULL;
	int rc = sqlite3_open(m_strDB.c_str(), &lpDB);
	if(rc)
	{
		printf("Can't open database: %s", sqlite3_errmsg(lpDB));
		sqlite3_close(lpDB);
		lpDB = NULL;
	}

	if(!lpDB) {
		g_resultQ.push(result);
		return NULL;
	}

	int ctInserted = 0;

	//Insert all records to db
	for(U32 i=0; i<m_vRecords.size(); i++) {
		if(insertRecord(lpDB, m_vRecords[i]))
			ctInserted++;
	}

	//Close DB
	sqlite3_close(lpDB);

	//Inform GUI Thread
	result.ctWrittenRecords = ctInserted;
	g_resultQ.push(result);
	if(m_fOnCompleted)
		m_fOnCompleted();

	return NULL;
}

bool DBInsertionTask::insertRecord(sqlite3* lpDB,
		const DBLogger::Record& record) {

	printf("WRITING: %s\n", record.xpTime.c_str());
	char* lpSqlError = 0;

	const char *strSQL = "INSERT INTO tblPerfLog (xpID, xpModelName, xpTime, xpForceModel, xpIntegrator, totalVolume, restVolume, ctVertices, ctElements, elementType, poissonRatio, youngModulo, "
						  "ctSolverThreads, ctAnimFPS, msAnimTotalFrame, msAnimSysSolver, msAnimApplyDisplacements, msPolyTriangleMesh, msPolyTetrahedraMesh, msRBFCreation, msRBFEvaluation) values"
						"(@xp_id, @xp_model_name, @xp_time, @xp_force_model, @xp_integrator, @total_volume, @rest_volume, @ct_vertices, @ct_elements, @element_type, @poisson_ratio, @young_modulo, "
						"@solver_threads, @anim_fps, @ms_anim_totalframe, @ms_anim_syssolver, @ms_anim_applydisplacements, @ms_poly_triangletime, @ms_poly_tetrahedratime, @ms_rbf_creation, "
						"@ms_rbf_evaluation);";
	sqlite3_stmt* statement;
	sqlite3_prepare_v2(lpDB, strSQL, -1, &statement, NULL);

	int idxParam = sqlite3_bind_parameter_index(statement, "@xp_id");
	sqlite3_bind_int(statement, idxParam, record.xpID + 1);

	//Bind All Parameters for Insert Statement
	idxParam = sqlite3_bind_parameter_index(statement, "@xp_model_name");
	sqlite3_bind_text(statement, idxParam, record.xpModelName.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@xp_time");
	sqlite3_bind_text(statement, idxParam, record.xpTime.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@xp_force_model");
	sqlite3_bind_text(statement, idxParam, record.xpForceModel.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@xp_integrator");
	sqlite3_bind_text(statement, idxParam, record.xpIntegrator.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@total_volume");
	sqlite3_bind_double(statement, idxParam, record.totalVolume);

	idxParam = sqlite3_bind_parameter_index(statement, "@rest_volume");
	sqlite3_bind_double(statement, idxParam, record.restVolume);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_vertices");
	sqlite3_bind_int(statement, idxParam, record.ctVertices);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_elements");
	sqlite3_bind_int(statement, idxParam, record.ctElements);

	idxParam = sqlite3_bind_parameter_index(statement, "@element_type");
	sqlite3_bind_text(statement, idxParam, record.xpElementType.c_str(), -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@poisson_ratio");
	sqlite3_bind_double(statement, idxParam, record.poissonRatio);

	idxParam = sqlite3_bind_parameter_index(statement, "@young_modulo");
	sqlite3_bind_double(statement, idxParam, record.youngModulo);

	//Write statistics
	idxParam = sqlite3_bind_parameter_index(statement, "@solver_threads");
	sqlite3_bind_double(statement, idxParam, record.ctSolverThreads);

	idxParam = sqlite3_bind_parameter_index(statement, "@anim_fps");
	sqlite3_bind_double(statement, idxParam, record.animFPS);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_anim_totalframe");
	sqlite3_bind_double(statement, idxParam, record.msAnimTotalFrame);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_anim_syssolver");
	sqlite3_bind_double(statement, idxParam, record.msAnimSysSolver);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_anim_applydisplacements");
	sqlite3_bind_double(statement, idxParam, record.msAnimApplyDisplacements);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_poly_triangletime");
	sqlite3_bind_double(statement, idxParam, record.msPolyTriangleMesh);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_poly_tetrahedratime");
	sqlite3_bind_double(statement, idxParam, record.msPolyTetrahedraMesh);

	//ms_rbf_creation, @ms_rbf_evaluation
	idxParam = sqlite3_bind_parameter_index(statement, "@ms_rbf_creation");
	sqlite3_bind_double(statement, idxParam, record.msRBFCreation);

	idxParam = sqlite3_bind_parameter_index(statement, "@ms_rbf_evaluation");
	sqlite3_bind_double(statement, idxParam, record.msRBFEvaluation);

	int rc = sqlite3_step(statement);
	if (rc != SQLITE_DONE) {
		printf("SQL error: %s", sqlite3_errmsg(lpDB));
		sqlite3_free(lpSqlError);
		return false;
	}

	//Free Statement
	sqlite3_finalize(statement);

	return true;
}
