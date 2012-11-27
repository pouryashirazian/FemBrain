/*
 * DBLogger.cpp
 *
 *  Created on: Nov 19, 2012
 *      Author: pourya
 */

#include "DBLogger.h"
#include "../AA_Sqlite/sqlite3.h"
#include "../AA_Sqlite/CppSQLite3.h"
#include "../PS_Base/PS_Logger.h"
#include "../PS_Base/PS_FileDirectory.h"

using namespace PS;
using namespace PS::FILESTRINGUTILS;

DBLogger::DBLogger()
{
	DAnsiStr strFP = ExtractFilePath(GetExePath()) + "fembrain_logs.db";
	m_strDB = string(strFP.c_str());

	this->createTable();
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
	const char *strSQL = "INSERT INTO tblPerfLog (xpID, xpModelName, xpTime, xpForceModel, xpIntegrator, totalVolume, restVolume, ctVertices, ctElements, elementType, poissonRatio, youngModulo) values"
						"(@xp_id, @xp_model_name, @xp_time, @xp_force_model, @xp_integrator, @total_volume, @rest_volume, @ct_vertices, @ct_elements, @element_type, @poisson_ratio, @young_modulo);";
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
						 "elementType VARCHAR(16), poissonRatio double, youngModulo double);";
	rc = sqlite3_exec(db, strSQL1, sqlite_Callback, 0, &lpSqlError);
	if(rc != SQLITE_OK)
	{
		LogErrorArg1("SQL error: %s\n", lpSqlError);
		sqlite3_free(lpSqlError);
		return false;
	}

	return true;
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
