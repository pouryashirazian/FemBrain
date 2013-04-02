#include "../PS_Base/PS_MathBase.h"
#include "../AA_Sqlite/sqlite3.h"
#include "loki/Singleton.h"
#include <tbb/task.h>
#include <tbb/concurrent_queue.h>
#include <string>

using namespace std;
using namespace Loki;

#define RECORDS_STORAGE_SPACE 32

//Structure to share result between gui and task thread
struct TaskLogInsertResult {
	U32 ctWrittenRecords;
};

//CALLBACK Definition
typedef void (*FOnInsertionCompleted)();


/*!
 * DBLogger handles all performance and events to be written to the database
 */
class DBLogger{
public:
	DBLogger();
	virtual ~DBLogger();

	//Defines a polymorphic record in database
	struct Record{
		U32 xpID;
		string xpModelName;
		string xpTime;
		string xpForceModel;
		string xpIntegrator;
		string xpElementType;

		double youngModulo;
		double poissonRatio;

		double msComputeTetrahedra;
		double msComputeDeformation;
		double msRenderTime;

		double restVolume;
		double totalVolume;
		U32 ctVertices;
		U32 ctElements;
		U32 szTotalMemUsed;
	};

	bool insert(const Record& record);
	void append(Record& record)
	{
		m_lastXPID ++;
		record.xpID = m_lastXPID;
		m_vRecords.push_back(record);
		if(m_vRecords.size() > RECORDS_STORAGE_SPACE)
			flush();
	}

	void flush();

	static string timestamp();

private:
	//Returns the xp id from the table
	int getLastXPID() const;
	bool isLogTableExist() const;
	bool createTable();

private:
	U32 m_lastXPID;
	string m_strDB;
	vector<Record> m_vRecords;
};

/*!
 * Insertion into database should be done in a separate thread for efficiency
 */
class DBInsertionTask : public tbb::task {

public:
	DBInsertionTask(const string& strDB,
					  const std::vector<DBLogger::Record>& vRecords,
					  FOnInsertionCompleted fOnCompleted);

	bool insertRecord(sqlite3* lpDB, const DBLogger::Record& record);
	tbb::task* execute();

private:
	string m_strDB;
	std::vector<DBLogger::Record> m_vRecords;
	FOnInsertionCompleted m_fOnCompleted;
};

typedef SingletonHolder<DBLogger, CreateUsingNew, PhoenixSingleton> TheDataBaseLogger;
