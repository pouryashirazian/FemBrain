#include "../PS_Base/PS_MathBase.h"
#include "loki/Singleton.h"
#include <string>

using namespace std;
using namespace Loki;

class DBLogger{
public:
	DBLogger();
	virtual ~DBLogger();

	//Defines a polymorphic record in database
	struct Record{
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
	void append(const Record& record)
	{
		m_vRecords.push_back(record);
		if(m_vRecords.size() > 128)
			flush();
	}

	void flush()
	{
		for(U32 i=0; i<m_vRecords.size(); i++)
			this->insert(m_vRecords[i]);
		m_vRecords.resize(0);
	}

	static string timestamp();

private:
	//Returns the xp id from the table
	int getLastXPID() const;
	bool isLogTableExist() const;
	bool createTable();

private:
	string m_strDB;
	vector<Record> m_vRecords;
};

typedef SingletonHolder<DBLogger, CreateUsingNew, PhoenixSingleton> TheDataBaseLogger;
