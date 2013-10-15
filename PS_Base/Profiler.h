/*
 * Profiler.h
 *
 *  Created on: Oct 14, 2013
 *      Author: pourya
 */

#ifndef PROFILER_H_
#define PROFILER_H_

#include <vector>
#include <time.h>
#include "PS_String.h"
#include "loki/Singleton.h"
#include "tbb/tick_count.h"

using namespace std;
using namespace Loki;
using namespace tbb;

#define MAX_LOG_EVENTS	32

//Easy usage with preprocessor
#define ProfileAuto() PS::ProfileAutoEvent profile(__FILE__, __FUNCTION__, __LINE__, NULL);
#define ProfileAutoArg(desc) PS::ProfileAutoEvent profile(__FILE__, __FUNCTION__, __LINE__, desc);

#define	 ProfileStart() psProfileStart(__FILE__, __FUNCTION__, __LINE__, NULL);
#define	 ProfileStartArg(desc) psProfileStart(__FILE__, __FUNCTION__, __LINE__, desc);
#define ProfileEnd() psProfileEnd();

namespace PS {

typedef tbb::tick_count		tick;

class ProfileAutoEvent {
public:
	ProfileAutoEvent(const char* filename, const char* funcname, int line, const char* desc = NULL);
	~ProfileAutoEvent();
};

/*!
 * Profiler event is a small structure
 */
class ProfileEvent {
public:
	ProfileEvent(const char* filename, const char* funcname, int line, const char* desc = NULL);
	virtual ~ProfileEvent();

	//Set timestamps
	void start();
	double end();

	//Access
	double timeMS() const { return m_timeMS;}
	DAnsiStr filename() const {return m_strFileName;}
	DAnsiStr funcname() const {return m_strFuncName;}
	DAnsiStr desc() const {return m_strDesc;}
	int line() const {return m_line;}

	//Ticks
	tick getStartTick() const {return m_start;}
	tick getEndTick() const {return m_end;}
private:
	DAnsiStr m_strFileName;
	DAnsiStr m_strFuncName;
	DAnsiStr m_strDesc;
	int m_line;
	tick m_start;
	tick m_end;
	double m_timeMS;
};

/*!
 * Profiler manages all profiling events and provides access for reporting
 */
class Profiler {
public:
	enum Behaviour {pbInjectToLogger = 1, pbWriteToTextFile = 2, pbWriteToSqlDB = 4};
public:
	Profiler();
	virtual ~Profiler();
	void cleanup();
	void flush();

	//Event Generation
	void startEvent(const char* filename, const char* funcname, int line, const char* desc);
	double endEvent();

	void addEvent(ProfileEvent* lpEvent);

	//Flags
	int flags() const { return m_flags;}
	void setWriteFlags(int flags) {m_flags = flags;}
	bool hasPendingEvents() const { return (m_stkPending.size() > 0);}


	static tick GetTickCount();

protected:
	//Serialization
	int writeToTextFile();
	int writeToSqlDB();
private:
	tick m_globalStart;
	std::vector<ProfileEvent*> m_vEvents;
	std::vector<U32> m_stkPending;
	int m_flags;
	DAnsiStr m_strTextFile;
	DAnsiStr m_strSQLDB;
};

//Singleton Instance
typedef SingletonHolder<Profiler, CreateUsingNew, PhoenixSingleton> TheProfiler;

void psProfileStart(const char* filename, const char* funcname, int line, const char* desc = NULL);
void psProfileEnd();

}

#endif /* PROFILER_H_ */
