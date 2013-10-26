/*
 * Profiler.cpp
 *
 *  Created on: Oct 14, 2013
 *      Author: pourya
 */
#include "Profiler.h"
#include "PS_Logger.h"
#include "PS_FileDirectory.h"
#include <fstream>

using namespace PS::FILESTRINGUTILS;

namespace PS {

ProfileAutoEvent::ProfileAutoEvent(const char* filename, const char* funcname, int line, const char* desc) {
	psProfileStart(filename, funcname, line, desc);
}

ProfileAutoEvent::~ProfileAutoEvent() {
	psProfileEnd();
}
///////////////////////////////////////////////////////////////////////////////////////////////////

ProfileEvent::ProfileEvent(const char* filename, const char* funcname, int line, const char* desc) {
	m_strFileName = DAnsiStr(filename);
	m_strFuncName = DAnsiStr(funcname);
	if(desc == NULL)
		m_strDesc = DAnsiStr("ProfileEvent");
	else
		m_strDesc = DAnsiStr(desc);
	m_line = line;
	m_timeMS = 0.0;
}

ProfileEvent::~ProfileEvent() {
}

void ProfileEvent::start() {
	m_start = tbb::tick_count::now();
}

double ProfileEvent::end() {
	m_end = tbb::tick_count::now();
	m_timeMS =  (m_end - m_start).seconds() * 1000.0;
	return m_timeMS;
}

//////////////////////////////////////////////////////////////////////////////////
Profiler::Profiler() {
	m_flags = pbInjectToLogger | pbWriteToTextFile;
	m_strTextFile = ChangeFileExt(GetExePath(), DAnsiStr(".psprofile"));
	m_strSQLDB = ChangeFileExt(GetExePath(), DAnsiStr(".sqlite"));
	m_globalStart = GetTickCount();
}

Profiler::~Profiler() {
	flush();
}

void Profiler::cleanup() {
	for(U32 i=0; i<m_vEvents.size(); i++) {
		SAFE_DELETE(m_vEvents[i]);
	}
	m_vEvents.resize(0);
	m_stkPending.resize(0);
}

void Profiler::flush() {
	if((m_flags & pbWriteToTextFile) != 0)
		this->writeToTextFile();
	if((m_flags & pbWriteToSqlDB) != 0)
		this->writeToSqlDB();
	cleanup();
}

void Profiler::startEvent(const char* filename, const char* funcname, int line, const char* desc) {
	ProfileEvent* lpEvent = new ProfileEvent(filename, funcname, line, desc);
	lpEvent->start();
	m_vEvents.push_back(lpEvent);
	m_stkPending.push_back(m_vEvents.size() - 1);
}

double Profiler::endEvent() {
	if(m_stkPending.size() == 0)
		LogError("The Profiler stack is empty! Did you forget to end an event before starting a new one?");
	ProfileEvent* lpEvent = m_vEvents[m_stkPending.back()];
	double t = lpEvent->end();

	//Remove index from unfinished ones
	m_stkPending.pop_back();

	//Inject log info
	if((m_flags & pbInjectToLogger) != 0) {
		DAnsiStr strSource = lpEvent->filename() + DAnsiStr(":") + lpEvent->funcname();
		psLog(EventLogger::etInfo, strSource.cptr(), lpEvent->line(), "%s Took %.4f [ms]", lpEvent->desc().cptr(), lpEvent->timeMS());
	}

	//Flush if exceeded the container size and there is no pending events
	if(m_vEvents.size() > MAX_LOG_EVENTS && !hasPendingEvents())
		flush();

	return t;
}

void Profiler::addEvent(ProfileEvent* lpEvent) {
	m_vEvents.push_back(lpEvent);
}

tick Profiler::GetTickCount() {
	return tbb::tick_count::now();
}

/*!
 * Write profiler events into the text file
 */
int Profiler::writeToTextFile() {
	if(m_vEvents.size() == 0)
		return 0;

	ofstream ofs;
	if(FileExists(m_strTextFile))
		ofs.open(m_strTextFile.cptr(), ios::out | ios::app);
	else
		ofs.open(m_strTextFile.cptr(), ios::out);
	if(!ofs.is_open())
		return false;

	ProfileEvent* lpEvent = NULL;
	DAnsiStr strLine;
	for(size_t i=0; i < m_vEvents.size(); i++)
	{
		lpEvent = m_vEvents[i];
		strLine = lpEvent->filename() + DAnsiStr(":") + lpEvent->funcname() + printToAStr(":%d", lpEvent->line());
		strLine += printToAStr(" %s took %.4f [ms]", lpEvent->desc().cptr(), lpEvent->timeMS());
		ofs << strLine << '\0' << endl;
	}
	ofs.close();

	return 1;
}

int Profiler::writeToSqlDB() {
	return 1;
}
//////////////////////////////////////////////////////////////////////////////////////////
void psProfileStart(const char* filename, const char* funcname, int line, const char* desc) {
	TheProfiler::Instance().startEvent(filename, funcname, line, desc);
}

void psProfileEnd() {
	TheProfiler::Instance().endEvent();
}


}


