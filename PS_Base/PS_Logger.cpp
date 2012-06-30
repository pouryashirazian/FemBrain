#include "PS_Logger.h"
#include "PS_FileDirectory.h"
#include <fstream>
#include <stdexcept>

#ifdef _WIN32
#include <windows.h>
#elif defined(__linux__)
#include <stdarg.h>
#endif


using namespace PS::FILESTRINGUTILS;

namespace PS{


	void psLog(const char* lpDesc, ...)
	{
		va_list	vl;
		va_start( vl, lpDesc );

		char	buff[PS_LOG_LINE_SIZE];
#ifdef PS_SECURE_API
		vsnprintf_s( buff, _countof(buff)-1, _TRUNCATE, pFmt, vl );
#else
		vsnprintf(buff, sizeof(buff)-1, lpDesc, vl);
#endif
		va_end( vl );

		TheEventLogger::Instance().add(buff, EventLogger::etInfo);
	}

	//Log a single line
	void psLog(EventLogger::EVENTTYPE etype, const char* lpSource, int line, const char* lpDesc, ...)
	{
		va_list	vl;
		va_start( vl, lpDesc );

		char	buff[PS_LOG_LINE_SIZE];
#ifdef PS_SECURE_API
		vsnprintf_s( buff, _countof(buff)-1, _TRUNCATE, pFmt, vl );
#else
		vsnprintf(buff, sizeof(buff)-1, lpDesc, vl);
#endif
		va_end( vl );

		TheEventLogger::Instance().add(buff, etype, lpSource, line);
	}


	//Class EventLogger
	EventLogger::EventLogger()
	{
		m_fOnDisplay = NULL;
		m_strFP = ChangeFileExt(GetExePath(), DAnsiStr(".pslog"));			
		m_szBufferSize = 0;
		setWriteFlags(PS_LOG_WRITE_EVENTTYPE | PS_LOG_WRITE_SOURCE | PS_LOG_WRITE_TO_SCREEN);
	}

	EventLogger::EventLogger(const char* lpFilePath, int flags)
	{
		m_fOnDisplay = NULL;
		m_strFP = DAnsiStr(lpFilePath);
		m_szBufferSize = 0;
		setWriteFlags(flags);
	}

	EventLogger::~EventLogger()
	{
		flush();
	}

	void EventLogger::setWriteFlags(int flags)
	{
		m_bWriteEventTypes = ((flags & PS_LOG_WRITE_EVENTTYPE) != 0);
		m_bWriteTimeStamps = ((flags & PS_LOG_WRITE_TIMESTAMP) != 0);
		m_bWriteSourceInfo = ((flags & PS_LOG_WRITE_SOURCE) != 0);		
		m_bWriteToScreen   = ((flags & PS_LOG_WRITE_TO_SCREEN) != 0);
		m_bWriteToModal    = ((flags & PS_LOG_WRITE_TO_MODAL) != 0);
	}

	void EventLogger::setOutFilePath(const char* lpStrFilePath)
	{
		m_strFP = DAnsiStr(lpStrFilePath);
	}

	void  EventLogger::add(const Event& e)
	{
		add(e.strDesc.cptr(), e.etype, e.strSource.cptr(), e.value); 
	}


	void EventLogger::add(const char* lpStrDesc, EVENTTYPE t, 
		const char* lpStrSource, int value)
	{
		if(lpStrDesc == NULL)
			return;

		DAnsiStr strEvent;			
		//Write Event Type
		if(m_bWriteEventTypes)
		{
			if(t == etInfo)
				strEvent += printToAStr("INFO: ");
			else if(t == etWarning)
				strEvent += printToAStr("WARNING: ");
			else if(t == etError)
				strEvent += printToAStr("ERROR: ");
		}

		//Write Event Time
		if(m_bWriteTimeStamps)
		{
			time_t rawtime;
			time (&rawtime);
#ifdef PS_SECURE_API
			char buffer[64];
			struct tm timeinfo;
			localtime_s(&timeinfo, &rawtime);
			asctime_s(timeinfo, buffer, 64)
				strEvent += printToAStr("TIME: [%s], ", buffer);
#else
			struct tm * timeinfo = localtime ( &rawtime );
			DAnsiStr strTime = DAnsiStr(asctime(timeinfo));
			strTime.trim();
			strEvent += printToAStr("TIME: [%s], ", strTime.cptr());
#endif
		}

		//Write Source + Value
		if(m_bWriteSourceInfo && lpStrSource)
		{
			strEvent += printToAStr("SOURCE: [%s, LINE:%d], ", lpStrSource, value);
		}

		//Write Event itself
		strEvent += DAnsiStr(lpStrDesc);			
		m_lstLog.push_back(strEvent);

		//Write Message to screen
		if(m_bWriteToScreen)
			display(strEvent.cptr());

		//Update Buffer size and Flush if ready
		m_szBufferSize += strEvent.length();
		if(m_szBufferSize > PS_LOG_BUFFER_SIZE)
			flush();
	}

	void EventLogger::display(const char* chrMessage) const
	{
		if(chrMessage == NULL) return;

		if(m_fOnDisplay)
			m_fOnDisplay(chrMessage);
		else
		{
#ifdef WIN32
			OutputDebugStringA(chrMessage);
			fprintf(stderr, chrMessage);			
			//DWideStr wstr(chrMessage);
			//MessageBoxW(0, (LPCWSTR)wstr.ptr(), (LPCWSTR)L"LOGSYSTEM", MB_OK);			
#else // !_WIN32
			fprintf(stderr, "\n");
			fprintf(stderr, chrMessage);
#endif // _WIN32
		}
	}


	//Open File and Append
	bool EventLogger::flush()
	{
		if(m_lstLog.size() == 0)
			return false;

		ofstream ofs;
		if(FileExists(m_strFP))		
			ofs.open(m_strFP.cptr(), ios::out | ios::app);
		else
			ofs.open(m_strFP.cptr(), ios::out);
		if(!ofs.is_open())
			return false;

		DAnsiStr strLine;
		for(size_t i=0; i < m_lstLog.size(); i++)
		{
			strLine = m_lstLog[i];
			if(strLine.length() > 0)
			{
				ofs << strLine << '\0' << endl;
			}
		}
		ofs.close();

		m_lstLog.resize(0);
		m_szBufferSize = 0;
		return true;
	}

}
