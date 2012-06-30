//#include "stdafx.h"
#ifdef _WIN32
	#include <windows.h>
#endif // _WIN32

#if defined(__linux__)
	#include <stdarg.h>
#endif
#include <stdexcept>
//#include <vector>

//#include "PS_Exceptions.h"
#include "PS_FileDirectory.h"
#include "PS_ErrorManager.h"
#include <fstream>


namespace PS{

CErrorManager* CErrorManager::sm_pErrorManager = NULL;

CErrorManager::CErrorManager(void)
{
	m_bDisplayRightAway = true;
	m_bWriteToFile = true;
	m_fOnPopError = NULL;
	m_stkErrors.clear();
}

CErrorManager::~CErrorManager(void)
{
	FlushErrors();
	m_fOnPopError = NULL;
	sm_pErrorManager = NULL;
}


void CErrorManager::PushError(const char *file, int line)
{
	pushToStack(FormatError(file, line, "An assertion failure has occurred."));
}

void CErrorManager::PushError(const char *file, int line, const char *message)
{
	pushToStack(FormatError(file, line, message));
}

void CErrorManager::PushErrorExt(const char *file, int line, const char *pFmt, ...)
{
	va_list	vl;
	va_start( vl, pFmt );

	char	buff[1024];

#ifdef PS_SECURE_API
	vsnprintf_s( buff, _countof(buff)-1, _TRUNCATE, pFmt, vl );
#else
	vsnprintf(buff, sizeof(buff)-1, pFmt, vl);
#endif
	va_end( vl );

	DAnsiStr strError = DAnsiStr(buff);
	DAnsiStr strHeader = printToAStr("%s(%d) : error: ", file, line);
	pushToStack(strHeader + strError);
}

void CErrorManager::PushError(const char *message)
{
	pushToStack(FormatError(message));
}

void CErrorManager::pushToStack(const DAnsiStr& strError)
{
	if(strError.length() == 0)
		return;
	if(m_stkErrors.size() > MAX_ERRORS_TO_KEEP)
		FlushErrors();
	else if(m_bDisplayRightAway)
	{
		displayError(strError.ptr());
	}
	m_stkErrors.push_back(strError);
}

void CErrorManager::PopMostRecentThenCleanup()
{
	if(m_stkErrors.size() == 0) return;
	DAnsiStr strError = m_stkErrors.back();
	if (strError.length() == 0) return;
	
	 DAnsiStr strMsg = printToAStr("There are %d errors. I am going to clear all of them but this is the most recent one:\n", m_stkErrors.size());
	 strMsg += strError;
	 displayError(strMsg.cptr());
	 m_stkErrors.resize(0);
}

/**
*/
void CErrorManager::PopError()
{
	if(m_stkErrors.size() == 0) return;
	DAnsiStr strError = m_stkErrors.back();
	if (strError.length() != 0)
		displayError(strError.cptr());

	m_stkErrors.pop_back();
}

void CErrorManager::displayError(const char* chrError) const
{
	if(chrError == NULL) return;
	if(m_fOnPopError)
		m_fOnPopError(chrError);
	else
	{
	#ifdef WIN32
		OutputDebugStringA(chrError);
		DWideStr wstr(chrError);
		MessageBoxW(0, (LPCWSTR)wstr.ptr(), (LPCWSTR)L"Error", MB_OK);
	#else // !_WIN32
		fprintf(stderr, "\n");
		fprintf(stderr, chrError);
	#endif // _WIN32
	}
}

void CErrorManager::ClearErrors()
{
	if(m_stkErrors.size() > 0 )
	{
		DAnsiStr strMsg = printToAStr("Clearing %d errors.", m_stkErrors.size());
		displayError(strMsg.ptr());
		m_stkErrors.clear();
	}	
}

/**
*/
void CErrorManager::FlushErrors()
 {
	if (m_bWriteToFile && m_stkErrors.size() > 0)
	{
		DAnsiStr strFN = PS::FILESTRINGUTILS::GetExePath();
		strFN = PS::FILESTRINGUTILS::ChangeFileExt(strFN, DAnsiStr(".log"));

		//Writing the stack error to disk
		ofstream ofs(strFN.ptr(), ios::out | ios::trunc);
		if (!ofs.is_open())
			return;

		DAnsiStr strLine;
		for (size_t i = 0; i < m_stkErrors.size(); i++)
		{
			strLine = m_stkErrors[i];
			if (strLine.length() > 0)
				ofs << strLine << '\0' << '\n';
		}
		ofs.close();
	}
	while (m_stkErrors.size() > 0) {
		PopError();
	}
}

/**
*/
DAnsiStr CErrorManager::FormatError(const char *file, int line, const char *message) const
{
	DAnsiStr strError;

	if(message)
		strError = printToAStr("%s(%d) : error: %s", file, line, message);
	else
		strError = printToAStr("%s(%d) : error: (invalid error message)", file, line);
	return strError;
}

/**
*/
DAnsiStr CErrorManager::FormatError(const char *message) const
{
	DAnsiStr strError;
	if (message)
		strError = printToAStr("error: %s", message);
	else 
		strError = DAnsiStr("error: (invalid error message)");

	return strError;
}

}
