#pragma once
#ifndef PS_ERRORMANAGER_H
#define PS_ERRORMANAGER_H


#include "PS_String.h"
#include <vector>

#define MAX_ERRORS_TO_KEEP 100

using namespace std;

namespace PS{

#define MarkError() CErrorManager::GetInstance().PushError(__FILE__, __LINE__)
#define ReportError(message) CErrorManager::GetInstance().PushError(__FILE__, __LINE__, message)
#define ReportErrorExt(message, arg) CErrorManager::GetInstance().PushErrorExt(__FILE__, __LINE__, message, arg)
#define FlushAllErrors() CErrorManager::GetInstance().FlushErrors()

typedef void (*FOnPopError)(const char* message);

class CErrorManager
{
public:
	static CErrorManager& GetInstance()
	{
		if (!sm_pErrorManager)
		{
			sm_pErrorManager = new CErrorManager();
		}

		return *sm_pErrorManager;
	}

	bool HasErrors() const { return (m_stkErrors.size() > 0);}
	void ClearErrors();
	size_t CountErrors() const {return m_stkErrors.size();}


	void PushError(const char *file, int line);
	void PushError(const char *file, int line, const char *message);
	void PushError(const char *message);
	void PushErrorExt(const char *file, int line, const char *pFmt, ...);
	void PopError();
	void PopMostRecentThenCleanup();
	void displayError(const char* chrError) const;
	void FlushErrors();


	void setCallBackPopError(FOnPopError cb) {m_fOnPopError = cb;}
	void setWriteToFile(bool bWrite) {m_bWriteToFile = bWrite;}
	void setDisplayRightAway(bool bDisplay) {m_bDisplayRightAway = bDisplay;}
private:
	CErrorManager(void);
	CErrorManager(const CErrorManager&);
	CErrorManager& operator=(const CErrorManager&);
	virtual ~CErrorManager(void);

	void pushToStack(const DAnsiStr& strError);
	DAnsiStr FormatError(const char *file, int line, const char *message) const;
	DAnsiStr FormatError(const char *message) const;
protected:
	bool			  m_bWriteToFile;
	bool			  m_bDisplayRightAway;
	FOnPopError		  m_fOnPopError;
	std::vector<DAnsiStr> m_stkErrors;
	static CErrorManager* sm_pErrorManager;

};





}
#endif // ERRORMANAGER_INCLUDED

