//#include "stdafx.h"
#include "PS_FileDirectory.h"
#include <fstream>

#ifdef WIN32
	#include <io.h>
	#include "Windows.h"
#elif defined(__linux__)
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <dirent.h>
	#include <unistd.h>
#endif


namespace PS
{

namespace FILESTRINGUTILS
{

long GetFileSize(const DAnsiStr& strFilePath)
{
	ifstream ifs(strFilePath.ptr(), ios::in | ios::binary );
	if(!ifs.is_open())
		return -1;
		
	long begin, end;
	begin = ifs.tellg();
	ifs.seekg(0, ios::end);
	end = ifs.tellg();	
	ifs.close();

	return end - begin;
}


//==================================================================
bool FileExists(const DAnsiStr& strFilePath)
{
	ifstream ifs(strFilePath.ptr(), ios::in | ios::binary );
	if(ifs.is_open())
	{
		ifs.close();
		return true;
	}
	else
	{
		return false;
	}	
}
//==================================================================
DAnsiStr GetExePath()
{
	char buff[1024];
	GetExePath(buff, 1024);
	DAnsiStr strOut = DAnsiStr(buff);
	return strOut;	
}
//==================================================================
void GetExePath(char *exePath, int szBuffer)
{
#ifdef WIN32
	WCHAR wszExeName[MAX_PATH + 1];
	wszExeName[MAX_PATH] = 0;
	GetModuleFileNameW(NULL, wszExeName, sizeof(wszExeName) - 1);
	WideCharToMultiByte(CP_ACP, 0, wszExeName, -1, (LPSTR)exePath, szBuffer, NULL, NULL);
#elif defined(__linux__)
	//getcwd only retrieves the current working directory
	//getcwd(exePath, (size_t)szBuffer);

	pid_t pid = getpid();
	DAnsiStr strProcessPath = printToAStr("/proc/%d/exe", pid);

	int nCharsWritten = readlink(strProcessPath.ptr(), exePath, szBuffer);
	if(nCharsWritten != -1)
	{
		exePath[nCharsWritten] = 0;

	}


#endif
}
//==================================================================
DAnsiStr ExtractFileTitleOnly(const DAnsiStr& strFilePath)
{
	size_t pos = 0;
	DAnsiStr strTemp = ExtractFileName(strFilePath);
	if(strTemp.lfind('.', pos))
		strTemp = strTemp.substr(0, pos);
	return strTemp;
}
//==================================================================
DAnsiStr ExtractFileName(const DAnsiStr& strPathFileName)
{
	size_t npos;

	DAnsiStr strOutput = strPathFileName;
	if(strPathFileName.rfind(L'/', npos) || strPathFileName.rfind(L'\\', npos))
	{
		strOutput = strPathFileName.substr(npos+1);
	}

	return strOutput;
}
//==================================================================
DAnsiStr ExtractOneLevelUp(const DAnsiStr& strFilePath)
{
	DAnsiStr strOutput = strFilePath;
	bool bHasLastSlash = false;
	char lastSlash;

	//Remove last forward or backward slash
	while((strOutput.lastChar() == '/') || (strOutput.lastChar() == '\\'))
	{
		lastSlash = strOutput.lastChar();
		strOutput = strOutput.substr(0, strOutput.length() - 1);		
		bHasLastSlash = true;
	}

	size_t pos = 0;
	//Up one level
	if(strOutput.rfind('/', pos))
		strOutput = strOutput.substr(0, pos);
	else if(strOutput.rfind('\\', pos))
		strOutput = strOutput.substr(0, pos);
	
	if(bHasLastSlash)
		strOutput += lastSlash;

	return strOutput;
}
//==================================================================
DAnsiStr ChangeFileExt(const DAnsiStr& strFilePath, const DAnsiStr& strExtWithDot)
{
	DAnsiStr strOut;
	size_t npos;
	if(strFilePath.rfind('.', npos))
	{
		strOut = strFilePath.substr(0, npos);
		strOut += strExtWithDot;
	}
	else
		strOut = strFilePath + strExtWithDot;

	return strOut;
}
//==================================================================
DAnsiStr CreateNewFileAtRoot(const char* pExtWithDot)
{
	char buffer[1024];
	GetExePath(buffer, 1024);

	DAnsiStr strOutput(buffer);
	size_t posDot;
	if(strOutput.rfind(L'.', posDot))
	{
		DAnsiStr temp = strOutput.substr(0, posDot);
		temp.appendFromT(pExtWithDot);
		return temp;
	}
	else
		return strOutput;	
}

//==================================================================
DAnsiStr ExtractFilePath(const DAnsiStr& fileName)
{
	size_t npos;
	DAnsiStr strOutput;
	if(fileName.rfind(L'/', npos) || fileName.rfind(L'\\', npos))	
	{
		strOutput = fileName.substr(0, npos+1);
	}
	return strOutput;	
}
//==================================================================
DAnsiStr ExtractFileExt(const DAnsiStr& strPathFileName)
{
	size_t npos;
	DAnsiStr strOut;
	if(strPathFileName.rfind('.', npos))
	{
		strOut = strPathFileName.substr(npos+1);
	}
	return strOut;	
}
//==================================================================
/*
int ListFilesOrDirectories(std::vector<DAnsiStr>& lstOutput, const char* chrPath, const char* chrExt, bool bDirOnly)
{

#ifdef WIN32
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	DWideStr wstrDir = toWideString(chrPath);

	hFind = FindFirstFile(wstrDir.ptr(), &ffd);
	if(hFind == INVALID_HANDLE_VALUE)
		return 0;

	DAnsiStr temp;
	do
	{
		if(bDirOnly &&(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
        {
			if (storeWithPath)
			{
				temp = strResolvedDir + "\\";
				temp += DAnsiStr(ffd.cFileName);
				lstOutput.push_back(temp);
			}
			else
			{
				temp = DAnsiStr(ffd.cFileName);
				lstOutput.push_back(temp);
			}
        }
		else
		{
			temp = strResolvedDir + "\\";
			temp += DAnsiStr(ffd.cFileName);

			if(chrExt)
			{
				DAnsiStr strExt = PS::FILESTRINGUTILS::ExtractFileExt(DAnsiStr(temp.ptr()));
				if(strExt == DAnsiStr(chrExt))
					lstOutput.push_back(temp);
			}
			else
				lstOutput.push_back(temp);
		}
	}while (FindNextFile(hFind, &ffd) != 0);
   FindClose(hFind);

#else
	DIR *dp;
	struct dirent *pEntry;
	struct stat st;

	if((dp  = opendir(chrPath)) == NULL)
	{
			return 0;
	}

	while ((pEntry = readdir(dp)) != NULL)
	{
		DAnsiStr strFileName = DAnsiStr(pEntry->d_name);
		DAnsiStr strFullFileName = DAnsiStr(chrPath) + DAnsiStr("/") + strFileName;

		if(strFileName[0] == '.')
			continue;

		if(stat(strFullFileName.c_str(), &st) == -1)
			continue;

		const bool isDir = (st.st_mode & S_IFDIR) != 0;

		if(bDirOnly)
		{
			if(isDir)
				lstOutput.push_back(strFullFileName);
		}
		else
		{
			if(chrExt)
			{
				DAnsiStr strExt = PS::FILESTRINGUTILS::ExtractFileExt(strFullFileName);
				if(strExt == DAnsiStr(chrExt))
					lstOutput.push_back(strFullFileName);
			}
			else
				lstOutput.push_back(strFullFileName);
		}
	}
	closedir(dp);
#endif

	return lstOutput.size();
}
*/
//==================================================================
int ListFilesInDir(std::vector<DAnsiStr>& lstFiles, const char* pDir, const char* pExtensions, bool storeWithPath)
{
	DAnsiStr strDir;
	DAnsiStr strResolvedDir;

	lstFiles.resize(0);
	if(pDir == NULL)	
		strResolvedDir = ExtractFilePath(GetExePath());
	else
		strResolvedDir = DAnsiStr(pDir);
		

	if(pExtensions != NULL)
		strDir = printToAStr("%s/*.%s", strResolvedDir.ptr(), pExtensions);
	else
		strDir = printToAStr("%s/*.*", strResolvedDir.ptr()); 
		

#ifdef WIN32
		WIN32_FIND_DATA ffd;
		HANDLE hFind = INVALID_HANDLE_VALUE;
		DWideStr wstrDir = toWideString(strDir);

		hFind = FindFirstFile(wstrDir.ptr(), &ffd);
		if(hFind == INVALID_HANDLE_VALUE)
			return 0;

		DAnsiStr temp;
		do
		{
		    if (!(ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
	        {
				if (storeWithPath)
				{
					temp = strResolvedDir + "\\";
					temp += DAnsiStr(ffd.cFileName);
					lstFiles.push_back(temp);
				}
				else
				{
					temp = DAnsiStr(ffd.cFileName);
					lstFiles.push_back(temp);
				}
	        }
    	}while (FindNextFile(hFind, &ffd) != 0);
	   FindClose(hFind);

#elif defined(PS_OS_MAC)
	   return 0;
#else
	   DIR *dp;
	   struct dirent *dirp;
	   if((dp  = opendir(strDir.ptr())) == NULL)
	   {
			//cout << "Error(" << errno << ") opening " << dir << endl;
			//return errno;
			return 0;
	   }

	   while ((dirp = readdir(dp)) != NULL)
	   {
		   	lstFiles.push_back(DAnsiStr(dirp->d_name));
	   }
	   closedir(dp);
#endif

	return (int)lstFiles.size();
}

bool WriteTextFile( DAnsiStr strFN, const std::vector<DAnsiStr>& content )
{
	ofstream ofs(strFN.ptr(), ios::out | ios::trunc);
	if(!ofs.is_open())
		return false;

	DAnsiStr strLine;
	for(size_t i=0; i < content.size(); i++)
	{
		strLine = content[i];
		if(strLine.length() > 0)
		{
			ofs << strLine << '\0' << '\n';
		}
	}
	ofs.close();

	return true;
}

bool WriteTextFile( DAnsiStr strFN, const DAnsiStr& strContent )
{
	ofstream ofs(strFN.ptr(), ios::out | ios::trunc);
	if(!ofs.is_open())
		return false;

	ofs << strContent << '\0' << '\n';
	ofs.close();

	return true;
}

bool ReadTextFile( DAnsiStr strFN, std::vector<DAnsiStr>& content )
{
	ifstream ifs(strFN.ptr(), ios::in);
	if(!ifs.is_open())
		return false;

	DAnsiStr strLine;
	char buffer[2048];

	while( !ifs.eof())
	{
		ifs.getline(buffer, 2048);
		//ifs >> strLine;
		strLine.copyFromT(buffer);
		strLine.trim();
		content.push_back(strLine);
	}	
	ifs.close();

	return true;
}

bool ReadTextFile( DAnsiStr strFN, DAnsiStr& strContent )
{
	ifstream ifs(strFN.ptr(), ios::in);
	if(!ifs.is_open())
		return false;

	DAnsiStr strLine;
	char buffer[2048];

	while( !ifs.eof())
	{
		ifs.getline(buffer, 2048);
		//ifs >> strLine;
		strLine.copyFromT(buffer);
		strLine.trim();
		strContent.appendFrom(strLine);
	}
	ifs.close();

	return true;
}

}
}
