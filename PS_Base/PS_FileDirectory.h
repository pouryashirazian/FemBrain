#pragma once
#include <string>
#include <vector>
#include "PS_String.h"

using namespace std;

namespace PS{
	namespace FILESTRINGUTILS{

		long GetFileSize(const char* chrFilePath);

		bool FileExists(const DAnsiStr& strFilePath);

		DAnsiStr GetExePath();

		void GetExePath(char *exePath, int szBuffer);

		DAnsiStr ExtractFileTitleOnly(const DAnsiStr& strFilePath);

		DAnsiStr ExtractFilePath(const DAnsiStr& strFilePath);

		DAnsiStr ExtractFileName(const DAnsiStr& strFilePath);

		/*!
		 * Extracts one level up directory. If the original path contains a slash at the end, the
		 * output path will also have an slash.
		 */
		DAnsiStr ExtractOneLevelUp(const DAnsiStr& strFilePath);

		/*!
		 * Returns file extension without dot
		 */
		DAnsiStr ExtractFileExt(const DAnsiStr& strPathFileName);

		DAnsiStr CreateNewFileAtRoot(const char* pExtWithDot);

		DAnsiStr ChangeFileExt(const DAnsiStr& strFilePath, const DAnsiStr& strExtWithDot);


        //int ListFilesOrDirectories(std::vector<DAnsiStr>& lstOutput, const char* chrPath, const char* chrExt, bool bDirOnly);
		int ListFilesInDir(std::vector<DAnsiStr>& lstFiles, const char* pDir, const char* pExtensions, bool storeWithPath);
		//int ListFilesInDir(DVec<DAnsiStr>& lstFiles, const char* pDir, const char* pExtensions, bool storeWithPath);


        bool WriteTextFile(const DAnsiStr& strFN, const std::vector<DAnsiStr>& content);
        bool WriteTextFile(const DAnsiStr& strFN, const DAnsiStr& strContent );

        bool ReadTextFile(const DAnsiStr& strFN, std::vector<DAnsiStr>& content);
        bool ReadTextFile(const DAnsiStr& strFN, DAnsiStr& content);
	}
}
