/*
 * PS_DebugUtils.cpp
 *
 *  Created on: 2012-02-16
 *      Author: pourya
 */
#include "PS_DebugUtils.h"
#include "PS_FileDirectory.h"

using namespace PS;
using namespace PS::FILESTRINGUTILS;

int Debug_SaveArrayCSV(const char* lpArrayName, float* lpArray, U32 count)
{

	DAnsiStr strPath = ExtractFilePath(GetExePath());
	DAnsiStr strFP = printToAStr("%s/%s", strPath.ptr(), lpArrayName);
	DAnsiStr strLine;
	for(int i=0; i<count; i++)
	{
		if(i < count - 1)
			strLine += printToAStr("%f, ", lpArray[i]);
		else
			strLine += printToAStr("%f", lpArray[i]);
	}

	WriteTextFile(strFP, strLine);
	return count;
}


