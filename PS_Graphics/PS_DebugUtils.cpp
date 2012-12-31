/*
 * PS_DebugUtils.cpp
 *
 *  Created on: 2012-02-16
 *      Author: pourya
 */
#include "PS_DebugUtils.h"
#include "../PS_Base/PS_FileDirectory.h"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace PS;
using namespace PS::FILESTRINGUTILS;

namespace PS{
namespace DEBUG{

int SaveArrayCSV(const char* lpArrayName, float* lpArray, U32 count)
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

void PrintArray(U32* lpData, U32 count)
{
	printf("\n");
	for(U32 i=0; i<count; i++)
	{
		printf("%d, ", lpData[i]);
	}
	printf("\n");
}

void PrintArrayF(float* lpData, U32 count)
{
	for(U32 i=0; i<count; i++)
	{
		printf("%.2f, ", lpData[i]);
	}
	printf("\n");
}

void FillArray(U32* lpData, U32 count, U32 nMin, U32 nMax)
{
	for(int i=0; i < count; i++)
		lpData[i] = static_cast<U32>(RandRangeT<float>(nMin, nMax));
}

void FillArrayF(float* lpData, U32 count, float nMin, float nMax)
{
	for(int i=0; i < count; i++)
		lpData[i] = RandRangeT<float>(nMin, nMax);
}

}
}
