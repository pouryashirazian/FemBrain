#pragma once
#ifndef PS_APPCONFIG_H
#define PS_APPCONFIG_H

#include <vector>
#include <fstream>
#include "PS_String.h"


using namespace std;

//using namespace PS::MATH;

namespace PS{


	class CAppConfig
	{
	public:
		enum FileMode {fmRead, fmWrite, fmReadWrite, fmMemoryStream};
	protected:
		DAnsiStr   m_strFileName;
		FileMode   m_fmode;
		std::vector<DAnsiStr> m_content;

		bool readFile();
		bool writeFile();

	public:
		//const int MAX_LINE_SIZE;
		CAppConfig():m_strFileName(""){};
		CAppConfig(DAnsiStr strFileName, FileMode mode = fmReadWrite);
		virtual ~CAppConfig();

		FileMode getFileMode() const {return m_fmode;}
		DAnsiStr getFileName() const {return m_strFileName;}


		void clearContentBuffer();
		void getContentBuffer(std::vector<DAnsiStr> outContentBuf) const;
		void setContentBuffer(const std::vector<DAnsiStr>& inContentBuf);

		void set(DAnsiStr strFileName, FileMode mode = fmReadWrite);
		bool setForRead();
		void setForWrite();

		DAnsiStr getFilePath() const {return m_strFileName;}

		int hasSection(DAnsiStr strSection);
		bool writeValue(DAnsiStr section, DAnsiStr variable, DAnsiStr strValue);

		bool readLine(DAnsiStr section, DAnsiStr variable, DAnsiStr& strLine);
		bool readValue(DAnsiStr section, DAnsiStr variable, DAnsiStr& strValue);

		int readInt(DAnsiStr section, DAnsiStr variable, int def = 0);
		void writeInt(DAnsiStr section, DAnsiStr variable, int val);

		float readFloat(DAnsiStr section, DAnsiStr variable, float def = 0.0f);
		void writeFloat(DAnsiStr section, DAnsiStr variable, float val);

		double readDouble(DAnsiStr section, DAnsiStr variable, double def = 0.0);
		void writeDouble(DAnsiStr section, DAnsiStr variable, double val);

		DAnsiStr readString(DAnsiStr section, DAnsiStr variable, DAnsiStr def = "");
		void writeString(DAnsiStr section, DAnsiStr variable, DAnsiStr val);

		bool readBool(DAnsiStr section, DAnsiStr variable, bool def = false);
		void writeBool(DAnsiStr section, DAnsiStr variable, bool val);

		bool readIntArray(DAnsiStr section, DAnsiStr variable, int ctExpected, std::vector<int>& arrayInt);
		int writeIntArray(DAnsiStr section, DAnsiStr variable, const std::vector<int>& arrayInt);

	};
}
#endif
