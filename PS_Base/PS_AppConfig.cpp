//#include "stdafx.h"
#include "PS_AppConfig.h"
#include "PS_FileDirectory.h"
#include "PS_ErrorManager.h"


#define MAX_CFG_LINE_SIZE 256

namespace PS{

	using namespace PS::FILESTRINGUTILS;

	//====================================================================================
	CAppConfig::CAppConfig(DAnsiStr strFileName, FileMode mode)
	{
		set(strFileName, mode);
	}
	//====================================================================================
	bool CAppConfig::setForRead()
	{
		//m_strFileName = changeFileExt(getExePath(), DAnsiStr(".inf"));
		m_strFileName = CreateNewFileAtRoot(".inf");
		m_fmode = fmRead;
		m_content.resize(0);
		return readFile();
	}
	//====================================================================================
	void CAppConfig::set( DAnsiStr strFileName, FileMode mode /*= fmReadWrite*/ )
	{
		m_fmode = mode;
		m_strFileName = strFileName;
		m_content.resize(0);

		if(m_fmode == fmRead || m_fmode == fmReadWrite)
			readFile();
	}
	//====================================================================================
	void CAppConfig::setForWrite()
	{
		m_strFileName = CreateNewFileAtRoot(".inf");
		m_fmode = fmWrite;
	}
	//====================================================================================
	CAppConfig::~CAppConfig()
	{
		if((m_fmode == fmReadWrite)||(m_fmode == fmWrite))
		{
			if(m_content.size() > 0)
			{
				writeFile();
			}
		}

		m_content.resize(0);
	}

	//====================================================================================
	bool CAppConfig::readFile()
	{

		ifstream ifs(m_strFileName.ptr(), ios::in);
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
			strLine.removeStartEndSpaces();
			m_content.push_back(strLine);
		}
		ifs.close();

		return true;
	}
	//====================================================================================
	bool CAppConfig::writeFile()
	{
		ofstream ofs(m_strFileName.ptr(), ios::out | ios::trunc);
		if(!ofs.is_open())
			return false;

		DAnsiStr strLine;
        for(int i=0; i < (int)m_content.size(); i++)
		{
			strLine = m_content[i];
			if(strLine.length() > 0)
			{
				ofs << strLine << '\0' << '\n';
			}
		}
		ofs.close();

		return true;
	}
	//====================================================================================
	int CAppConfig::hasSection(DAnsiStr strSection)
	{
		DAnsiStr str;
        for(int i=0; i < (int)m_content.size(); i++)
		{
			str = m_content[i];
			char ch = str.firstChar();
			ch = str.lastChar();

			if((str.firstChar() == '[')&&(str.lastChar() == ']'))
			{
				//[ + ] =  2
				str = str.substr(1, str.length() - 2);

				if( str.toUpper() == strSection.toUpper() )
					return i;
			}
		}

		return -1;
	}
	//====================================================================================
	bool CAppConfig::writeValue(DAnsiStr section, DAnsiStr variable, DAnsiStr strValue)
	{
		DAnsiStr strLine, strToAdd;
		strToAdd = variable;
		strToAdd += "=";
		strToAdd += strValue;

		int startPos = hasSection(section);
		if(startPos >= 0)
		{
			//Record file pos
            for(int i=startPos+1; i < (int)m_content.size(); i++)
			{
				strLine = m_content[i];

				//If it is a comment then ignore
				if(strLine.firstChar() == '#')
					continue;

				//Is it another section?
				if((strLine.firstChar() == L'[')&&(strLine.lastChar() == L']'))
				{
					m_content.insert(m_content.begin() + i, strToAdd);
					return true;
				}
				else
				{
					//Check Variable
                    for(int iChar=0; iChar < strLine.length(); iChar++)
					{
						if(strLine[iChar] == L'=')
						{
							strLine = strLine.substr(0, iChar);
							if( strLine.toUpper() == variable.toUpper() )
							{
								//Write it here
								m_content[i] = strToAdd;
								return true;
							}
							else
								break;
						}
					}
				}
			}

			//Not Written
			m_content.push_back(strToAdd);
		}
		else
		{
			//Add it if not found anywhere
			strLine = DAnsiStr("[") + section + DAnsiStr("]");
			m_content.push_back(strLine);
			m_content.push_back(strToAdd);
		}

		return true;
	}

	//====================================================================================
	bool CAppConfig::readLine(DAnsiStr section, DAnsiStr variable, DAnsiStr& strLine)
	{
		int startPos = hasSection(section);
		if(startPos < 0) return false;

		//Record file pos
        for(int i=startPos+1; i < (int)m_content.size(); i++)
		{
			strLine = m_content[i];

			//If it is a comment then ignore
			if(strLine.firstChar() == '#')
				continue;


			//Is section?
			if((strLine.firstChar() == L'[')&&(strLine.lastChar() == L']'))
			{
				return false;
			}
			else
			{
                for(int iChar=0; iChar < strLine.length(); iChar++)
				{
					if(strLine[iChar] == L'=')
					{
						DAnsiStr str = strLine.substr(0, iChar);
						if( str.toUpper() == variable.toUpper() )
						{
							return true;
						}
						else
							break;
					}
				}
			}

		}

		return false;
	}

	//====================================================================================
	bool CAppConfig::readValue(DAnsiStr section, DAnsiStr variable, DAnsiStr& strValue)
	{
		DAnsiStr strLine = "";
		if(readLine(section, variable, strLine))
		{
            int len = strLine.length();
            for(int i=0; i < len; i++)
			{
				if(strLine[i] == L'=')
				{
					strValue = strLine.substr(i+1);
					return true;
				}
			}

		}
		else
		{
			DAnsiStr strFileTitle = ExtractFileName(m_strFileName);
			DAnsiStr strMsg = printToAStr("CAppConfig tried to read [file: %s; section: %s; variable: %s] which is not found.",
				strFileTitle.ptr(),
				section.ptr(),
				variable.ptr());
			ReportError(strMsg.ptr());
		}
		return false;
	}
	//====================================================================================
	int CAppConfig::readInt(DAnsiStr section, DAnsiStr variable, int def)
	{
		DAnsiStr strVal;
		if(readValue(section, variable, strVal))
			//def = atoi(strVal.c_str());
			def = atoi(strVal.ptr());
		return def;
	}

	//====================================================================================
	void CAppConfig::writeInt(DAnsiStr section, DAnsiStr variable, int val)
	{
		writeValue(section, variable, printToAStr("%d", val));
	}
	//====================================================================================
	float CAppConfig::readFloat(DAnsiStr section, DAnsiStr variable, float def)
	{
		DAnsiStr strVal;
		if(readValue(section, variable, strVal))
			//def = static_cast<float>(atof(strVal.c_str()));
			def = static_cast<float>(atof(strVal.ptr()));
		return def;

	}
	//====================================================================================
	void CAppConfig::writeFloat(DAnsiStr section, DAnsiStr variable, float val)
	{
		writeValue(section, variable, printToAStr("%f", val));
	}
	//====================================================================================
	double CAppConfig::readDouble(DAnsiStr section, DAnsiStr variable, double def)
	{
		DAnsiStr strVal;
		if(readValue(section, variable, strVal))
			//def = atof(strVal.c_str());
			def = atof(strVal.ptr());
		return def;

	}
	//====================================================================================
	void CAppConfig::writeDouble(DAnsiStr section, DAnsiStr variable, double val)
	{
		writeValue(section, variable, printToAStr("%f", val));
	}
	//====================================================================================
	DAnsiStr CAppConfig::readString(DAnsiStr section, DAnsiStr variable, DAnsiStr def)
	{
		DAnsiStr strVal;
		if(readValue(section, variable, strVal))
			def = strVal;
		return def;
	}

	//====================================================================================
	void CAppConfig::writeString(DAnsiStr section, DAnsiStr variable, DAnsiStr val)
	{
		writeValue(section, variable, val);
	}

	//====================================================================================
	bool CAppConfig::readBool(DAnsiStr section, DAnsiStr variable, bool def)
	{
		DAnsiStr strVal;
		if(readValue(section, variable, strVal))
			//def = (atoi(strVal.c_str()) == 1)?true:false;
			def = (atoi(strVal.ptr()) == 1)?true:false;
		return def;

	}

	//====================================================================================
	void CAppConfig::writeBool(DAnsiStr section, DAnsiStr variable, bool val)
	{
		if(val)
			writeValue(section, variable, "1");
		else
			writeValue(section, variable, "0");
	}

	//====================================================================================
	bool CAppConfig::readIntArray(DAnsiStr section, DAnsiStr variable, int ctExpected, std::vector<int>& arrayInt)
	{
		DAnsiStr strVal;
		if(readValue(section, variable, strVal))
		{
            int pos;
			int iComp = 0;
			DAnsiStr strTemp;
			if(strVal.firstChar() == '(')
				strVal = strVal.substr(1);
			else
				return false;
			while(strVal.lfind(',', pos))
			{
				strTemp = strVal.substr(0, pos);
				strVal = strVal.substr(pos + 1);
				strVal.removeStartEndSpaces();
				arrayInt.push_back(atoi(strTemp.ptr()));
				iComp++;
			}

			if(strVal.length() >= 1)
			{
				if(strVal.lastChar() == ')')
				{
					strTemp = strVal.substr(0, strVal.length() - 1);
					strTemp.removeStartEndSpaces();
					if(strTemp.length() > 0)
						arrayInt.push_back(atoi(strTemp.ptr()));
				}
			}
		}
		return ((int)arrayInt.size() == ctExpected);
	}

	//====================================================================================
	int CAppConfig::writeIntArray(DAnsiStr section, DAnsiStr variable, const std::vector<int>& arrayInt)
	{
		DAnsiStr strValue, strTemp;
		if(arrayInt.size() > 1)
		{
            for(int i=0; i<(int)arrayInt.size(); i++)
			{
				if(i == 0)
					strTemp = printToAStr("(%d, ", arrayInt[i]);
                else if(i == (int)arrayInt.size() - 1)
					strTemp = printToAStr("%d)", arrayInt[i]);
				else
					strTemp = printToAStr("%d, ", arrayInt[i]);
				strValue += strTemp;
			}
			writeValue(section, variable, strValue);
		}
		else if(arrayInt.size() == 1)
		{
			strTemp = printToAStr("(%d)", arrayInt[0]);
			writeValue(section, variable, strTemp);
		}
		else
			writeValue(section, variable, DAnsiStr("()"));
		return arrayInt.size();
	}
	

	void CAppConfig::clearContentBuffer()
	{
		m_content.resize(0);
	}

	void CAppConfig::getContentBuffer( std::vector<DAnsiStr> outContentBuf ) const
	{
		outContentBuf.assign(m_content.begin(), m_content.end());
	}

	void CAppConfig::setContentBuffer( const std::vector<DAnsiStr>& inContentBuf )
	{
		m_content.assign(inContentBuf.begin(), inContentBuf.end());
	}
}
