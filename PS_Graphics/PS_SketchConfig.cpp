#include "PS_SketchConfig.h"

namespace PS{
	CSketchConfig::CSketchConfig()
	{
		m_strFileName = "MEM";
		m_fmode = fmMemoryStream;
	}

	CSketchConfig::CSketchConfig( const std::vector<DAnsiStr>& inputContent )
	{
		m_strFileName = "MEM";
		m_fmode = fmMemoryStream;
		setContentBuffer(inputContent);
	}

	//====================================================================================
	svec3f CSketchConfig::readVec3f(DAnsiStr section, DAnsiStr variable)
	{
		DAnsiStr strVal;
		svec3f res;
		if(readValue(section, variable, strVal))
		{
			float f[4];
			size_t pos;
			int iComp = 0;
			DAnsiStr strTemp;

			if(strVal.firstChar() == '(')
				strVal = strVal.substr(1);
			else
				return res;
			while(strVal.lfind(',', pos))
			{
				strTemp = strVal.substr(0, pos);
				strVal = strVal.substr(pos + 1);
				strVal.removeStartEndSpaces();
				f[iComp] = static_cast<float>(atof(strTemp.ptr()));
				iComp++;
			}

			if(strVal.length() >= 1 && iComp < 3)
			{
				if(strVal.lastChar() == ')')
				{
					strTemp = strVal.substr(0, strVal.length() - 1);
					strTemp.removeStartEndSpaces();
					f[iComp] = static_cast<float>(atof(strTemp.ptr()));
				}
			}

			res = svec3f(f[0], f[1], f[2]);
		}
		return res;
	}

	//====================================================================================
	void CSketchConfig::writeVec3f(DAnsiStr section, DAnsiStr variable, svec3f val)
	{
		DAnsiStr strTemp = printToAStr("(%f, %f, %f)", val.x, val.y, val.z);
		writeValue(section, variable, strTemp);
	}

	//====================================================================================
	svec4f CSketchConfig::readVec4f(DAnsiStr section, DAnsiStr variable)
	{
		DAnsiStr strVal;
		svec4f res;
		if(readValue(section, variable, strVal))
		{
			float f[4];
			size_t pos;
			int iComp = 0;
			DAnsiStr strTemp;

			if(strVal.firstChar() == '(')
				strVal = strVal.substr(1);
			else
				return res;
			while(strVal.lfind(',', pos))
			{
				strTemp = strVal.substr(0, pos);
				strVal = strVal.substr(pos + 1);
				strVal.removeStartEndSpaces();
				f[iComp] = static_cast<float>(atof(strTemp.ptr()));
				iComp++;
			}

			if(strVal.length() >= 1 && iComp < 4)
			{
				if(strVal.lastChar() == ')')
				{
					strTemp = strVal.substr(0, strVal.length() - 1);
					strTemp.removeStartEndSpaces();
					f[iComp] = static_cast<float>(atof(strTemp.ptr()));
				}
			}

			res = svec4f(f[0], f[1], f[2], f[3]);
		}
		return res;
	}

	//====================================================================================
	void CSketchConfig::writeVec4f(DAnsiStr section, DAnsiStr variable, svec4f val)
	{
		DAnsiStr strTemp = printToAStr("(%f, %f, %f, %f)", val.x, val.y, val.z, val.w);
		writeValue(section, variable, strTemp);
	}
	//====================================================================================
}