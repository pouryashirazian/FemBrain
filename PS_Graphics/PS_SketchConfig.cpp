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
	vec2f CSketchConfig::readVec2f(DAnsiStr section, DAnsiStr variable)
	{
		DAnsiStr strVal;
		vec2f res;
		if(readValue(section, variable, strVal))
		{
			float f[2];
            int pos;
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

			if(strVal.length() >= 1 && iComp < 2)
			{
				if(strVal.lastChar() == ')')
				{
					strTemp = strVal.substr(0, strVal.length() - 1);
					strTemp.removeStartEndSpaces();
					f[iComp] = static_cast<float>(atof(strTemp.ptr()));
				}
			}

			res = vec2f(f[0], f[1]);
		}
		return res;
	}

	//====================================================================================
	void CSketchConfig::writeVec2f(DAnsiStr section, DAnsiStr variable, const vec2f& val)
	{
		DAnsiStr strTemp = printToAStr("(%f, %f)", val.x, val.y);
		writeValue(section, variable, strTemp);
	}
	//====================================================================================
	vec3f CSketchConfig::readVec3f(DAnsiStr section, DAnsiStr variable)
	{
		DAnsiStr strVal;
		vec3f res;
		if(readValue(section, variable, strVal))
		{
			float f[4];
            int pos;
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

			res = vec3f(f[0], f[1], f[2]);
		}
		return res;
	}

	//====================================================================================
	void CSketchConfig::writeVec3f(DAnsiStr section, DAnsiStr variable, const vec3f& val)
	{
		DAnsiStr strTemp = printToAStr("(%f, %f, %f)", val.x, val.y, val.z);
		writeValue(section, variable, strTemp);
	}

	//====================================================================================
	vec4f CSketchConfig::readVec4f(DAnsiStr section, DAnsiStr variable)
	{
		DAnsiStr strVal;
		vec4f res;
		if(readValue(section, variable, strVal))
		{
			float f[4];
            int pos;
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

			res = vec4f(f[0], f[1], f[2], f[3]);
		}
		return res;
	}

	//====================================================================================
	void CSketchConfig::writeVec4f(DAnsiStr section, DAnsiStr variable, const vec4f& val)
	{
		DAnsiStr strTemp = printToAStr("(%f, %f, %f, %f)", val.x, val.y, val.z, val.w);
		writeValue(section, variable, strTemp);
	}
	//====================================================================================
}
