#pragma once
#ifndef PS_SKETCHCONFIG_H
#define PS_SKETCHCONFIG_H

#include <vector>
#include "PS_AppConfig.h"
#include "PS_VectorMath.h"
using namespace PS::FUNCTIONALMATH;
using namespace std;

namespace PS{

	class CSketchConfig : public CAppConfig
	{
	public:
		CSketchConfig();

		CSketchConfig(const std::vector<DAnsiStr>& inputContent);

		CSketchConfig(DAnsiStr strFileName, FileMode mode)
		{
			set(strFileName, mode);
		};

		~CSketchConfig()
		{

		}

		svec3f readVec3f(DAnsiStr section, DAnsiStr variable);
		void writeVec3f(DAnsiStr section, DAnsiStr variable, svec3f val);

		svec4f readVec4f(DAnsiStr section, DAnsiStr variable);
		void writeVec4f(DAnsiStr section, DAnsiStr variable, svec4f val);
	};


}
#endif