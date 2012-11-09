#pragma once
#ifndef PS_SKETCHCONFIG_H
#define PS_SKETCHCONFIG_H

#include <vector>
#include "../PS_Base/PS_AppConfig.h"
#include "PS_Vector.h"

using namespace PS::MATH;
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

		//Read and Write Vectors
		vec2f readVec2f(DAnsiStr section, DAnsiStr variable);
		void writeVec2f(DAnsiStr section, DAnsiStr variable, const vec2f& val);

		vec3f readVec3f(DAnsiStr section, DAnsiStr variable);
		void writeVec3f(DAnsiStr section, DAnsiStr variable, const vec3f& val);

		vec4f readVec4f(DAnsiStr section, DAnsiStr variable);
		void writeVec4f(DAnsiStr section, DAnsiStr variable, const vec4f& val);
	};


}
#endif
