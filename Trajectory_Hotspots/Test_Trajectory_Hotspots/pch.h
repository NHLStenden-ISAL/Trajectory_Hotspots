// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef PCH_H
#define PCH_H

#include "CppUnitTest.h"
#include <stdint.h>
#include <math.h>
#include <limits>
#include <iostream>
#include <vector>
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace Microsoft
{
	namespace VisualStudio
	{
		namespace CppUnitTestFramework
		{
			template<> static std::wstring ToString<Segment>(const class Segment& t) { return L"Segment"; }
			template<> static std::wstring ToString<Segment>(const class Segment* t) { return L"Segment"; }
			template<> static std::wstring ToString<std::vector<Vec2>>(const class std::vector<Vec2>& t) { return L"Vec2"; }
			template<> static std::wstring ToString<std::vector<Vec2>>(const class std::vector<Vec2>* t) { return L"Vec2"; }
			template<> static std::wstring ToString<Segment>(class Segment* t) { return L"Segment"; }
			template<> static std::wstring ToString<Vec2>(const class Vec2& t) { return L"Vec2"; }
			template<> static std::wstring ToString<Vec2>(const class Vec2* t) { return L"Vec2"; }
			template<> static std::wstring ToString<Vec2>(class Vec2* t) { return L"Vec2"; }
		}
	}
}

#endif //PCH_H

