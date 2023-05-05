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

#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/float.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/trapezoidal_map.h"

//namespace Microsoft
//{
//    namespace VisualStudio
//    {
//        namespace CppUnitTestFramework
//        {
//            template<> static std::wstring ToString<Segment>(const class Segment& t) { return L"Segment"; }
//            template<> static std::wstring ToString<Segment>(const class Segment* t) { return L"Segment"; }
//            template<> static std::wstring ToString<Segment>(class Segment* t) { return L"Segment"; }
//            template<> static std::wstring ToString<Vec2>(const class Vec2& t) { return L"Vec2"; }
//            template<> static std::wstring ToString<Vec2>(const class Vec2* t) { return L"Vec2"; }
//            template<> static std::wstring ToString<Vec2>(class Vec2* t) { return L"Vec2"; }
//            template<> static std::wstring ToString<Float>(class Float* t) { return L"Float"; }
//            template<> static std::wstring ToString<Float>(const class Float& t) { return L"Float"; }
//            template<> static std::wstring ToString<Trapezoidal_Leaf_Node>(class Trapezoidal_Leaf_Node* t) { return L"Trapezoidal_Leaf_Node"; }
//            template<> static std::wstring ToString<Trapezoidal_Leaf_Node>(const class Trapezoidal_Leaf_Node* t) { return L"const Trapezoidal_Leaf_Node"; }
//            template<> static std::wstring ToString<Trapezoidal_Leaf_Node>(const class Trapezoidal_Leaf_Node& t) { return L"const Trapezoidal_Leaf_Node"; }
//        }
//    }
//}

#endif //PCH_H

