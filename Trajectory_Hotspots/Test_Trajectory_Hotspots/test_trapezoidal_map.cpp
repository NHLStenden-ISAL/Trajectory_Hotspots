#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/trapezoidal_map.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;


namespace Microsoft
{
    namespace VisualStudio
    {
        namespace CppUnitTestFramework
        {
            template<> static std::wstring ToString<Segment>(const class Segment& t) { return L"Segment"; }
            template<> static std::wstring ToString<Segment>(const class Segment* t) { return L"Segment"; }
            template<> static std::wstring ToString<Segment>(class Segment* t) { return L"Segment"; }
            template<> static std::wstring ToString<Vec2>(const class Vec2& t) { return L"Vec2"; }
            template<> static std::wstring ToString<Vec2>(const class Vec2* t) { return L"Vec2"; }
            template<> static std::wstring ToString<Vec2>(class Vec2* t) { return L"Vec2"; }
        }
    }
}

namespace TestTrajectoryHotspots
{
    TEST_CLASS(TestTrajectoryHotspotsTrapezoidalMap)
    {
    public:

        TEST_METHOD(default_map_construction)
        {
            Trapezoidal_Map trapezoidal_map;

            Assert::AreEqual(-std::numeric_limits<float>::infinity(), trapezoidal_map.bottom_point.x);
            Assert::AreEqual(-std::numeric_limits<float>::infinity(), trapezoidal_map.bottom_point.y);

            Assert::AreEqual(std::numeric_limits<float>::infinity(), trapezoidal_map.top_point.x);
            Assert::AreEqual(std::numeric_limits<float>::infinity(), trapezoidal_map.top_point.y);
        }

        TEST_METHOD(update_with_simple_case)
        {
            Trapezoidal_Map trapezoidal_map;

            Segment segment_a(Vec2(1.5f, 2.5f), Vec2(5.f, 7.f), 0.f);

            trapezoidal_map.add_segment(segment_a);

            Trapezoidal_Leaf_Node* query_result_top = trapezoidal_map.query_point(Vec2(3.f, 8.f));
            Assert::IsNotNull(query_result_top);

            Assert::AreEqual(segment_a.get_top_point(), query_result_top->bottom_point);

            Trapezoidal_Leaf_Node* query_result_left = trapezoidal_map.query_point(Vec2(3.f, 6.f));
            Assert::IsNotNull(query_result_left);
            Assert::AreEqual(segment_a, *query_result_left->right_segment);

            Trapezoidal_Leaf_Node* query_result_right = trapezoidal_map.query_point(Vec2(5.f, 3.f));
            Assert::IsNotNull(query_result_right);
            Assert::AreEqual(segment_a, *query_result_right->left_segment);


            Trapezoidal_Leaf_Node* query_result_bottom = trapezoidal_map.query_point(Vec2(3.5f, 1.f));
            Assert::IsNotNull(query_result_bottom);
            Assert::AreEqual(segment_a.get_bottom_point(), query_result_top->top_point);
        }
    };
}