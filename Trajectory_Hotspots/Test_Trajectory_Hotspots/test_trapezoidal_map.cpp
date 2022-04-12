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
            template<> static std::wstring ToString<Trapezoidal_Leaf_Node>(class Trapezoidal_Leaf_Node* t) { return L"Trapezoidal_Leaf_Node"; }
            template<> static std::wstring ToString<Trapezoidal_Leaf_Node>(const class Trapezoidal_Leaf_Node* t) { return L"Trapezoidal_Leaf_Node"; }
            template<> static std::wstring ToString<Trapezoidal_Leaf_Node>(const class Trapezoidal_Leaf_Node& t) { return L"Trapezoidal_Leaf_Node"; }

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

            //Fully embedded
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
            Assert::AreEqual(segment_a.get_bottom_point(), query_result_bottom->top_point);

            //New top point overlapping
            Segment segment_b(Vec2(1.5f, 2.5f), Vec2(1.f, 2.f), 0.f);
            trapezoidal_map.add_segment(segment_b);

            Trapezoidal_Leaf_Node* query_result_bottom_left = trapezoidal_map.query_point(Vec2(.5f, 2.1f));
            Trapezoidal_Leaf_Node* query_result_bottom_right = trapezoidal_map.query_point(Vec2(4.5f, 2.2f));
            Trapezoidal_Leaf_Node* query_result_bottom_bottom = trapezoidal_map.query_point(Vec2(3.f, 1.99999f));

            Assert::IsNotNull(query_result_bottom_left);
            Assert::IsNotNull(query_result_bottom_right);
            Assert::IsNotNull(query_result_bottom_bottom);

            Assert::AreEqual(query_result_left, query_result_bottom_left->top_left);
            Assert::AreEqual(query_result_right, query_result_bottom_right->top_right);

            Assert::AreEqual(*segment_b.get_top_point(), *query_result_bottom_left->top_point);
            Assert::AreEqual(*segment_b.get_top_point(), *query_result_bottom_right->top_point);
            Assert::AreEqual(*segment_b.get_bottom_point(), *query_result_bottom_bottom->top_point);

            Assert::AreEqual(*segment_b.get_bottom_point(), *query_result_bottom_left->bottom_point);
            Assert::AreEqual(*segment_b.get_bottom_point(), *query_result_bottom_right->bottom_point);
            Assert::AreEqual(trapezoidal_map.bottom_point, *query_result_bottom_bottom->bottom_point);

            Assert::IsNull(query_result_bottom_bottom->bottom_left);
            Assert::IsNull(query_result_bottom_bottom->bottom_right);


            //New bottom point overlapping
            Segment segment_c(Vec2(5.f, 7.f), Vec2(7.f, 9.f), 0.f);
            trapezoidal_map.add_segment(segment_c);



            //Both endpoints overlapping


            //TODO: Check point on segment?
        }
    };
}