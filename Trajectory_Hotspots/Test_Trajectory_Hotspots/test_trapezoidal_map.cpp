#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/trajectory.h"
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

            Assert::IsTrue(trapezoidal_map.bottom_point.x == -std::numeric_limits<float>::infinity());
            Assert::IsTrue(trapezoidal_map.bottom_point.y == -std::numeric_limits<float>::infinity());

            Assert::IsTrue(trapezoidal_map.top_point.x == std::numeric_limits<float>::infinity());
            Assert::IsTrue(trapezoidal_map.top_point.y == std::numeric_limits<float>::infinity());
        }

        TEST_METHOD(update_with_simple_case_fully_embedded)
        {
            Trapezoidal_Map trapezoidal_map;

            //Fully embedded segment
            Segment segment_a(Vec2(1.5f, 2.5f), Vec2(5.f, 7.f), 0.f);
            trapezoidal_map.add_segment(segment_a);

            Trapezoidal_Leaf_Node* query_result_top = trapezoidal_map.query_point(Vec2(3.f, 8.f));
            Trapezoidal_Leaf_Node* query_result_left = trapezoidal_map.query_point(Vec2(3.f, 6.f));
            Trapezoidal_Leaf_Node* query_result_right = trapezoidal_map.query_point(Vec2(5.f, 3.f));
            Trapezoidal_Leaf_Node* query_result_bottom = trapezoidal_map.query_point(Vec2(3.5f, 1.f));

            Assert::IsNotNull(query_result_top);
            Assert::IsNotNull(query_result_left);
            Assert::IsNotNull(query_result_right);
            Assert::IsNotNull(query_result_bottom);

            ////Top Trapezoid
            //Check neighbours of top trapezoid
            Assert::AreEqual(query_result_left, query_result_top->bottom_left);
            Assert::AreEqual(query_result_right, query_result_top->bottom_right);
            Assert::IsNull(query_result_top->top_left);
            Assert::IsNull(query_result_top->top_right);

            //Check top and bottom points
            Assert::AreEqual(trapezoidal_map.top_point, *query_result_top->top_point);
            Assert::AreEqual(*segment_a.get_top_point(), *query_result_top->bottom_point);

            //Check top left and right segment
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_top->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_top->right_segment);

            ////Bottom trapezoid
            //Check neighbours of bottom trapezoid
            Assert::AreEqual(query_result_left, query_result_bottom->top_left);
            Assert::AreEqual(query_result_right, query_result_bottom->top_right);
            Assert::IsNull(query_result_bottom->bottom_left);
            Assert::IsNull(query_result_bottom->bottom_right);

            //Check bottom trapezoid top and bottom points
            Assert::AreEqual(*segment_a.get_bottom_point(), *query_result_bottom->top_point);
            Assert::AreEqual(trapezoidal_map.bottom_point, *query_result_bottom->bottom_point);

            //Check bottom trapezoid left and right segment
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_bottom->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_bottom->right_segment);

            ////Left trapezoid
            //Check neighbours of left trapezoid
            Assert::AreEqual(query_result_bottom, query_result_left->bottom_left);
            Assert::AreEqual(query_result_top, query_result_left->top_left);
            Assert::IsNull(query_result_left->bottom_right);
            Assert::IsNull(query_result_left->top_right);

            //Check left trapezoid top and bottom points
            Assert::AreEqual(*segment_a.get_bottom_point(), *query_result_left->bottom_point);
            Assert::AreEqual(*segment_a.get_top_point(), *query_result_left->top_point);

            //Check left trapezoid left and right segment
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_left->left_segment);
            Assert::AreEqual(segment_a, *query_result_left->right_segment);

            ////Right trapezoid
            //Check neighbours of right trapezoid
            Assert::AreEqual(query_result_bottom, query_result_right->bottom_right);
            Assert::AreEqual(query_result_top, query_result_right->top_right);
            Assert::IsNull(query_result_right->bottom_left);
            Assert::IsNull(query_result_right->top_left);

            //Check right trapezoid top and bottom points
            Assert::AreEqual(*segment_a.get_bottom_point(), *query_result_right->bottom_point);
            Assert::AreEqual(*segment_a.get_top_point(), *query_result_right->top_point);

            //Check right trapezoid left and right segment
            Assert::AreEqual(segment_a, *query_result_right->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_right->right_segment);

            //TODO: Check point on segment?
        }

        TEST_METHOD(update_with_simple_case_fully_embedded_bottom_overlap)
        {
            Trapezoidal_Map trapezoidal_map;

            //Fully embedded segments
            Segment bottom_segment(Vec2(1.5f, 2.5f), Vec2(1.f, 2.f), 0.f);
            Segment middle_segment(Vec2(1.5f, 2.5f), Vec2(5.f, 7.f), 0.f);
            Segment top_segment(Vec2(6.89635f, 15.48397f), Vec2(7.514f, 16.620f));

            trapezoidal_map.add_segment(bottom_segment);
            trapezoidal_map.add_segment(middle_segment);
            trapezoidal_map.add_segment(top_segment);

            //New segment, bottom point overlapping with top of middle segment
            Segment new_segment(Vec2(5.f, 7.f), Vec2(7.f, 9.f), 0.f);
            trapezoidal_map.add_segment(new_segment);

            Trapezoidal_Leaf_Node* query_result_top_left = trapezoidal_map.query_point(Vec2(6.497f, 16.16206f));
            Trapezoidal_Leaf_Node* query_result_top_right = trapezoidal_map.query_point(Vec2(8.232596f, 16.04239f));
            Trapezoidal_Leaf_Node* query_result_middle_left = trapezoidal_map.query_point(Vec2(3.f, 6.f));
            Trapezoidal_Leaf_Node* query_result_middle_right = trapezoidal_map.query_point(Vec2(5.f, 3.f));

            Trapezoidal_Leaf_Node* query_result_new_left = trapezoidal_map.query_point(Vec2(3.f, 8.f));
            Trapezoidal_Leaf_Node* query_result_new_right = trapezoidal_map.query_point(Vec2(13.432054f, 7.98329f));
            Trapezoidal_Leaf_Node* query_result_new_top = trapezoidal_map.query_point(Vec2(6.92894f, 11.51954f));

            Assert::IsNotNull(query_result_top_left);
            Assert::IsNotNull(query_result_top_right);
            Assert::IsNotNull(query_result_middle_left);
            Assert::IsNotNull(query_result_middle_right);
            Assert::IsNotNull(query_result_new_left);
            Assert::IsNotNull(query_result_new_right);
            Assert::IsNotNull(query_result_new_top);

            //Check top neighbours
            Assert::AreEqual(query_result_new_top, query_result_top_left->bottom_left);
            Assert::IsNull(query_result_top_left->bottom_right);
            Assert::IsNull(query_result_top_right->bottom_left);
            Assert::AreEqual(query_result_new_top, query_result_top_right->bottom_right);

            //Check bottom neighbours
            Assert::AreEqual(query_result_new_left, query_result_middle_left->top_left);
            Assert::IsNull(query_result_middle_left->top_right);
            Assert::IsNull(query_result_middle_right->top_left);
            Assert::AreEqual(query_result_new_right, query_result_middle_right->top_right);

            ////Top Trapezoid
            //Check neighbours of top trapezoid
            Assert::AreEqual(query_result_new_left, query_result_new_top->bottom_left);
            Assert::AreEqual(query_result_new_right, query_result_new_top->bottom_right);
            Assert::AreEqual(query_result_top_left, query_result_new_top->top_left);
            Assert::AreEqual(query_result_top_right, query_result_new_top->top_right);

            //Check top and bottom points
            Assert::AreEqual(*top_segment.get_bottom_point(), *query_result_new_top->top_point);
            Assert::AreEqual(*new_segment.get_top_point(), *query_result_new_top->bottom_point);

            //Check top left and right segment
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_new_top->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_new_top->right_segment);

            ////Left trapezoid
            //Check neighbours of left trapezoid
            Assert::AreEqual(query_result_middle_left, query_result_new_left->bottom_left);
            Assert::AreEqual(query_result_new_top, query_result_new_left->top_left);
            Assert::IsNull(query_result_new_left->bottom_right);
            Assert::IsNull(query_result_new_left->top_right);

            //Check left trapezoid top and bottom points
            Assert::AreEqual(*new_segment.get_bottom_point(), *query_result_new_left->bottom_point);
            Assert::AreEqual(*new_segment.get_top_point(), *query_result_new_left->top_point);

            //Check left trapezoid left and right segment
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_new_left->left_segment);
            Assert::AreEqual(new_segment, *query_result_new_left->right_segment);

            ////Right trapezoid
            //Check neighbours of right trapezoid
            Assert::AreEqual(query_result_middle_right, query_result_new_right->bottom_right);
            Assert::AreEqual(query_result_new_top, query_result_new_right->top_right);
            Assert::IsNull(query_result_new_right->bottom_left);
            Assert::IsNull(query_result_new_right->top_left);

            //Check right trapezoid top and bottom points
            Assert::AreEqual(*new_segment.get_bottom_point(), *query_result_new_right->bottom_point);
            Assert::AreEqual(*new_segment.get_top_point(), *query_result_new_right->top_point);

            //Check right trapezoid left and right segment
            Assert::AreEqual(new_segment, *query_result_new_right->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_new_right->right_segment);
        }

        TEST_METHOD(update_with_simple_case_fully_embedded_top_overlap)
        {
            Trapezoidal_Map trapezoidal_map;

            //Fully embedded segment
            Segment segment_a(Vec2(1.5f, 2.5f), Vec2(5.f, 7.f), 0.f);
            trapezoidal_map.add_segment(segment_a);

            Trapezoidal_Leaf_Node* query_result_top = trapezoidal_map.query_point(Vec2(3.f, 8.f));
            Trapezoidal_Leaf_Node* query_result_left = trapezoidal_map.query_point(Vec2(3.f, 6.f));
            Trapezoidal_Leaf_Node* query_result_right = trapezoidal_map.query_point(Vec2(5.f, 3.f));
            Trapezoidal_Leaf_Node* query_result_bottom = trapezoidal_map.query_point(Vec2(3.5f, 1.f));

            Assert::IsNotNull(query_result_top);
            Assert::IsNotNull(query_result_left);
            Assert::IsNotNull(query_result_right);
            Assert::IsNotNull(query_result_bottom);

            //New segment with top point overlapping
            Segment segment_b(Vec2(1.5f, 2.5f), Vec2(1.f, 2.f), 0.f);
            trapezoidal_map.add_segment(segment_b);

            Trapezoidal_Leaf_Node* query_result_bottom_left = trapezoidal_map.query_point(Vec2(.5f, 2.1f));
            Trapezoidal_Leaf_Node* query_result_bottom_right = trapezoidal_map.query_point(Vec2(4.5f, 2.2f));
            Trapezoidal_Leaf_Node* query_result_bottom_bottom = trapezoidal_map.query_point(Vec2(3.f, 1.99999f));

            Assert::IsNotNull(query_result_bottom_left);
            Assert::IsNotNull(query_result_bottom_right);
            Assert::IsNotNull(query_result_bottom_bottom);

            ////Bottom Trapezoid
            //Check neighbours of bottom trapezoid
            Assert::AreEqual(query_result_bottom_left, query_result_bottom_bottom->top_left);
            Assert::AreEqual(query_result_bottom_right, query_result_bottom_bottom->top_right);
            Assert::IsNull(query_result_bottom_bottom->bottom_left);
            Assert::IsNull(query_result_bottom_bottom->bottom_right);

            //Check bottom trapezoid top and bottom points
            Assert::AreEqual(*segment_b.get_bottom_point(), *query_result_bottom_bottom->top_point);
            Assert::AreEqual(trapezoidal_map.bottom_point, *query_result_bottom_bottom->bottom_point);

            //Check bottom trapezoid left and right segment
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_bottom_bottom->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_bottom_bottom->right_segment);

            ////Left Trapezoid
            //Check neighbours of left trapezoid
            Assert::AreEqual(query_result_bottom_bottom, query_result_bottom_left->bottom_left);
            Assert::AreEqual(query_result_left, query_result_bottom_left->top_left);
            Assert::IsNull(query_result_bottom_left->bottom_right);
            Assert::IsNull(query_result_bottom_left->top_right);

            //Check left trapezoid top and bottom points
            Assert::AreEqual(*segment_b.get_top_point(), *query_result_bottom_left->top_point);
            Assert::AreEqual(*segment_b.get_bottom_point(), *query_result_bottom_left->bottom_point);

            //Check left trapezoid left and right segment
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_bottom_left->left_segment);
            Assert::AreEqual(segment_b, *query_result_bottom_left->right_segment);

            ////Right Trapezoid
            //Check neighbours of left trapezoid
            Assert::AreEqual(query_result_bottom_bottom, query_result_bottom_right->bottom_right);
            Assert::AreEqual(query_result_right, query_result_bottom_right->top_right);
            Assert::IsNull(query_result_bottom_right->bottom_left);
            Assert::IsNull(query_result_bottom_right->top_left);

            //Check left trapezoid top and bottom points
            Assert::AreEqual(*segment_b.get_top_point(), *query_result_bottom_right->top_point);
            Assert::AreEqual(*segment_b.get_bottom_point(), *query_result_bottom_right->bottom_point);

            //Check left trapezoid left and right segment
            Assert::AreEqual(segment_b, *query_result_bottom_right->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_bottom_right->right_segment);
        }

        TEST_METHOD(update_with_simple_case_fully_embedded_both_overlap)
        {
            Trapezoidal_Map trapezoidal_map;

            Vec2 A(5.43706f, 11.5284f);
            Vec2 B(8.5516f, 9.47866f);
            Vec2 C(5.14424f, 2.4776f);
            Vec2 D(2.6952f, 3.86184f);

            Vec2 E(12.86404f, 12.64644f);
            Vec2 F(9.80274f, 1.1466f);
            Vec2 G(2.93478f, 14.16378f);
            Vec2 H(1.098f, 0.13504f);

            Segment top_segment(A, B);
            Segment bottom_segment(D, C);
            Segment left_segment(G, H);
            Segment right_segment(E, F);
            Segment middle_segment(B, D);

            // |     
            // |      |
            // |  \   |
            // |  /   |
            // |  \   |
            // |      |
            // |

            //All of these inserts are fully embedded without overlaps
            trapezoidal_map.add_segment(left_segment);
            trapezoidal_map.add_segment(right_segment);
            trapezoidal_map.add_segment(top_segment);
            trapezoidal_map.add_segment(bottom_segment);

            //Insert segment with both endpoints overlapping
            trapezoidal_map.add_segment(middle_segment);

            Assert::AreEqual(5, trapezoidal_map.segment_count);

            Vec2 point_in_top_left(5.72988f, 10.43698f);
            Vec2 point_in_top_right(9.13724f, 10.67656f);
            Vec2 point_in_left(4.79818f, 7.16272f);
            Vec2 point_in_right(7.43356f, 6.71018f);
            Vec2 point_in_bottom_left(3.04126f, 3.11648f);
            Vec2 point_in_bottom_right(5.80974f, 3.35606f);

            Trapezoidal_Leaf_Node* query_result_top_left = trapezoidal_map.query_point(point_in_top_left);
            Trapezoidal_Leaf_Node* query_result_top_right = trapezoidal_map.query_point(point_in_top_right);
            Trapezoidal_Leaf_Node* query_result_left = trapezoidal_map.query_point(point_in_left);
            Trapezoidal_Leaf_Node* query_result_right = trapezoidal_map.query_point(point_in_right);
            Trapezoidal_Leaf_Node* query_result_bottom_left = trapezoidal_map.query_point(point_in_bottom_left);
            Trapezoidal_Leaf_Node* query_result_bottom_right = trapezoidal_map.query_point(point_in_bottom_right);

            Assert::IsNotNull(query_result_top_left);
            Assert::IsNotNull(query_result_top_right);
            Assert::IsNotNull(query_result_left);
            Assert::IsNotNull(query_result_right);
            Assert::IsNotNull(query_result_bottom_left);
            Assert::IsNotNull(query_result_bottom_right);

            ////Left Trapezoid
            //Check neighbours of left trapezoid
            Assert::AreEqual(query_result_bottom_left, query_result_left->bottom_left);
            Assert::AreEqual(query_result_top_left, query_result_left->top_left);
            Assert::IsNull(query_result_left->bottom_right);
            Assert::IsNull(query_result_left->top_right);

            //Check left trapezoid top and bottom points
            Assert::AreEqual(*middle_segment.get_top_point(), *query_result_left->top_point);
            Assert::AreEqual(*middle_segment.get_bottom_point(), *query_result_left->bottom_point);

            //Check left trapezoid left and right segment
            Assert::AreEqual(left_segment, *query_result_left->left_segment);
            Assert::AreEqual(middle_segment, *query_result_left->right_segment);

            ////Right Trapezoid
            //Check neighbours of left trapezoid
            Assert::AreEqual(query_result_bottom_right, query_result_right->bottom_right);
            Assert::AreEqual(query_result_top_right, query_result_right->top_right);
            Assert::IsNull(query_result_right->bottom_left);
            Assert::IsNull(query_result_right->top_left);

            //Check left trapezoid top and bottom points
            Assert::AreEqual(*middle_segment.get_top_point(), *query_result_right->top_point);
            Assert::AreEqual(*middle_segment.get_bottom_point(), *query_result_right->bottom_point);

            //Check left trapezoid left and right segment
            Assert::AreEqual(middle_segment, *query_result_right->left_segment);
            Assert::AreEqual(right_segment, *query_result_right->right_segment);

            //Check Neighbour pointers
            Assert::IsNull(query_result_top_left->bottom_right);
            Assert::IsNull(query_result_top_right->bottom_left);
            Assert::IsNull(query_result_bottom_left->top_right);
            Assert::IsNull(query_result_bottom_right->top_left);

            Assert::AreEqual(query_result_left, query_result_top_left->bottom_left);
            Assert::AreEqual(query_result_right, query_result_top_right->bottom_right);
            Assert::AreEqual(query_result_left, query_result_bottom_left->top_left);
            Assert::AreEqual(query_result_right, query_result_bottom_right->top_right);
        }

        TEST_METHOD(update_with_segment_overlapping_two_trapezoids)
        {
            Trapezoidal_Map trapezoidal_map;

            Vec2 right_segment_top(236.223876199f, 102.783533699f);
            Vec2 right_segment_bottom(230.249842158f, 52.7903014615f);
            Vec2 left_segment_top(210.441202969f, 121.334481511f);
            Vec2 left_segment_bottom(210.755625813f, 86.4335457980f);

            Segment right_segment(right_segment_top, right_segment_bottom);
            Segment left_segment(left_segment_top, left_segment_bottom);

            trapezoidal_map.add_segment(right_segment);
            trapezoidal_map.add_segment(left_segment);

            //  |
            //  |   /
            //     /

            Vec2 point_in_top(213.5523f, 134.67516f);
            Vec2 point_left_of_left(194.7737f, 107.30614f);
            Vec2 point_above_right(233.1300f, 111.50208f);
            Vec2 point_between_left_and_right(225.3389f, 96.31938f);
            Vec2 point_below_left(214.5512f, 72.1469f);
            Vec2 point_right_of_right(248.5124f, 77.14129f);
            Vec2 point_in_bottom(225.3389f, 42.58063f);

            Trapezoidal_Leaf_Node* query_result_top = trapezoidal_map.query_point(point_in_top);//This is returning above_right?
            Trapezoidal_Leaf_Node* query_result_left_of_left = trapezoidal_map.query_point(point_left_of_left);
            Trapezoidal_Leaf_Node* query_result_above_right = trapezoidal_map.query_point(point_above_right);
            Trapezoidal_Leaf_Node* query_result_between_left_and_right = trapezoidal_map.query_point(point_between_left_and_right);
            Trapezoidal_Leaf_Node* query_result_below_left = trapezoidal_map.query_point(point_below_left);
            Trapezoidal_Leaf_Node* query_result_right_of_right = trapezoidal_map.query_point(point_right_of_right);
            Trapezoidal_Leaf_Node* query_result_bottom = trapezoidal_map.query_point(point_in_bottom);

            Assert::IsNotNull(query_result_top);
            Assert::IsNotNull(query_result_left_of_left);
            Assert::IsNotNull(query_result_above_right);
            Assert::IsNotNull(query_result_between_left_and_right);
            Assert::IsNotNull(query_result_below_left);
            Assert::IsNotNull(query_result_right_of_right);
            Assert::IsNotNull(query_result_bottom);

            Assert::AreEqual(query_result_left_of_left, query_result_top->bottom_left);
            Assert::AreEqual(query_result_above_right, query_result_top->bottom_right);
            Assert::IsNull(query_result_top->top_left);
            Assert::IsNull(query_result_top->top_right);
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_top->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_top->right_segment);
            Assert::AreEqual(*left_segment.get_top_point(), *query_result_top->bottom_point);
            Assert::AreEqual(trapezoidal_map.top_point, *query_result_top->top_point);

            Assert::AreEqual(query_result_below_left, query_result_left_of_left->bottom_left);
            Assert::IsNull(query_result_left_of_left->bottom_right);
            Assert::AreEqual(query_result_top, query_result_left_of_left->top_left);
            Assert::IsNull(query_result_left_of_left->top_right);
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_left_of_left->left_segment);
            Assert::AreEqual(left_segment, *query_result_left_of_left->right_segment);
            Assert::AreEqual(*left_segment.get_bottom_point(), *query_result_left_of_left->bottom_point);
            Assert::AreEqual(*left_segment.get_top_point(), *query_result_left_of_left->top_point);

            Assert::AreEqual(query_result_between_left_and_right, query_result_above_right->bottom_left);
            Assert::AreEqual(query_result_right_of_right, query_result_above_right->bottom_right);
            Assert::IsNull(query_result_above_right->top_left);
            Assert::AreEqual(query_result_top, query_result_above_right->top_right);
            Assert::AreEqual(left_segment, *query_result_above_right->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_above_right->right_segment);
            Assert::AreEqual(*right_segment.get_top_point(), *query_result_above_right->bottom_point);
            Assert::AreEqual(*left_segment.get_top_point(), *query_result_above_right->top_point);

            Assert::IsNull(query_result_between_left_and_right->bottom_left);
            Assert::AreEqual(query_result_below_left, query_result_between_left_and_right->bottom_right);
            Assert::AreEqual(query_result_above_right, query_result_between_left_and_right->top_left);
            Assert::IsNull(query_result_between_left_and_right->top_right);
            Assert::AreEqual(left_segment, *query_result_between_left_and_right->left_segment);
            Assert::AreEqual(right_segment, *query_result_between_left_and_right->right_segment);
            Assert::AreEqual(*left_segment.get_bottom_point(), *query_result_between_left_and_right->bottom_point);
            Assert::AreEqual(*right_segment.get_top_point(), *query_result_between_left_and_right->top_point);

            Assert::AreEqual(query_result_bottom, query_result_below_left->bottom_left);
            Assert::IsNull(query_result_below_left->bottom_right);
            Assert::AreEqual(query_result_left_of_left, query_result_below_left->top_left);
            Assert::AreEqual(query_result_between_left_and_right, query_result_below_left->top_right);
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_below_left->left_segment);
            Assert::AreEqual(right_segment, *query_result_below_left->right_segment);
            Assert::AreEqual(*right_segment.get_bottom_point(), *query_result_below_left->bottom_point);
            Assert::AreEqual(*left_segment.get_bottom_point(), *query_result_below_left->top_point);

            Assert::IsNull(query_result_right_of_right->bottom_left);
            Assert::AreEqual(query_result_bottom, query_result_right_of_right->bottom_right);
            Assert::IsNull(query_result_right_of_right->top_left);
            Assert::AreEqual(query_result_above_right, query_result_right_of_right->top_right);
            Assert::AreEqual(right_segment, *query_result_right_of_right->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_right_of_right->right_segment);
            Assert::AreEqual(*right_segment.get_bottom_point(), *query_result_right_of_right->bottom_point);
            Assert::AreEqual(*right_segment.get_top_point(), *query_result_right_of_right->top_point);

            Assert::IsNull(query_result_bottom->bottom_left);
            Assert::IsNull(query_result_bottom->bottom_right);
            Assert::AreEqual(query_result_below_left, query_result_bottom->top_left);
            Assert::AreEqual(query_result_right_of_right, query_result_bottom->top_right);
            Assert::AreEqual(trapezoidal_map.left_border, *query_result_bottom->left_segment);
            Assert::AreEqual(trapezoidal_map.right_border, *query_result_bottom->right_segment);
            Assert::AreEqual(trapezoidal_map.bottom_point, *query_result_bottom->bottom_point);
            Assert::AreEqual(*right_segment.get_bottom_point(), *query_result_bottom->top_point);
        }

        TEST_METHOD(update_with_segment_overlapping_eight_trapezoids)
        {
            Vec2 point_A(7.09263f, 11.57507612f);
            Vec2 point_B(8.54463f, 15.88498099f);
            Vec2 point_C(9.97358f, 9.47774285f);
            Vec2 point_D(11.7021580952381f, 7.2421238095238f);
            Vec2 point_E(14.97492f, 23.7211714285714f);
            Vec2 point_F(15.4819676190476f, 18.9733619047619f);
            Vec2 point_G(16.726539047619f, 11.4137428571429f);
            Vec2 point_H(19.3539676190476f, 21.2550761904762f);

            Vec2 point_I(4.8472236875924f, 22.3692090783514f);
            Vec2 point_J(4.2599429240947f, 25.4901868500819f);
            Vec2 point_K(23.0697068064061f, 22.5370035822079f);
            Vec2 point_L(25.217476455769f, 26.0439087128083f);

            std::vector<Segment> segments
            {
                Segment{point_A, point_B}, //0
                Segment{point_D, point_E}, //1
                Segment{point_B, point_C}, //2
                Segment{point_C, point_D}, //3
                Segment{point_F, point_G}, //4
                Segment{point_G, point_H}, //5
                Segment{point_I, point_J}, //6
                Segment{point_K, point_L}  //7
            };

            //seed 9999 = 6,5,3,0,2,4,7,1
            Trapezoidal_Map trapezoidal_map(segments, 9999);

            Vec2 point_left3(9.3993354f, 17.4046591f);
            Vec2 point_left4(8.602549f, 22.0014889f);

            Trapezoidal_Leaf_Node* query_result_left3 = trapezoidal_map.query_point(point_left3);
            Trapezoidal_Leaf_Node* query_result_left4 = trapezoidal_map.query_point(point_left4);

            Assert::AreEqual(query_result_left3, query_result_left4);

            Vec2 point_right1(14.6397562935346f, 17.2513922958021f);
            Vec2 point_right2(14.7010478200373f, 13.941649864651f);
            Trapezoidal_Leaf_Node* query_result_right1 = trapezoidal_map.query_point(point_right1);
            Trapezoidal_Leaf_Node* query_result_right2 = trapezoidal_map.query_point(point_right2);

            Assert::AreEqual(query_result_right1, query_result_right2);

            Vec2 point_bottom_right1(16.6543384465475f, 8.7611657163598f);
            Vec2 point_bottom_right2(16.8552329077f, 10.2009093546193f);

            Trapezoidal_Leaf_Node* query_result_bottom_right1 = trapezoidal_map.query_point(point_bottom_right1);
            Trapezoidal_Leaf_Node* query_result_bottom_right2 = trapezoidal_map.query_point(point_bottom_right2);

            Assert::AreEqual(query_result_bottom_right1, query_result_bottom_right2);

            Vec2 point_between_0_2(8.326830f, 12.89602947f);
            Trapezoidal_Leaf_Node* query_result_between_0_2 = trapezoidal_map.query_point(point_between_0_2);

            Assert::IsNull(query_result_between_0_2->top_left);
            Assert::IsNull(query_result_between_0_2->top_right);
            Assert::IsNull(query_result_between_0_2->bottom_left);

            Vec2 point_between_4_5(17.17672f, 17.97505f);
            Vec2 point_in_right_3(16.f, 20.f);

            Trapezoidal_Leaf_Node* query_result_between_4_5 = trapezoidal_map.query_point(point_between_4_5);
            Trapezoidal_Leaf_Node* query_result_right_3 = trapezoidal_map.query_point(point_in_right_3);

            Assert::AreEqual(query_result_right_3->bottom_right, query_result_between_4_5);
            Assert::AreEqual(query_result_right_3->bottom_left, query_result_right2);

            Assert::IsNull(query_result_between_4_5->bottom_left);
            Assert::IsNull(query_result_between_4_5->bottom_right);
            Assert::IsNull(query_result_between_4_5->top_left);
            Assert::AreEqual(query_result_right_3, query_result_between_4_5->top_right);

            Vec2 point_in_bottomleft(11.3394f, 8.840f);
            Vec2 point_in_left_1(11.1454797f, 10.337f);
            Vec2 point_in_left_2(10.89f, 13.966f);

            Trapezoidal_Leaf_Node* query_result_bottomleft = trapezoidal_map.query_point(point_in_bottomleft);
            Trapezoidal_Leaf_Node* query_result_left_1 = trapezoidal_map.query_point(point_in_left_1);
            Trapezoidal_Leaf_Node* query_result_left_2 = trapezoidal_map.query_point(point_in_left_2);


            Assert::AreEqual(query_result_left_1, query_result_left_2);

            Assert::AreEqual(query_result_bottomleft, query_result_left_1->bottom_right);

            Assert::AreEqual(query_result_left_1, query_result_bottomleft->top_right);
            Assert::IsNull(query_result_bottomleft->top_left);
            Assert::IsNull(query_result_bottomleft->bottom_left);
            Assert::IsNull(query_result_bottomleft->bottom_right);

            Vec2 point_left_of_0(4.7156966773694f, 14.0466823528248f);
            Vec2 point_left_of_6(2.888396f, 23.7755f);
            Vec2 point_right_of_7(28.32464f, 24.3020f);

            Trapezoidal_Leaf_Node* query_result_left_of_0 = trapezoidal_map.query_point(point_left_of_0);
            Trapezoidal_Leaf_Node* query_result_left_of_6 = trapezoidal_map.query_point(point_left_of_6);
            Trapezoidal_Leaf_Node* query_result_right_of_7 = trapezoidal_map.query_point(point_right_of_7);

            Assert::AreEqual(query_result_left_of_6, query_result_left4->top_left);
            Assert::AreEqual(query_result_left_of_0, query_result_left3->bottom_left);
            Assert::AreEqual(query_result_between_0_2->bottom_right, query_result_left_of_0->bottom_left);


            Vec2 right_4(17.7484326595587f, 21.9502308689088f);
            Vec2 right_5(20.6006216641344f, 22.4612518933329f);
            Vec2 right_of_5(21.16f, 17.569f);

            Trapezoidal_Leaf_Node* query_result_right_4 = trapezoidal_map.query_point(right_4);
            Trapezoidal_Leaf_Node* query_result_right_5 = trapezoidal_map.query_point(right_5);
            Trapezoidal_Leaf_Node* query_result_right_of_5 = trapezoidal_map.query_point(right_of_5);


            Assert::AreEqual(query_result_right_4, query_result_right_5);
            Assert::AreEqual(query_result_right_of_5, query_result_right_4->bottom_right);
            Assert::AreEqual(query_result_right_of_7, query_result_right_5->top_right);
        }

        TEST_METHOD(left_right_trace_in_trapezoid)
        {

            const std::vector<Vec2> trajectory_points
            {
                Vec2(4.f, 4.f),
                Vec2(6.f, 8.f),
                Vec2(8.f, 3.f),
                Vec2(10.f, 6.f),
                Vec2(12.f, 2.f),
                Vec2(16.f, 9.f)
            };

            Trajectory trajectory(trajectory_points);

            Trapezoidal_Map trapezoidal_map(trajectory.get_ordered_trajectory_segments());

            Vec2 trace_point(9.f, 7.f);

            const Segment* left_segment;
            const Segment* right_segment;

            trapezoidal_map.trace_left_right(trace_point, true, left_segment, right_segment);

            Assert::AreEqual(trajectory.get_ordered_trajectory_segments()[1], *left_segment);
            Assert::AreEqual(trajectory.get_ordered_trajectory_segments()[4], *right_segment);

            left_segment = nullptr;
            right_segment = nullptr;
            
            trace_point = Vec2(10.f, 4.f);

            trapezoidal_map.trace_left_right(trace_point, true, left_segment, right_segment);

            Assert::AreEqual(trajectory.get_ordered_trajectory_segments()[2], *left_segment);
            Assert::AreEqual(trajectory.get_ordered_trajectory_segments()[3], *right_segment);

            trapezoidal_map.trace_left_right(trace_point, false, left_segment, right_segment);

            Assert::AreEqual(trajectory.get_ordered_trajectory_segments()[2], *left_segment);
            Assert::AreEqual(trajectory.get_ordered_trajectory_segments()[3], *right_segment);
        }

        TEST_METHOD(left_right_trace_in_trapezoid_edge_case_on_point)
        {

            const std::vector<Vec2> trajectory_points
            {
                Vec2(4.f, 4.f),
                Vec2(6.f, 8.f),
                Vec2(8.f, 3.f),
                Vec2(10.f, 6.f),
                Vec2(12.f, 2.f),
                Vec2(16.f, 9.f)
            };

            Trajectory trajectory(trajectory_points);

            Trapezoidal_Map trapezoidal_map(trajectory.get_ordered_trajectory_segments());

            Vec2 trace_point(10.f, 6.f);

            const Segment* left_segment;
            const Segment* right_segment;

            trapezoidal_map.trace_left_right(trace_point, true, left_segment, right_segment);

            Assert::AreEqual(trajectory.get_ordered_trajectory_segments()[1], *left_segment);
            Assert::AreEqual(trajectory.get_ordered_trajectory_segments()[4], *right_segment);
        }




    };
}