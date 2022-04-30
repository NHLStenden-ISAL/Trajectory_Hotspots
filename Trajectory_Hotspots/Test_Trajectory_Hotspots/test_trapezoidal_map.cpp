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
            Vec2 D(2.6952, 3.86184);

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

            Vec2 point_in_top_left(5.72988, 10.43698);
            Vec2 point_in_top_right(9.13724, 10.67656);
            Vec2 point_in_left(4.79818f, 7.16272f);
            Vec2 point_in_right(7.43356f, 6.71018f);
            Vec2 point_in_bottom_left(3.04126, 3.11648);
            Vec2 point_in_bottom_right(5.80974, 3.35606);

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
    };
}