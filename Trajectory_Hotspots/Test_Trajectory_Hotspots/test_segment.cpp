#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/float.h"


namespace Microsoft
{
    namespace VisualStudio
    {
        namespace CppUnitTestFramework
        {
            template<> static std::wstring ToString<Float>(class Float* t) { return L"Float"; }
            template<> static std::wstring ToString<Float>(const class Float& t) { return L"Float"; }
            template<> static std::wstring ToString<Vec2>(const class Vec2& t) { return L"Vec2"; }
        }
    }
}

using namespace Microsoft::VisualStudio::CppUnitTestFramework;


namespace TestTrajectoryHotspots
{

    TEST_CLASS(TestTrajectoryHotspotsSegment)
    {
    public:

        TEST_METHOD(x_intersect)
        {
            Vec2 start_point(1.f, 0.5f);
            Vec2 end_point(10.f, 2.f);

            Segment segment(start_point, end_point);

            Float vertical_line = 4.5f;

            Float x_intersect = segment.x_intersect(vertical_line);

            Assert::AreEqual(x_intersect, Float(1.08333333333f));
        }

        TEST_METHOD(y_intersect)
        {
            Vec2 start_point(1.f, 15.0f);
            Vec2 end_point(10.f, 2.f);

            Segment segment(start_point, end_point);

            Float horizontal_line = 8.5f;

            Float y_intersect = segment.y_intersect(horizontal_line);

            Assert::AreEqual(y_intersect, Float(5.5f));
        }

        TEST_METHOD(x_intersect_inf)
        {
            Vec2 start_point(10.f, 0.5f);
            Vec2 end_point(10.f, 2.f);

            Segment segment(start_point, end_point);

            Float vertical_line = 10.f;

            Float x_intersect = segment.x_intersect(vertical_line);

            Assert::IsTrue(x_intersect.is_inf());
        }

        TEST_METHOD(y_intersect_inf)
        {
            Vec2 start_point(1.f, 12.f);
            Vec2 end_point(10.f, 12.f);

            Segment segment(start_point, end_point);

            Float horizontal_line = 12.f;

            Float y_intersect = segment.y_intersect(horizontal_line);

            Assert::IsTrue(y_intersect.is_inf());
        }

        TEST_METHOD(x_intersects)
        {
            Vec2 start_point(-1.f, 0.5f);
            Vec2 end_point(10.f, 2.f);

            Segment segment(start_point, end_point);

            Float vertical_line = -4.5f;

            Float intersection_point;
            bool x_intersects = segment.x_intersects(vertical_line, intersection_point);

            Assert::IsFalse(x_intersects);

            vertical_line = 0.f;

            x_intersects = segment.x_intersects(vertical_line, intersection_point);

            Assert::IsTrue(x_intersects);
            Assert::AreEqual(intersection_point, Float(0.63636363f));

            vertical_line = 140.f;

            x_intersects = segment.x_intersects(vertical_line, intersection_point);

            Assert::IsFalse(x_intersects);
        }

        TEST_METHOD(y_intersects)
        {
            Vec2 start_point(-1.f, 0.5f);
            Vec2 end_point(10.f, -2.f);

            Segment segment(start_point, end_point);

            Float horizontal_line = -10.5f;

            Float intersection_point;
            bool y_intersects = segment.y_intersects(horizontal_line, intersection_point);

            Assert::IsFalse(y_intersects);

            horizontal_line = -1.f;

            y_intersects = segment.y_intersects(horizontal_line, intersection_point);

            Assert::IsTrue(y_intersects);
            Assert::AreEqual(intersection_point, Float(5.6f));

            horizontal_line = 32.f;

            y_intersects = segment.y_intersects(horizontal_line, intersection_point);

            Assert::IsFalse(y_intersects);
        }

        TEST_METHOD(get_time_at_x)
        {
            Segment segment(Vec2(4.f, 10.f), Vec2(30.f, 20.f), 0.f);

            Float x_position = 12.f;

            Float time_at_x = segment.get_time_at_x(x_position);
            Assert::AreEqual(time_at_x, Float(8.5713158628825f));

            Segment segment_with_extra_time(Vec2(4.f, 10.f), Vec2(30.f, 20.f), 10.f);

            time_at_x = segment_with_extra_time.get_time_at_x(x_position);
            Assert::AreEqual(time_at_x, Float(18.5713158628825f));

        }

        TEST_METHOD(get_time_at_y)
        {
            Segment segment(Vec2(-5.f, 25.f), Vec2(35.f, 15.f), 0.f);

            Float y_position = 16.f;

            Float time_at_y = segment.get_time_at_y(y_position);
            Assert::AreEqual(time_at_y, Float(37.1079506305589f));

            Segment segment_with_extra_time(Vec2(-5.f, 25.f), Vec2(35.f, 15.f), 10.f);

            time_at_y = segment_with_extra_time.get_time_at_y(y_position);
            Assert::AreEqual(time_at_y, Float(47.1079506305589f));
        }

        TEST_METHOD(get_time_at_point)
        {
            Segment segment(Vec2(-4.f, -5.f), Vec2(4.f, 23.f), 0.f);

            Vec2 point(2.f, 16.f);

            Float time_at_point = segment.get_time_at_point(point);
            Assert::AreEqual(time_at_point, Float(21.840329667841554813291907474581f));

            Segment segment_with_extra_time(Vec2(-4.f, -5.f), Vec2(4.f, 23.f), 10.f);

            time_at_point = segment_with_extra_time.get_time_at_point(point);
            Assert::AreEqual(time_at_point, Float(31.840329667841554813291907474581f));
        }

        TEST_METHOD(get_point_at_time)
        {
            Segment segment(Vec2(-14.f, -15.f), Vec2(14.f, 23.f), 2.f);

            Vec2 result_point = segment.get_point_at_time(37.401271163617839783622493448623f);
            Assert::AreEqual(result_point, Vec2(7.f, 13.5f));
        }

        TEST_METHOD(get_bottom_point)
        {
            Vec2 top_point(14.f, 23.f);
            Vec2 bottom_point(-14.f, -15.f);

            Segment segment(top_point, bottom_point);
            Assert::AreEqual(*segment.get_bottom_point(), bottom_point);

            Segment segment_reversed(bottom_point, top_point);
            Assert::AreEqual(*segment_reversed.get_bottom_point(), bottom_point);

        }

        TEST_METHOD(get_top_point)
        {
            Vec2 top_point(14.f, 23.f);
            Vec2 bottom_point(-14.f, -15.f);

            Segment segment(top_point, bottom_point);
            Assert::AreEqual(*segment.get_top_point(), top_point);

            Segment segment_reversed(bottom_point, top_point);
            Assert::AreEqual(*segment_reversed.get_top_point(), top_point);
        }

        TEST_METHOD(x_overlap)
        {
            Segment segment(Vec2(12.f, 15.f), Vec2(14.f, 16.f));

            Segment other_segment(Vec2(10.f, 12.f), Vec2(13.f, 23.f));
            Assert::IsTrue(segment.x_overlap(other_segment));

            other_segment = Segment(Vec2(8.f, 12.f), Vec2(16.f, 23.f));
            Assert::IsTrue(segment.x_overlap(other_segment));

            other_segment = Segment(Vec2(14.f, 12.f), Vec2(17.f, 23.f));
            Assert::IsTrue(segment.x_overlap(other_segment));

            other_segment = Segment(Vec2(8.f, 12.f), Vec2(10.f, 23.f));
            Assert::IsFalse(segment.x_overlap(other_segment));

            other_segment = Segment(Vec2(16.f, 12.f), Vec2(18.f, 23.f));
            Assert::IsFalse(segment.x_overlap(other_segment));
        }

        TEST_METHOD(y_overlap)
        {
            Segment segment(Vec2(12.f, 8.f), Vec2(14.f, 24.f));

            Segment other_segment(Vec2(10.f, 4.f), Vec2(13.f, 10.f));
            Assert::IsTrue(segment.y_overlap(other_segment));

            other_segment = Segment(Vec2(10.f, 13.f), Vec2(13.f, 23.f));
            Assert::IsTrue(segment.y_overlap(other_segment));

            other_segment = Segment(Vec2(10.f, 12.f), Vec2(13.f, 26.f));
            Assert::IsTrue(segment.y_overlap(other_segment));

            other_segment = Segment(Vec2(10.f, 4.f), Vec2(13.f, 7.f));
            Assert::IsFalse(segment.y_overlap(other_segment));

            other_segment = Segment(Vec2(10.f, 26.f), Vec2(13.f, 28.f));
            Assert::IsFalse(segment.y_overlap(other_segment));
        }

        TEST_METHOD(get_AABB)
        {
            Segment segment(Vec2(12.f, 8.f), Vec2(14.f, 24.f));

            AABB segment_aabb = segment.get_AABB();

            Assert::AreEqual(segment_aabb.min.x, Float(12.f));
            Assert::AreEqual(segment_aabb.min.y, Float(8.f));
            Assert::AreEqual(segment_aabb.max.x, Float(14.f));
            Assert::AreEqual(segment_aabb.max.y, Float(24.f));
        }

        TEST_METHOD(get_points_on_same_axis_with_distance_l)
        {
            Segment start_segment(Vec2(2.f, 10.f), Vec2(6.f, 2.f), 0.f);
            Segment end_segment(Vec2(10.f, 4.f), Vec2(18.f, 12.f), start_segment.length() + 10.f);

            Float length = 17.f;

            Vec2 p, q;

            Vec2 expected_p(4.0593470965373f, 5.8813058069254f);
            Vec2 expected_q(11.8813058069254f, 5.8813058069254f);

            Segment::get_points_on_same_axis_with_distance_l(start_segment, end_segment, length, false, p, q);

            Assert::AreEqual(p, expected_p);
            Assert::AreEqual(q, expected_q);
        }

        TEST_METHOD(get_points_on_same_axis_with_distance_l_y_no_solution)
        {
            Segment start_segment(Vec2(9.f, 17.f), Vec2(13.f, 13.f), 0.f);
            Segment end_segment(Vec2(20.f, 5.f), Vec2(26.5f, 17.5f), start_segment.length() + 10.f);

            Float length = 17.f;

            Vec2 p, q;


            bool result = Segment::get_points_on_same_axis_with_distance_l(start_segment, end_segment, length, false, p, q);

            Assert::IsFalse(result);
        }

        TEST_METHOD(get_points_on_same_axis_with_distance_l_y_start_segment_upside_down_no_solution)
        {
            Segment start_segment(Vec2(8.215f, 6.58f), Vec2(11.092f, 10.604f), 0.f);
            Segment end_segment(Vec2(20.f, 5.f), Vec2(24.f, 14.f), start_segment.length() + 10.f);

            Float length = 17.f;

            Vec2 p, q;


            bool result = Segment::get_points_on_same_axis_with_distance_l(start_segment, end_segment, length, false, p, q);

            Assert::IsFalse(result);
        }

        TEST_METHOD(get_points_on_same_axis_with_distance_l_y_end_segment_upside_down_no_solution)
        {
            Segment start_segment(Vec2(9.f, 17.f), Vec2(13.f, 13.f), 0.f);
            Segment end_segment(Vec2(25.f, 17.5f), Vec2(28.8f, 8.7f), start_segment.length() + 10.f);

            Float length = 17.f;

            Vec2 p, q;


            bool result = Segment::get_points_on_same_axis_with_distance_l(start_segment, end_segment, length, false, p, q);

            Assert::IsFalse(result);
        }

        TEST_METHOD(get_points_on_same_axis_with_distance_l_x_no_solution)
        {
            Segment start_segment(Vec2(40.f, 30.f), Vec2(36.02f, 37.85f), 0.f);
            Segment end_segment(Vec2(25.f, 17.5f), Vec2(40.f, 10.f), start_segment.length() + 10.f);

            Float length = 17.f;

            Vec2 p, q;


            bool result = Segment::get_points_on_same_axis_with_distance_l(start_segment, end_segment, length, true, p, q);

            Assert::IsFalse(result);
        }

        TEST_METHOD(get_points_on_same_axis_with_distance_l_x_start_segment_upside_down_no_solution)
        {
            Segment start_segment(Vec2(27.4f, 12.95f), Vec2(36.02f, 16.5f), 0.f);
            Segment end_segment(Vec2(25.f, 17.5f), Vec2(40.f, 10.f), start_segment.length() + 10.f);

            Float length = 17.f;

            Vec2 p, q;


            bool result = Segment::get_points_on_same_axis_with_distance_l(start_segment, end_segment, length, true, p, q);

            Assert::IsFalse(result);
        }

        TEST_METHOD(get_points_on_same_axis_with_distance_l_x_end_segment_upside_down_no_solution)
        {
            Segment start_segment(Vec2(31.918f, 33.802f), Vec2(24.443f, 30.97f), 0.f);
            Segment end_segment(Vec2(38.726f, 21.799f), Vec2(21.518f, 16.475f), start_segment.length() + 10.f);

            Float length = 17.f;

            Vec2 p, q;


            bool result = Segment::get_points_on_same_axis_with_distance_l(start_segment, end_segment, length, true, p, q);

            Assert::IsFalse(result);
        }
    };
}