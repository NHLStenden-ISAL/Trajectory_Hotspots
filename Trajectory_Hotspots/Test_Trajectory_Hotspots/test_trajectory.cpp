#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/trajectory.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestTrajectoryHotspots
{

    TEST_CLASS(TestTrajectoryHotspotsTrajectory)
    {
    public:

        TEST_METHOD(get_hotspot_fixed_radius)
        {

        }

        TEST_METHOD(get_hotspot_fixed_length)
        {

        }

        TEST_METHOD(get_hotspot_fixed_radius_contiguous)
        {

        }

        TEST_METHOD(get_hotspot_fixed_radius_contiguous_non_zero)
        {
            std::vector<Vec2> test_points = {
                { 114.38f, 296.93f },
                { 114.47f, 296.928f },
                { 114.29,296.871 },
                { 114.13,296.835 },
                { 113.87,296.82 },
                { 113.84,296.841 },
                { 114,296.837 },
                { 113.88,296.828 },
                { 113.73,296.827 },
                { 113.7,296.818 },
                { 113.74,296.804 },
                { 113.8,296.743 },
                { 113.59,296.755 },
                { 113.51,296.787 },
                { 113.5,296.797 },
                { 113.32,296.753 },
                { 113.16,296.722 },
                { 113.04,296.726 },
                { 112.9,296.736 },
                { 112.89,296.72 },
                { 113.15,296.708 },
                { 113.24,296.695 },
                { 113.23,296.694 },
                { 112.93,296.701 },
                { 112.69,296.707 },
                { 112.47,296.691 },
                { 112.51,296.669 },
                { 112.64,296.644 },
                { 112.8,296.596 },
                { 112.43,296.561 },
                { 111.49,296.512 },
                { 110.8,296.501 },
                { 110.91,296.525 },
                { 111.32,296.534 },
                { 111.3,296.528 },
                { 111.22,296.521 },
                { 111.13,296.449 },
                { 111.24,296.382 },
                { 111.07,296.306 },
                { 110.97,296.248 },
                { 110.37,296.21 },
                { 109.91,296.174 },
                { 110.58,296.232 },
                { 110.65,296.261 },
                { 110.33,296.359 },
                { 110.12,296.398 },
                { 109.67,296.373 },
                { 109.37,296.313 },
                { 109.03,296.236 },
                { 109.01,296.147 },
                { 110.54,296.124 },
                { 111.39,296.238 },
                { 111.61,296.305 },
                { 112.42,296.326 },
                { 112.87,296.363 },
                { 112.92,296.404 },
                { 112.73,296.415 },
                { 113.08,296.497 },
                { 113.2,296.602 },
                { 114.31,296.922 },
                { 114.34,296.943 },
                { 114.51,296.911 },
                { 114.92,296.848 },
                { 115.26,296.81 },
                { 115.39,296.803 },
                { 115.09,296.813 },
                { 114.49,296.839 },
                { 114.52,296.849 },
                { 114.54,296.854 },
                { 114.6,296.852 },
                { 114.68,296.846 },
                { 114.9,296.844 },
                { 114.73,296.862 },
                { 114.61,296.845 },
                { 114.4,296.887 },
                { 114.7,296.898 },
                { 114.8,296.888 },
                { 115.02,296.895 },
                { 114.96,296.9 },
                { 114.95,296.894 },
                { 114.36,296.912 },
                { 114.04,296.923 },
                { 113.44,296.925 },
                { 113.11,296.919 },
                { 112.86,296.905 },
                { 112.81,296.873 },
                { 113.53,296.877 },
                { 114.93,296.838 },
                { 115.89,296.836 },
                { 116.02,296.863 },
                { 116,296.855 },
                { 115.18,296.819 },
                { 115.07,296.825 },
                { 115.17,296.822 },
                { 115.41,296.826 },
                { 115.77,296.834 },
                { 116.29,296.809 },
                { 116.35,296.79 },
                { 117.07,296.904 },
                { 117.33,296.982 },
                { 117.62,297.039 },
                { 117.92,297.064 },
                { 118.5,297.06 },
                { 118.87,297.073 },
                { 119.4,297.107 },
                { 119.35,297.117 },
                { 119.1,297.13 },
                { 118.98,297.118 },
                { 118.53,297.09 },
                { 118.45,297.056 },
                { 118.43,297.051 },
                { 117.71,297.075 },
                { 117.51,297.047 },
                { 117.39,296.976 },
                { 116.78,296.884 },
                { 115.86,296.824 },
                { 116.31,296.853 },
                { 116.13,296.842 },
                { 115.82,296.786 },
                { 115.25,296.793 },
                { 115.65,296.73 },
                { 115.56,296.714 },
                { 115.52,296.754 },
                { 115.54,296.762 },
                { 115.6,296.745 },
                { 115.97,296.737 },
                { 116.36,296.796 },
                { 116.17,296.776 },
                { 115.66,296.768 },
                { 115.61,296.757 },
                { 115.59,296.758 },
                { 115.29,296.771 } };

               Trajectory trajectory(test_points);

               AABB aabb = trajectory.get_hotspot_fixed_radius_contiguous(5.f);

               Assert::IsTrue(aabb.max_size() > 0.f);
        }

        //2B
        TEST_METHOD(get_hotspot_fixed_length_contiguous_full_diagonal)
        {
            std::vector<Vec2> trajectory_points;

            trajectory_points.emplace_back(1.f, 1.f);
            trajectory_points.emplace_back(2.f, 4.f);
            trajectory_points.emplace_back(5.f, 7.f);
            trajectory_points.emplace_back(8.f, 4.f);
            trajectory_points.emplace_back(10.f, 1.f);

            Trajectory trajectory(trajectory_points);

            Float query_length = 2.0f;

            AABB hotspot = trajectory.get_hotspot_fixed_length_contiguous(query_length);

            Assert::IsTrue(hotspot.max_size() == 1.4142135623730950488016887242097f);
        }

        TEST_METHOD(get_hotspot_fixed_length_contiguous_curl)
        {
            std::vector<Vec2> trajectory_points;

            trajectory_points.emplace_back(4.f, 5.f);
            trajectory_points.emplace_back(2.f, 4.f);
            trajectory_points.emplace_back(5.f, 7.f);
            trajectory_points.emplace_back(8.f, 4.f);
            trajectory_points.emplace_back(10.f, 1.f);

            Trajectory trajectory(trajectory_points);

            Float query_length = 6.4787086646191f;

            AABB hotspot = trajectory.get_hotspot_fixed_length_contiguous(query_length);

            Assert::IsTrue(hotspot.max_size() == 3.0f);

            Assert::IsTrue(hotspot.min == Vec2(2.f, 4.f));
            Assert::IsTrue(hotspot.max == Vec2(5.f, 7.f));
        }

        TEST_METHOD(get_hotspot_fixed_length_contiguous_basic_breakpoint_v)
        {
            std::vector<Vec2> trajectory_points;

            trajectory_points.emplace_back(3.f, 3.f);
            trajectory_points.emplace_back(5.14f, 5.69f);
            trajectory_points.emplace_back(5.5f, 5.5f);
            trajectory_points.emplace_back(4.5f, 3.5f);

            Trajectory trajectory(trajectory_points);

            Float query_length = 1.8041624196468f;

            AABB hotspot = trajectory.get_hotspot_fixed_length_contiguous(query_length);

            Assert::IsTrue(hotspot.min == Vec2(4.60556459f, 5.018209f));
            Assert::IsTrue(hotspot.max == Vec2(5.5f, 5.69f));
        }

        TEST_METHOD(get_hotspot_fixed_length_contiguous_basic_breakpoint_I)
        {
            std::vector<Vec2> trajectory_points;

            trajectory_points.emplace_back(4.f, 24.f);
            trajectory_points.emplace_back(4.f, 20.f);
            trajectory_points.emplace_back(6.f, 16.f);
            trajectory_points.emplace_back(8.f, 14.f);
            trajectory_points.emplace_back(12.f, 10.5f);

            Trajectory trajectory(trajectory_points);

            Float query_length = 3.f;

            AABB hotspot = trajectory.get_hotspot_fixed_length_contiguous(query_length);

            Assert::IsTrue(hotspot.min == Vec2(6.f, 13.8870182f));
            Assert::IsTrue(hotspot.max == Vec2(8.12912178f, 16.f));
        }

        TEST_METHOD(get_hotspot_fixed_length_contiguous_basic_breakpoint_II)
        {
            std::vector<Vec2> trajectory_points;

            trajectory_points.emplace_back(12.f, 10.5f);
            trajectory_points.emplace_back(8.f, 14.f);
            trajectory_points.emplace_back(6.f, 16.f);
            trajectory_points.emplace_back(4.f, 20.f);
            trajectory_points.emplace_back(4.f, 24.f);

            Trajectory trajectory(trajectory_points);

            Float query_length = 3.f;

            AABB hotspot = trajectory.get_hotspot_fixed_length_contiguous(query_length);

            Assert::IsTrue(hotspot.min == Vec2(6.f, 13.8870182f));
            Assert::IsTrue(hotspot.max == Vec2(8.12912178f, 16.f));
        }

    };


}
