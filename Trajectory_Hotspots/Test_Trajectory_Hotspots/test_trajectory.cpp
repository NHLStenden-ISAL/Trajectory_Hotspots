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
