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
            std::vector<Segment> trajectory_edges;

            trajectory_points.emplace_back(1.f, 1.f);
            trajectory_points.emplace_back(2.f, 4.f);
            trajectory_points.emplace_back(5.f, 7.f);
            trajectory_points.emplace_back(8.f, 4.f);
            trajectory_points.emplace_back(10.f, 1.f);

            //Create trajectory edges, set t with lengths
            Float start_t = 0.f;
            for (size_t i = 0; i < trajectory_points.size() - 1; i++)
            {
                trajectory_edges.emplace_back(trajectory_points.at(i), trajectory_points.at(i + 1), start_t);
                start_t += trajectory_edges.cbegin()->length();
            }

            Trajectory trajectory(trajectory_edges);

            Float query_length = 2.0f;

            AABB hotspot = trajectory.get_hotspot_fixed_length_contiguous(query_length);

            Assert::IsTrue(hotspot.max_size() == 1.4142135623730950488016887242097f);
        }

        TEST_METHOD(get_hotspot_fixed_length_contiguous_curl)
        {
            std::vector<Vec2> trajectory_points;
            std::vector<Segment> trajectory_edges;

            trajectory_points.emplace_back(4.f, 5.f);
            trajectory_points.emplace_back(2.f, 4.f);
            trajectory_points.emplace_back(5.f, 7.f);
            trajectory_points.emplace_back(8.f, 4.f);
            trajectory_points.emplace_back(10.f, 1.f);

            //Create trajectory edges, set t with lengths
            Float start_t = 0.f;
            for (size_t i = 0; i < trajectory_points.size() - 1; i++)
            {
                trajectory_edges.emplace_back(trajectory_points.at(i), trajectory_points.at(i + 1), start_t);
                start_t += trajectory_edges.cbegin()->length();
            }

            Trajectory trajectory(trajectory_edges);

            Float query_length = 6.4787086646191f;

            AABB hotspot = trajectory.get_hotspot_fixed_length_contiguous(query_length);

            Assert::IsTrue(hotspot.max_size() == 3.0f);

            Assert::IsTrue(hotspot.min == Vec2(2.f, 4.f));
            Assert::IsTrue(hotspot.max == Vec2(5.f, 7.f));
        }
    };


}
