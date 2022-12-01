#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/sweep_line_status_structure.h"
#include "../Trajectory_Hotspots/segment_intersection.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using namespace Segment_Intersection_Sweep_Line;


namespace TestTrajectoryHotspots
{
	TEST_CLASS(TestSegmentintersection)
	{
	public:

		TEST_METHOD(build_tree_with_intersection)
		{
			std::vector<Segment> test_segments;
			//test_segments.emplace_back(Vec2(3.2f, 5.28f), Vec2(4.43f, 8.73f)); //g
			//test_segments.emplace_back(Vec2(9.2f, 4.35f), Vec2(8.01f, 8.22f)); //h
			//test_segments.emplace_back(Vec2(12.0f, 4.44f), Vec2(11.29f, 8.39f)); //i
			//test_segments.emplace_back(Vec2(1.0f, 4.0f), Vec2(2.0f, 6.0f)); //j
			//test_segments.emplace_back(Vec2(9.0f, 7.0f), Vec2(6.0f, 2.0f)); //f
			test_segments.emplace_back(Vec2(9.0f, 7.0f), Vec2(6.0f, 2.0f)); //f
			test_segments.emplace_back(Vec2(9.2f, 4.35f), Vec2(8.01f, 8.22f)); //g
			//status_structure.set_line_position(line_pos);
			std::vector<Vec2> test = find_segment_intersections(test_segments);
			std::vector<Vec2> correct;
			correct.emplace_back(Vec2(8.593479498861f, 6.3224658314351f));
			Assert::AreEqual(test, correct);

		}

		TEST_METHOD(build_tree_with_2_segments_with_2_intersections)
		{
			std::vector<Segment> test_segments;
			test_segments.emplace_back(Vec2(5.0f, 7.0f), Vec2(9.0f, 4.0f)); //f
			test_segments.emplace_back(Vec2(5.0f, 4.0f), Vec2(9.0f, 7.0f)); //g
			test_segments.emplace_back(Vec2(12.0f, 6.0f), Vec2(15.0f, 3.0f)); //h
			test_segments.emplace_back(Vec2(12.0f, 3.0f), Vec2(15.0f, 6.0f)); //i

			std::vector<Vec2> test = find_segment_intersections(test_segments);
			std::vector<Vec2> correct;
			correct.emplace_back(Vec2(7.0, 5.5f));
			correct.emplace_back(Vec2(13.5, 4.5f));
			Assert::AreEqual(test, correct);

		}

		TEST_METHOD(build_tree_with_nasty_case_1)
		{
			std::vector<Segment> test_segments;
			test_segments.emplace_back(Vec2(2.0f, 7.0f), Vec2(6.0f, 3.0f)); //f
			test_segments.emplace_back(Vec2(2.0f, 3.0f), Vec2(6.0f, 7.0f)); //g
			test_segments.emplace_back(Vec2(4.0f, 7.0f), Vec2(4.0f, 6.0f)); //h
			

			std::vector<Vec2> test = find_segment_intersections(test_segments);
			std::vector<Vec2> correct;
			correct.emplace_back(Vec2(4.0f, 5.0f));
			Assert::AreEqual(test, correct);

		}
	};
}