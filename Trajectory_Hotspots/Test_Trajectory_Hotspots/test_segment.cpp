#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/sweep_line_status_structure.h"
#include "../Trajectory_Hotspots/segment_intersection.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestTrajectoryHotspots
{
	TEST_CLASS(TestSegment)
	{
	public:
		TEST_METHOD(intersection)
		{
			std::vector<Segment> test_segments;
			test_segments.emplace_back(Vec2(9.0f, 7.0f), Vec2(6.0f, 2.0f)); //f
			test_segments.emplace_back(Vec2(9.2f, 4.35f), Vec2(8.01f, 8.22f)); //g
			Vec2 intersection;
			Segment::intersection_two_segments(&test_segments.at(0), &test_segments.at(1),intersection);
			Vec2 correct;
			correct = (Vec2(8.593479498861f, 6.3224658314351f));
			Assert::AreEqual(intersection, correct);

		}
	};
}