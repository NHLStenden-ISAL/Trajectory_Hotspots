#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/sweep_line_status_structure.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using namespace Segment_Intersection_Sweep_Line;

namespace TestTrajectoryHotspots
{
    TEST_CLASS(TestSweepLineStatusStructure)
    {
    public:
        TEST_METHOD(construction)
        {
            Sweep_Line_Status_structure sweep_line(100.0f);

            Assert::IsNull(sweep_line.root.get());
        }

        TEST_METHOD(insert_non_intersecting_segments)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(5.183123123f, 2.f), Vec2(5.22f, 11.27898f));    //3
            test_segments.emplace_back(Vec2(4.3813123f, 9.77978f), Vec2(2.8f, 3.5798f));    //2
            test_segments.emplace_back(Vec2(7.0267869f, 8.46879f), Vec2(7.24f, 0.7792f));   //4
            test_segments.emplace_back(Vec2(10.48974f, 10.287976f), Vec2(9.16f, 2.97974f)); //6
            test_segments.emplace_back(Vec2(1.f, 11.7932f), Vec2(1.9f, 1.78964f));          //1
            test_segments.emplace_back(Vec2(14.47892f, 11.08984f), Vec2(8.7798f, 1.5794f)); //7
            test_segments.emplace_back(Vec2(7.6784f, 10.2974f), Vec2(8.27894f, 0.7975f));   //5


            Sweep_Line_Status_structure status_structure(test_segments.at(0).get_top_point()->y);

            for (size_t i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_line_position(test_segments.at(i).get_top_point()->y);
                status_structure.insert(&test_segments.at(i));

                Assert::IsTrue(status_structure.contains(&test_segments.at(i)));
            }

            for (size_t i = 0; i < test_segments.size(); i++)
            {
                Assert::IsTrue(status_structure.contains(&test_segments.at(i)));
            }
        }

        TEST_METHOD(remove)
        {
            //TODO: Change test segments
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(5.183123123f, 2.f), Vec2(5.22f, 11.27898f));    //3
            test_segments.emplace_back(Vec2(4.3813123f, 9.77978f), Vec2(2.8f, 3.5798f));    //2
            test_segments.emplace_back(Vec2(7.0267869f, 8.46879f), Vec2(7.24f, 0.7792f));   //4
            test_segments.emplace_back(Vec2(10.48974f, 10.287976f), Vec2(9.16f, 2.97974f)); //6
            test_segments.emplace_back(Vec2(1.f, 11.7932f), Vec2(1.9f, 1.78964f));          //1
            test_segments.emplace_back(Vec2(14.47892f, 11.08984f), Vec2(8.7798f, 1.5794f)); //7
            test_segments.emplace_back(Vec2(7.6784f, 10.2974f), Vec2(8.27894f, 0.7975f));   //5


            Sweep_Line_Status_structure status_structure(test_segments.at(0).get_top_point()->y);

            for (size_t i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_line_position(test_segments.at(i).get_top_point()->y);
                status_structure.insert(&test_segments.at(i));
            }

            for (size_t i = 0; i < test_segments.size(); i++)
            {
                status_structure.remove(&test_segments.at(i));
                Assert::IsFalse(status_structure.contains(&test_segments.at(i)));
            }

            Assert::IsNull(status_structure.root.get());
        }

        TEST_METHOD(contains)
        {

        }

        TEST_METHOD(destruction)
        {

        }
    };
}