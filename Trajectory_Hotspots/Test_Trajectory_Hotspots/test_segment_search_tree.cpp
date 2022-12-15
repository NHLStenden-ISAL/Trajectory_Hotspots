#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/segment_search_tree.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestTrajectoryHotspots
{
    TEST_CLASS(TestTrajectoryHotspotsSegmentSearchTree)
    {
    public:
        TEST_METHOD(tree_construction)
        {
            std::vector<Vec2> ordered_points;

            ordered_points.push_back(Vec2(4.2f, 2.8f));
            ordered_points.push_back(Vec2(6.3f, 3.1f));
            ordered_points.push_back(Vec2(2.422f, 7.442f));
            ordered_points.push_back(Vec2(9.4822f, 12.6492f));
            ordered_points.push_back(Vec2(1.2321f, 0.231f));
            ordered_points.push_back(Vec2(-3.321f, -3.2323f));

            //Convert points to segments
            std::vector<Segment> ordered_segments;
            Float total_time_t = 0.0f;
            for (size_t i = 0; i < ordered_points.size() - 1; i++)
            {
                ordered_segments.push_back(Segment(ordered_points.at(i), ordered_points.at(i + 1), total_time_t));
                total_time_t = ordered_segments.at(ordered_segments.size() - 1).end_t;
            }

            Segment_Search_Tree ss_tree(ordered_segments);

            Assert::IsTrue(ss_tree.root.left->left->left->node_start_t == 0.f);
            Assert::IsTrue(ss_tree.root.left->left->left->node_end_t == ordered_segments.at(0).end_t);

            Assert::IsTrue(ss_tree.root.left->left->right->node_start_t == ordered_segments.at(1).start_t);
            Assert::IsTrue(ss_tree.root.left->left->right->node_end_t == ordered_segments.at(1).end_t);

            Assert::IsTrue(ss_tree.root.left->right->node_start_t == ordered_segments.at(2).start_t);
            Assert::IsTrue(ss_tree.root.left->right->node_end_t == ordered_segments.at(2).end_t);

            Assert::IsTrue(ss_tree.root.right->left->node_start_t == ordered_segments.at(3).start_t);
            Assert::IsTrue(ss_tree.root.right->left->node_end_t == ordered_segments.at(3).end_t);

            Assert::IsTrue(ss_tree.root.right->right->node_start_t == ordered_segments.at(4).start_t);
            Assert::IsTrue(ss_tree.root.right->right->node_end_t == ordered_segments.at(4).end_t);
        }

        TEST_METHOD(tree_query)
        {
            std::vector<Vec2> ordered_points;

            ordered_points.push_back(Vec2(4.2f, 2.8f));
            ordered_points.push_back(Vec2(6.3f, 3.1f));
            ordered_points.push_back(Vec2(2.422f, 7.442f));
            ordered_points.push_back(Vec2(9.4822f, 12.6492f));
            ordered_points.push_back(Vec2(1.2321f, 0.231f));
            ordered_points.push_back(Vec2(-3.321f, -3.2323f));

            //Convert points to segments
            std::vector<Segment> ordered_segments;
            Float total_time_t = 0.0f;
            for (size_t i = 0; i < ordered_points.size() - 1; i++)
            {
                ordered_segments.push_back(Segment(ordered_points.at(i), ordered_points.at(i + 1), total_time_t));
                total_time_t = ordered_segments.at(ordered_segments.size() - 1).end_t;
            }

            Segment_Search_Tree ss_tree(ordered_segments);

            AABB queried_bb = ss_tree.Query(2.2f, 15.f);

            AABB queried_bb2 = ss_tree.Query(16.7157536f, 37.3452644f);

        }

        TEST_METHOD(tree_destruction)
        {

        }
    };
}