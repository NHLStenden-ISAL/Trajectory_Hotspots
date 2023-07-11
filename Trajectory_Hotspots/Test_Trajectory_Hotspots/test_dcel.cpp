#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/dcel.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace Microsoft::VisualStudio::CppUnitTestFramework
{
    template<> static std::wstring ToString<DCEL::DCEL_Half_Edge>(class DCEL::DCEL_Half_Edge* t) { return L"DCEL::DCEL_Half_Edge*"; }
    template<> static std::wstring ToString<DCEL::DCEL_Half_Edge>(const class DCEL::DCEL_Half_Edge& t) { return L"DCEL::DCEL_Half_Edge&"; }

    template<> static std::wstring ToString<DCEL::DCEL_Vertex>(class DCEL::DCEL_Vertex* t) { return L"DCEL::DCEL_Vertex*"; }
    template<> static std::wstring ToString<DCEL::DCEL_Vertex>(const class DCEL::DCEL_Vertex& t) { return L"DCEL::DCEL_Vertex&"; }
}

namespace TestTrajectoryHotspots
{
    TEST_CLASS(TestTrajectoryHotspotsDCEL)
    {
        TEST_METHOD(DCEL_Construction_single_segment)
        {
            DCEL test_dcel;

            const Segment new_segment(Vec2(6.f, 6.f), Vec2(20.f, 20.f));

            test_dcel.insert_segment(new_segment);

            Assert::AreEqual((size_t)2, test_dcel.vertex_count());
            Assert::AreEqual((size_t)2, test_dcel.half_edge_count());

            Assert::AreEqual(test_dcel.half_edges[0]->twin, test_dcel.half_edges[1].get());
            Assert::AreEqual(test_dcel.half_edges[1]->twin, test_dcel.half_edges[0].get());

            //With a single segment the half edges should be each others next and prev
            Assert::AreEqual(test_dcel.half_edges[0]->next, test_dcel.half_edges[1].get());
            Assert::AreEqual(test_dcel.half_edges[1]->next, test_dcel.half_edges[0].get());
            Assert::AreEqual(test_dcel.half_edges[0]->prev, test_dcel.half_edges[1].get());
            Assert::AreEqual(test_dcel.half_edges[1]->prev, test_dcel.half_edges[0].get());

            //Both half edges should have a unique origin vertex, which of the two new vertices doesn't matter
            Assert::IsTrue((test_dcel.half_edges[0]->origin == test_dcel.vertices[0].get() && test_dcel.half_edges[1]->origin == test_dcel.vertices[1].get())
                || (test_dcel.half_edges[1]->origin == test_dcel.vertices[0].get() && test_dcel.half_edges[0]->origin == test_dcel.vertices[1].get()));

            //And the two vertices should each have a unique incident half edge
            Assert::IsTrue((test_dcel.vertices[0]->incident_half_edge == test_dcel.half_edges[0].get() && test_dcel.vertices[1]->incident_half_edge == test_dcel.half_edges[1].get())
                || (test_dcel.vertices[0]->incident_half_edge == test_dcel.half_edges[1].get() && test_dcel.vertices[1]->incident_half_edge == test_dcel.half_edges[0].get()));
        }

        TEST_METHOD(DCEL_Construction_two_internal_intersecting_segments)
        {
            DCEL test_dcel;

            const Segment new_segment(Vec2(6.f, 6.f), Vec2(20.f, 20.f));
            const Segment new_segment_2(Vec2(4.f, 20.f), Vec2(22.f, 8.f));

            test_dcel.insert_segment(new_segment);
            test_dcel.insert_segment(new_segment_2);

            //Four endpoints plus the intersection point
            Assert::AreEqual((size_t)5, test_dcel.vertex_count());

            //Every segment consists of two half-edges, 
            //both segments split into two at the intersection point, doubling the amount of half-edge to 8
            Assert::AreEqual((size_t)8, test_dcel.half_edge_count());

            //The last added vertex should be at the intersection point
            Vec2 intersection_point;
            new_segment.intersects(new_segment_2, intersection_point);

            Assert::AreEqual(intersection_point, test_dcel.vertices[4]->position);

            //Both segments get split into two, so there should be four incident half-edges around the vertex at the intersection point
            std::vector<DCEL::DCEL_Half_Edge*> incident_half_edges = test_dcel.vertices[4]->get_incident_half_edges();

            Assert::AreEqual((size_t)4, incident_half_edges.size());

            //All endpoints should show up once around the intersection vertex
            std::vector<Vec2> incident_targets;
            for (auto& incident_half_edge : incident_half_edges)
            {
                incident_targets.push_back(incident_half_edge->target()->position);
            }

            std::vector<Vec2> endpoints {new_segment.start, new_segment_2.start, new_segment.end, new_segment_2.end};

            Assert::IsTrue(std::is_permutation(incident_targets.begin(), incident_targets.end(), endpoints.begin()));
        }

    };
}