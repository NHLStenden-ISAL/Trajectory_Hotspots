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

            Assert::AreEqual((size_t)5, test_dcel.vertex_count());
            Assert::AreEqual((size_t)8, test_dcel.half_edge_count());

            Vec2 intersection_point;
            new_segment.intersects(new_segment_2, intersection_point);

            Assert::AreEqual(intersection_point, test_dcel.vertices[4]->position);

            //TODO: Test pointers
        }

    };
}