#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/halfedge.h"
#include "../Trajectory_Hotspots/vertex.h"
#include "../Trajectory_Hotspots/face.h"
#include "../Trajectory_Hotspots/dcel.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestTrajectoryHotspots
{
	TEST_CLASS(TestTrajectoryHotspotsEdgeInsertionDcel)
	{
	public:

		TEST_METHOD(insert_edge)
		{
			
		}
		TEST_METHOD(polar_coordinates)
        {
            // Create a vertex at position (3, 4)
            Vertex dummy_v(Vec2(3, 4));
            // Create a vector of HalfEdge pointers
            std::vector<HalfEdge*> halfedges;
            //Vertex* origin, HalfEdge* twin, Face* incidentFace, HalfEdge* next, HalfEdge* prev
            // Create half edges with positions (0, 0), (1, 0), (0, 1) and (1, 1)
            HalfEdge h1(dummy_v, 0, 0);
            HalfEdge h2(1, 0);
            HalfEdge h3(0, 1);
            HalfEdge h4(1, 1);

            // Add the half edges to the vector of HalfEdge pointers
            halfedges.push_back(&h1);
            halfedges.push_back(&h2);
            halfedges.push_back(&h3);
            halfedges.push_back(&h4);


            

            // Call the get_polar_angles function
            std::vector<float> polar_angles = HalfEdge::get_polar_angles(halfedges, &v);

            // Check that the returned polar angles are correct
            assert(polar_angles[0] == atan2(-4, -3));
            assert(polar_angles[1] == atan2(-4, -2));
            assert(polar_angles[2] == atan2(-3, -3));
            assert(polar_angles[3] == atan2(-3, -2));
		}
		TEST_METHOD(correct_prev_next_ptrs)
		{

		}
	};
}
