#include "pch.h"
#include "trajectory.h"

#include "segment_search_tree.h"

Trajectory::Trajectory(std::vector<Segment>& ordered_segments) : trajectory_segments(ordered_segments)
{
    trajectory_end = ordered_segments.end()->end_t;
}

//Returns a hotspot with a fixed radius at a position that maximizes the trajectory inside it
AABB Trajectory::get_hotspot_fixed_radius(float radius) const
{
    return AABB();
}

AABB Trajectory::get_hotspot_fixed_length(float length) const
{
    return AABB();
}

AABB Trajectory::get_hotspot_fixed_radius_contiguous(float radius) const
{
    return AABB();
}

AABB Trajectory::get_hotspot_fixed_length_contiguous(float length) const
{
    Segment_Search_Tree tree(trajectory_segments);

    AABB hotspot;

    //Breakpoint type I, start at vertices of the trajectory
    for (auto& trajectory_segment : trajectory_segments)
    {
        float start = trajectory_segment.start_t;
        float end = start + length;

        if (end > trajectory_end)
        {
            //When we exceed the trajectories bounds we can stop
            break;
        }

        AABB current_hotspot = tree.query(start, end);

        if (current_hotspot.width() < hotspot.width())
        {
            hotspot = current_hotspot;
        }
    }

    //Breakpoint type II, end at the vertices of the trajectory
    std::vector<Segment>::const_reverse_iterator r_iterator;
    for (r_iterator = trajectory_segments.rbegin(); r_iterator != trajectory_segments.rend(); r_iterator++)
    {
        float end = r_iterator->end_t;
        float start = end - length;

        if (start < 0)
        {
            //When we exceed the trajectories bounds we can stop
            break;
        }

        AABB current_hotspot = tree.query(start, end);

        if (current_hotspot.width() < hotspot.width())
        {
            hotspot = current_hotspot;
        }
    }

    //Breakpoint type III and IV, the start/end of the subtrajectory coincides 
    //with the minimum or maximum x or y-coordinate in the bounding box
    //based on the first and last vertex

    //TODO: iterator through the segments, query bounding box between u,v
    //Check if the min/max x/y intersects with the start of end line segments
    //If so, extend the bounding box and check if hotspot

    //TODO: Add segment query to the segment_search_tree so we dont need to loop through the segments to find the end/begin segment


    //Breakpoint type V, the start and end of the subtrajectory lie on the same x or y coordinate

    return hotspot;
}