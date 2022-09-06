#include "pch.h"
#include "trajectory.h"

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
    //TODO:Check if length is enough for an UV to exist..
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
    //with the minimum or maximum x or y-coordinate of the bounding box based on the first and last vertex

    //For each segment, query with start + L and query with end + L, iterate from first to last.
    //For each end segment query bounding box uv, then check if the border lines intersect the start of end segment
    for (auto& trajectory_segment : trajectory_segments)
    {

        //Find all segments in which the sub-trajectories starting in this segment end
        float end_range_start = trajectory_segment.start_t + length;
        float end_range_end = trajectory_segment.end_t + length;

        int end_range_start_index = tree.query(end_range_start);
        int end_range_end_index = tree.query(end_range_end);

        //Get u, the time at the first vertex after the sub-trajectory start point
        float start = trajectory_segment.end_t;

        //Loop through all the possible end segments and query for an AABB of the sub-trajectory between u and v
        //Check if the start or end segment intersects one of its border lines
        //If so, report possible hotspot
        for (int i = end_range_start_index; i <= end_range_end_index; ++i)
        {
            const Segment& end_segment = trajectory_segments.at(i);

            //Get v, the time at the first vertex before the sub-trajectory end point
            float end = end_segment.start_t;

            //Obtain the bounding box of the subtrajectory between u and v
            AABB uv_bounding_box = tree.query(start, end);

            //TODO: Check if forward/backward time is on the end/start segment, the trajectory has to start/end in the same start/end segments else UV condition breaks!
            //TODO: Pass start_t& and end_t& to check_breakpoint III and IV instead of tree and current_hoptspot?

            //Check if any of the four sides of the AABB intersects either the start or end segment, if so, check for new hotspot
            AABB current_hotspot;
            if (check_breakpoint_III_x(tree, length, trajectory_segment, uv_bounding_box.min.x, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (check_breakpoint_III_x(tree, length, trajectory_segment, uv_bounding_box.max.x, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (check_breakpoint_III_y(tree, length, trajectory_segment, uv_bounding_box.min.y, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (check_breakpoint_III_y(tree, length, trajectory_segment, uv_bounding_box.max.y, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }

            if (check_breakpoint_IV_x(tree, length, end_segment, uv_bounding_box.min.x, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (check_breakpoint_IV_x(tree, length, end_segment, uv_bounding_box.max.x, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (check_breakpoint_IV_y(tree, length, end_segment, uv_bounding_box.min.y, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (check_breakpoint_IV_y(tree, length, end_segment, uv_bounding_box.max.y, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
        }
    }



    //Breakpoint type V, the start and end of the subtrajectory lie on the same x or y coordinate

    //TODO: Same loop at IV, check with helper function and check for hotspot

    return hotspot;
}

bool Trajectory::check_breakpoint_III_x(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_x, AABB& potential_hotspot) const
{
    float intersection_y;
    if (trajectory_segment.x_intersects(line_x, intersection_y))
    {
        float start_t = trajectory_segment.get_time_at_y(intersection_y);
        float end_t = start_t + length;

        potential_hotspot = tree.query(start_t, end_t);

        return true;
    }

    return false;
}

bool Trajectory::check_breakpoint_III_y(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_y, AABB& potential_hotspot) const
{
    float intersection_x;
    if (trajectory_segment.y_intersects(line_y, intersection_x))
    {
        float start_t = trajectory_segment.get_time_at_x(intersection_x);
        float end_t = start_t + length;

        potential_hotspot = tree.query(start_t, end_t);

        return true;
    }

    return false;
}

bool Trajectory::check_breakpoint_IV_x(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_x, AABB& potential_hotspot) const
{
    float intersection_y;
    if (trajectory_segment.x_intersects(line_x, intersection_y))
    {
        float end_t = trajectory_segment.get_time_at_y(intersection_y);
        float start_t = end_t - length;

        potential_hotspot = tree.query(start_t, end_t);

        return true;
    }

    return false;
}

bool Trajectory::check_breakpoint_IV_y(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_y, AABB& potential_hotspot) const
{
    float intersection_x;
    if (trajectory_segment.y_intersects(line_y, intersection_x))
    {
        float end_t = trajectory_segment.get_time_at_x(intersection_x);
        float start_t = end_t - length;

        potential_hotspot = tree.query(start_t, end_t);

        return true;
    }

    return false;
}