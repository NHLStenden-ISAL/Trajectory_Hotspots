#include "pch.h"
#include "trajectory.h"

Trajectory::Trajectory(std::vector<Segment>& ordered_segments) : trajectory_segments(ordered_segments)
{
    trajectory_start = ordered_segments.begin()->start_t;
    trajectory_end = ordered_segments.end()->end_t;

    for (Segment& i : ordered_segments)
    {
        trajectory_length += i.length();
    }
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
    //TODO: This works for now with using time because time == length, but we might want to change the tree to also store the lengths..
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

        if (start < trajectory_start)
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
        //Check breakpoints IV and V
        for (int i = end_range_start_index; i <= end_range_end_index; ++i)
        {
            const Segment& end_segment = trajectory_segments.at(i);

            //Get v, the time at the first vertex before the sub-trajectory end point
            float end = end_segment.start_t;

            //Obtain the bounding box of the subtrajectory between u and v
            AABB uv_bounding_box = tree.query(start, end);

            //TODO: Check if forward/backward time is on the end/start segment, the trajectory has to start/end in the same start/end segments else UV condition breaks!
            //TODO: Pass start_t& and end_t& to flc_breakpoint III and IV instead of tree and current_hoptspot?

            //Breakpoint IV, Check if any of the four sides of the AABB of the subtrajectory between u and v intersects either the start or end segment, if so, check for new hotspot
            AABB current_hotspot;
            if (flc_breakpoint_III_x(tree, length, trajectory_segment, uv_bounding_box.min.x, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (flc_breakpoint_III_x(tree, length, trajectory_segment, uv_bounding_box.max.x, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (flc_breakpoint_III_y(tree, length, trajectory_segment, uv_bounding_box.min.y, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (flc_breakpoint_III_y(tree, length, trajectory_segment, uv_bounding_box.max.y, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }

            if (flc_breakpoint_IV_x(tree, length, end_segment, uv_bounding_box.min.x, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_x(tree, length, end_segment, uv_bounding_box.max.x, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_y(tree, length, end_segment, uv_bounding_box.min.y, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_y(tree, length, end_segment, uv_bounding_box.max.y, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }

            //Breakpoint V, the start and end of the subtrajectory lie on the same x or y coordinate
            //TODO: What if a segment lies perpendicular to the other segment? Add condition in overlap? Probably fine because of the L restraint, still check the math..
            if (flc_breakpoint_V_x(tree, length, trajectory_segment, end_segment, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
            if (flc_breakpoint_V_y(tree, length, trajectory_segment, end_segment, current_hotspot)) { if (current_hotspot.width() > hotspot.width()) { hotspot = current_hotspot; } }
        }
    }

    return hotspot;
}

bool Trajectory::flc_breakpoint_III_x(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_x, AABB& potential_hotspot) const
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

bool Trajectory::flc_breakpoint_III_y(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_y, AABB& potential_hotspot) const
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

bool Trajectory::flc_breakpoint_IV_x(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_x, AABB& potential_hotspot) const
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

bool Trajectory::flc_breakpoint_IV_y(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_y, AABB& potential_hotspot) const
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

////bool Trajectory::flc_breakpoint_V(float length, vec2* s1, vec2* s2, vec2* s3, vec2* s4, vec2& p, vec2& q, bool x) {
//bool Trajectory::flc_breakpoint_V(const float length, const Segment& start_segment, const Segment& start_segment, const Segment& end_segment, vec2& p, vec2& q, bool x) const
//{
//    try {
//        float distanceBetweenEdges = (s3->time - s2->time) * this->getTotalLength();
//        float remainingLength = length - distanceBetweenEdges;
//        vec2 startVector = *s1 - *s2;
//        vec2 endVector = *s4 - *s3;
//        vec2 startEndDifference = *s2 - *s3;
//
//        float startDiff = (x) ? s2->getX() - s1->getX() : s2->getY() - s1->getY();
//        float endDiff = (x) ? s3->getX() - s4->getX() : s3->getY() - s4->getY();
//        float startLength = s1->calculateDistance(s2);
//        float endLength = s3->calculateDistance(s4);
//
//        if (startDiff >= 0) {
//            startLength *= -1;
//        }
//
//        if (endDiff >= 0) {
//            endLength *= -1;
//        }
//
//        float determinant = startDiff * endLength - -endDiff * startLength;
//
//        if (determinant == 0) {
//            return false;
//        }
//
//        float a = startDiff;
//        float b = -endDiff;
//        float c = startLength;
//        float d = endLength;
//
//        float e = (x) ? startEndDifference.getX() : startEndDifference.getY();
//        float f = remainingLength;
//
//        float lambda = (e * d - b * f) / determinant;
//        float rho = (a * f - e * c) / determinant;
//
//        p = *s2 + (startVector * lambda);
//        q = *s3 + (endVector * rho);
//
//        return true;
//    }
//    catch (std::invalid_argument& e) {
//        return false;
//    }
//    return true;
//}

bool Trajectory::flc_breakpoint_V_x(const Segment_Search_Tree& tree, const float length, const Segment& start_segment, const Segment& end_segment, AABB& potential_hotspot) const
{
    if (!start_segment.x_overlap(end_segment))
    {
        return false;
    }

    Vec2 start_vector = start_segment.start - start_segment.end;
    Vec2 end_vector = end_segment.end - end_segment.start;

    float determinant_x = start_vector.x * end_vector.length() - end_vector.x * start_vector.length();

    //TODO: Determinant perpendicular?
    if (determinant_x == 0.f)
    {
        return false;
    }

    //TODO: This works for now with using time because time == length, but we might want to change the tree to also store the lengths..
    float edge_distance = (end_segment.start_t - start_segment.end_t) * trajectory_length;
    float remaining_length = length - edge_distance;

    float start_length = start_vector.length();
    float end_length = end_vector.length();

    float start_end_difference = start_segment.end.x - end_segment.start.x;

    //Calculate the scalar for the vectors pointing to points p and q
    float lambda = ((start_end_difference * end_length) - (-end_vector.x * remaining_length)) / determinant_x;
    float rho = ((start_vector.x * remaining_length) - (start_end_difference * start_length)) / determinant_x;

    //Return false if p or q does not lie on their segment
    if (lambda < 0.f || lambda > 1.0f || rho < 0.f || rho > 1.0f)
    {
        return false;
    }

    //Calculate start and end point using the scalars
    Vec2 p = start_segment.end + lambda * start_vector;
    Vec2 q = end_segment.start + rho * end_vector;

    //Query the tree using the found start and end point
    potential_hotspot = tree.query(start_segment.get_time_at_point(p), end_segment.get_time_at_point(q));

    return true;
}

bool Trajectory::flc_breakpoint_V_y(const Segment_Search_Tree& tree, const float length, const Segment& start_segment, const Segment& end_segment, AABB& potential_hotspot) const
{
    if (!start_segment.y_overlap(end_segment))
    {
        return false;
    }

    Vec2 start_vector = start_segment.start - start_segment.end;
    Vec2 end_vector = end_segment.end - end_segment.start;

    float determinant_y = start_vector.y * end_vector.length() - end_vector.y * start_vector.length();

    //TODO: Determinant perpendicular?
    if (determinant_y == 0.f)
    {
        return false;
    }

    //TODO: This works for now with using time because time == length, but we might want to change the tree to also store the lengths..
    float edge_distance = (end_segment.start_t - start_segment.end_t) * trajectory_length;
    float remaining_length = length - edge_distance;

    float start_length = start_vector.length();
    float end_length = end_vector.length();

    float start_end_difference = start_segment.end.y - end_segment.start.y;

    //Calculate the scalar for the vectors pointing to points p and q
    float lambda = ((start_end_difference * end_length) - (-end_vector.y * remaining_length)) / determinant_y;
    float rho = ((start_vector.y * remaining_length) - (start_end_difference * start_length)) / determinant_y;

    //Return false if p or q does not lie on their segment
    if (lambda < 0.f || lambda > 1.0f || rho < 0.f || rho > 1.0f)
    {
        return false;
    }

    //Calculate start and end point using the scalars
    Vec2 p = start_segment.end + lambda * start_vector;
    Vec2 q = end_segment.start + rho * end_vector;

    //Query the tree using the found start and end point
    potential_hotspot = tree.query(start_segment.get_time_at_point(p), end_segment.get_time_at_point(q));

    return true;
}