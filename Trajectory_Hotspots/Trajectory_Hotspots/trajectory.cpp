#include "pch.h"
#include "trajectory.h"

Trajectory::Trajectory(std::vector<Segment>& ordered_segments) : trajectory_segments(ordered_segments)
{
    trajectory_start = ordered_segments.front().start_t;
    trajectory_end = ordered_segments.back().end_t;

    for (Segment& i : ordered_segments)
    {
        trajectory_length += i.length();
    }
}

//Returns a hotspot with a fixed radius at a position that maximizes the trajectory inside it
AABB Trajectory::get_hotspot_fixed_radius(Float radius) const
{
    return AABB();
}

AABB Trajectory::get_hotspot_fixed_length(Float length) const
{
    return AABB();
}

AABB Trajectory::get_hotspot_fixed_radius_contiguous(Float radius) const
{
    return AABB();
}

AABB Trajectory::get_hotspot_fixed_length_contiguous(Float length) const
{
    //TODO:Check if length is enough for an UV to exist..
    Segment_Search_Tree tree(trajectory_segments);

    AABB smallest_hotspot(
        std::numeric_limits<float>::max() / 2.f,
        std::numeric_limits<float>::max() / 2.f,
        std::numeric_limits<float>::lowest() / 2.f,
        std::numeric_limits<float>::lowest() / 2.f);

    //Breakpoint type I, the subtrajectory starts at a vertex of the trajectory
    for (auto& trajectory_segment : trajectory_segments)
    {
        Float start = trajectory_segment.start_t;
        Float end = start + length;

        if (end > trajectory_end)
        {
            //When we exceed the trajectories bounds we can stop
            break;
        }

        const AABB current_hotspot = tree.query(start, end);

        if (current_hotspot.max_size() < smallest_hotspot.max_size())
        {
            smallest_hotspot = current_hotspot;
        }
    }

    //Breakpoint type II, the subtrajectory ends at a vertex of the trajectory
    std::vector<Segment>::const_reverse_iterator r_iterator;
    for (r_iterator = trajectory_segments.rbegin(); r_iterator != trajectory_segments.rend(); r_iterator++)
    {
        Float end = r_iterator->end_t;
        Float start = end - length;

        if (start < trajectory_start)
        {
            //When we exceed the trajectories bounds we can stop
            break;
        }

        const AABB current_hotspot = tree.query(start, end);

        if (current_hotspot.max_size() < smallest_hotspot.max_size())
        {
            smallest_hotspot = current_hotspot;
        }
    }

    //Breakpoint type III and IV, the start/end of the subtrajectory coincides 
    //with the minimum or maximum x or y-coordinate of the bounding box 
    //based on the subtrajectory between the first and last vertex on the optimal subtrajectory

    //For each segment, query with start + L and end + L, iterate from first to last.
    //For each end segment query bounding box uv, then check if the border lines intersect the start of end segment
    for (const Segment& trajectory_segment : trajectory_segments)
    {

        //Find all segments in which the sub-trajectories starting on this segment end
        const Float end_range_start = trajectory_segment.start_t + length;
        const Float end_range_end = trajectory_segment.end_t + length;

        const int end_range_start_index = tree.query(end_range_start);
        const int end_range_end_index = tree.query(end_range_end);

        //Get u, the time at the first vertex after the sub-trajectory start point
        Float start = trajectory_segment.end_t;

        //Loop through all the possible end segments and query for an AABB of the sub-trajectory between u and v
        //Check breakpoints IV and V
        for (int i = end_range_start_index; i <= end_range_end_index; ++i)
        {
            const Segment& end_segment = trajectory_segments.at(i);

            //Get v, the time at the first vertex before the sub-trajectory end point
            Float end = end_segment.start_t;

            //Obtain the bounding box of the subtrajectory between u and v
            AABB uv_bounding_box = tree.query(start, end);

            //TODO: Check if forward/backward time is on the end/start segment, the trajectory has to start/end in the same start/end segments else UV condition breaks!
            //TODO: Pass start_t& and end_t& to flc_breakpoint III and IV instead of tree and current_hoptspot?

            //Breakpoint IV, Check if any of the four sides of the AABB of the subtrajectory between u and v intersects either the start or end segment, if so, check for new hotspot
            AABB current_hotspot;
            if (flc_breakpoint_III_x(length, trajectory_segment, end_segment, uv_bounding_box.min.x, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_III_x(length, trajectory_segment, end_segment, uv_bounding_box.max.x, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_III_y(length, trajectory_segment, end_segment, uv_bounding_box.min.y, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_III_y(length, trajectory_segment, end_segment, uv_bounding_box.max.y, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }

            if (flc_breakpoint_IV_x(length, trajectory_segment, end_segment, uv_bounding_box.min.x, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_x(length, trajectory_segment, end_segment, uv_bounding_box.max.x, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_y(length, trajectory_segment, end_segment, uv_bounding_box.min.y, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_y(length, trajectory_segment, end_segment, uv_bounding_box.max.y, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }

            //Breakpoint V, the start and end of the subtrajectory lie on the same x or y coordinate
            //TODO: What if a segment lies perpendicular to the other segment? Add condition in overlap? Probably fine because of the L restraint, still check the math..
            if (flc_breakpoint_V_x(tree, length, trajectory_segment, end_segment, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_V_y(tree, length, trajectory_segment, end_segment, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
        }
    }

    return smallest_hotspot;
}

//Checks if the vertical line through the side of the uv AABB intersects the starting segment and returns a potential hotspot if the subtrajectory still ends in the end segment
bool Trajectory::flc_breakpoint_III_x(const Float length, const Segment& start_segment, const Segment& end_segment, const Float vertical_line_x, const AABB& uv_bounding_box, AABB& potential_hotspot) const
{
    Float intersection_y;

    //Does the line through the side of the AABB intersect the start segment?
    if (!start_segment.x_intersects(vertical_line_x, intersection_y))
    {
        return false;
    }

    Float start_time = start_segment.get_time_at_x(vertical_line_x);
    Float end_time = start_time + length;

    //TODO: If intersection is infinite we exit here, is that ok?
    //The trajectory must start and end on the start and end segments or it violates the breakpoint because u and v change
    if (end_time < end_segment.start_t || end_time > end_segment.end_t)
    {
        return false;
    }

    //The vertical line intersects this start segment, construct start point
    Vec2 start_point(vertical_line_x, intersection_y);

    //Find the end point of the trajectory on the end segment
    Vec2 end_point = end_segment.get_point_at_time(end_time);

    //Augment the hotspot with the start and end points and return
    potential_hotspot = AABB::augment(uv_bounding_box, start_point);
    potential_hotspot.augment(end_point);

    return true;
}

//Checks if the horizontal line through the side of the uv AABB intersects the starting segment and returns a potential hotspot if the subtrajectory still ends in the end segment
bool Trajectory::flc_breakpoint_III_y(const Float length, const Segment& start_segment, const Segment& end_segment, const Float horizontal_line_y, const AABB& uv_bounding_box, AABB& potential_hotspot) const
{
    Float intersection_x;

    //Does the line through the side of the AABB intersect the start segment?
    if (!start_segment.y_intersects(horizontal_line_y, intersection_x))
    {
        return false;
    }

    Float start_time = start_segment.get_time_at_y(horizontal_line_y);
    Float end_time = start_time + length;

    //The trajectory must start and end on the start and end segments or it violates the breakpoint because u and v change
    if (end_time < end_segment.start_t || end_time > end_segment.end_t)
    {
        return false;
    }

    //The horizontal line intersects this start segment, construct start point
    Vec2 start_point(intersection_x, horizontal_line_y);

    //Find the end point of the trajectory on the end segment
    Vec2 end_point = end_segment.get_point_at_time(end_time);

    //Augment the hotspot with the start and end points and return
    potential_hotspot = AABB::augment(uv_bounding_box, start_point);
    potential_hotspot.augment(end_point);

    return true;
}

//Checks if the vertical line through the side of the uv AABB intersects the end segment and returns a potential hotspot if the subtrajectory still starts in the start segment
bool Trajectory::flc_breakpoint_IV_x(const Float length, const Segment& start_segment, const Segment& end_segment, const Float vertical_line_x, const AABB& uv_bounding_box, AABB& potential_hotspot) const
{
    Float intersection_y;

    //Does the line through the side of the AABB intersect the end segment?
    if (!end_segment.x_intersects(vertical_line_x, intersection_y))
    {
        return false;
    }

    Float end_time = end_segment.get_time_at_x(vertical_line_x);
    Float start_time = end_time - length;

    //The trajectory must start and end on the start and end segments or it violates the breakpoint because u and v change
    if (start_time < start_segment.start_t || start_time > start_segment.end_t)
    {
        return false;
    }

    //The vertical line intersects this end segment, construct end point
    Vec2 end_point(vertical_line_x, intersection_y);

    //Find the start point of the trajectory on the start segment
    Vec2 start_point = end_segment.get_point_at_time(start_time);

    //Augment the hotspot with the start and end points and return
    potential_hotspot = AABB::augment(uv_bounding_box, start_point);
    potential_hotspot.augment(end_point);

    return true;
}

//Checks if the horizontal line through the side of the uv AABB intersects the end segment and returns a potential hotspot if the subtrajectory still starts in the start segment
bool Trajectory::flc_breakpoint_IV_y(const Float length, const Segment& start_segment, const Segment& end_segment, const Float horizontal_line_y, const AABB& uv_bounding_box, AABB& potential_hotspot) const
{
    Float intersection_x;

    //Does the line through the side of the AABB intersect the end segment?
    if (!end_segment.y_intersects(horizontal_line_y, intersection_x))
    {
        return false;
    }

    Float end_time = end_segment.get_time_at_y(horizontal_line_y);
    Float start_time = end_time - length;

    //The trajectory must start and end on the start and end segments or it violates the breakpoint because u and v change
    if (start_time < start_segment.start_t || start_time > start_segment.end_t)
    {
        return false;
    }

    //The vertical line intersects this end segment, construct end point
    Vec2 end_point(intersection_x, horizontal_line_y);

    //Find the start point of the trajectory on the start segment
    Vec2 start_point = end_segment.get_point_at_time(start_time);

    //Augment the hotspot with the start and end points and return
    potential_hotspot = AABB::augment(uv_bounding_box, start_point);
    potential_hotspot.augment(end_point);

    return true;
}

////bool Trajectory::flc_breakpoint_V(Float length, vec2* s1, vec2* s2, vec2* s3, vec2* s4, vec2& p, vec2& q, bool x) {
//bool Trajectory::flc_breakpoint_V(const Float length, const Segment& start_segment, const Segment& start_segment, const Segment& end_segment, vec2& p, vec2& q, bool x) const
//{
//    try {
//        Float distanceBetweenEdges = (s3->time - s2->time) * this->getTotalLength();
//        Float remainingLength = length - distanceBetweenEdges;
//        vec2 startVector = *s1 - *s2;
//        vec2 endVector = *s4 - *s3;
//        vec2 startEndDifference = *s2 - *s3;
//
//        Float startDiff = (x) ? s2->getX() - s1->getX() : s2->getY() - s1->getY();
//        Float endDiff = (x) ? s3->getX() - s4->getX() : s3->getY() - s4->getY();
//        Float startLength = s1->calculateDistance(s2);
//        Float endLength = s3->calculateDistance(s4);
//
//        if (startDiff >= 0) {
//            startLength *= -1;
//        }
//
//        if (endDiff >= 0) {
//            endLength *= -1;
//        }
//
//        Float determinant = startDiff * endLength - -endDiff * startLength;
//
//        if (determinant == 0) {
//            return false;
//        }
//
//        Float a = startDiff;
//        Float b = -endDiff;
//        Float c = startLength;
//        Float d = endLength;
//
//        Float e = (x) ? startEndDifference.getX() : startEndDifference.getY();
//        Float f = remainingLength;
//
//        Float lambda = (e * d - b * f) / determinant;
//        Float rho = (a * f - e * c) / determinant;
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

bool Trajectory::flc_breakpoint_V_x(const Segment_Search_Tree& tree, const Float length, const Segment& start_segment, const Segment& end_segment, AABB& potential_hotspot) const
{
    if (!start_segment.x_overlap(end_segment))
    {
        return false;
    }

    Vec2 start_vector = start_segment.start - start_segment.end;
    Vec2 end_vector = end_segment.end - end_segment.start;

    Float determinant_x = start_vector.x * end_vector.length() - end_vector.x * start_vector.length();

    //TODO: Determinant perpendicular?
    if (determinant_x == 0.f)
    {
        return false;
    }

    //TODO: This works for now with using time (start_t & end_t) because time == length, but we might want to change the tree to also store the lengths..
    Float edge_distance = (end_segment.start_t - start_segment.end_t) * trajectory_length;
    Float remaining_length = length - edge_distance;

    Float start_length = start_vector.length();
    Float end_length = end_vector.length();

    Float start_end_difference = start_segment.end.x - end_segment.start.x;

    //Calculate the scalar for the vectors pointing to points p and q
    Float lambda = ((start_end_difference * end_length) - (-end_vector.x * remaining_length)) / determinant_x;
    Float rho = ((start_vector.x * remaining_length) - (start_end_difference * start_length)) / determinant_x;

    //Return false if p or q does not lie on their respective segment
    if (lambda < 0.f || lambda > 1.0f || rho < 0.f || rho > 1.0f)
    {
        return false;
    }

    //Calculate start and end point using the scalars
    Vec2 p = start_segment.end + (lambda * start_vector);
    Vec2 q = end_segment.start + (rho * end_vector);

    //Query the tree using the found start and end point
    potential_hotspot = tree.query(start_segment.get_time_at_point(p), end_segment.get_time_at_point(q));

    return true;
}

bool Trajectory::flc_breakpoint_V_y(const Segment_Search_Tree& tree, const Float length, const Segment& start_segment, const Segment& end_segment, AABB& potential_hotspot) const
{
    if (!start_segment.y_overlap(end_segment))
    {
        return false;
    }

    Vec2 start_vector = start_segment.start - start_segment.end;
    Vec2 end_vector = end_segment.end - end_segment.start;

    Float determinant_y = start_vector.y * end_vector.length() - end_vector.y * start_vector.length();

    //TODO: Determinant perpendicular?
    if (determinant_y == 0.f)
    {
        return false;
    }

    //TODO: This works for now with using time because time == length, but we might want to change the tree to also store the lengths..
    Float edge_distance = (end_segment.start_t - start_segment.end_t) * trajectory_length;
    Float remaining_length = length - edge_distance;

    Float start_length = start_vector.length();
    Float end_length = end_vector.length();

    Float start_end_difference = start_segment.end.y - end_segment.start.y;

    //Calculate the scalar for the vectors pointing to points p and q
    Float lambda = ((start_end_difference * end_length) - (-end_vector.y * remaining_length)) / determinant_y;
    Float rho = ((start_vector.y * remaining_length) - (start_end_difference * start_length)) / determinant_y;

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