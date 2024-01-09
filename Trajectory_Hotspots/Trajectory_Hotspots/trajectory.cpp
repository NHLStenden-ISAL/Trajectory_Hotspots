#include "pch.h"
#include "trajectory.h"

Trajectory::Trajectory(const std::vector<Segment>& ordered_segments) : trajectory_segments(ordered_segments)
{
    trajectory_start = ordered_segments.front().start_t;
    trajectory_end = ordered_segments.back().end_t;

    for (const Segment& i : ordered_segments)
    {
        trajectory_length += i.length();
    }
}

Trajectory::Trajectory(const std::vector<Vec2>& ordered_trajectory_points)
{
    trajectory_segments.reserve(ordered_trajectory_points.size() - 1);

    //Create trajectory edges, set t with lengths
    Float start_t = 0.f;
    for (size_t i = 0; i < ordered_trajectory_points.size() - 1; i++)
    {
        trajectory_segments.emplace_back(ordered_trajectory_points[i], ordered_trajectory_points[i + 1], start_t);
        start_t += trajectory_segments.back().length();
    }

    trajectory_start = trajectory_segments.front().start_t;
    trajectory_end = trajectory_segments.back().end_t;

    trajectory_length = start_t;
}

const std::vector<Segment>& Trajectory::get_ordered_trajectory_segments() const
{
    return trajectory_segments;
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
    Float longest_valid_subtrajectory(0.f);
    AABB optimal_hotspot;

    //Setup trapezoidal maps representing the graphs with the x or y coördinates on the y-axis and time on the x-axis
    std::vector<Segment> x_segments;
    std::vector<Segment> y_segments;

    for (auto& trajectory_segment : trajectory_segments)
    {
        x_segments.emplace_back(Vec2(trajectory_segment.start_t, trajectory_segment.start.x), Vec2(trajectory_segment.end_t, trajectory_segment.end.x), trajectory_segment.start_t, trajectory_segment.end_t);
        y_segments.emplace_back(Vec2(trajectory_segment.start_t, trajectory_segment.start.y), Vec2(trajectory_segment.end_t, trajectory_segment.end.y), trajectory_segment.start_t, trajectory_segment.end_t);
    }

    Trapezoidal_Map trapezoidal_map_x(x_segments);
    Trapezoidal_Map trapezoidal_map_y(y_segments);

    //Setup segment search tree
    Segment_Search_Tree segment_tree(trajectory_segments);

    //Loop through all vertices and query trapezoidal maps and the segment search tree
    for (size_t i = 0; i < trajectory_segments.size(); i++)
    {
        Vec2 current_x_vert = x_segments[i].start;
        Vec2 current_y_vert = y_segments[i].start;

        //We slightly diverge from the paper here by tracing left and right from both the vector and vector + or - radius and taking the shortest of both.
        //instead of going up from the found point on the left and tracing back. 
        //This gives the same answer and we remove a number of checks.

        //Test vertical lines
        //Test left, current vert is at top
        Vec2 vert_at_radius(current_x_vert.x, current_x_vert.y - radius); //TODO: Remove, can calc in function..
        frc_test_between_lines(trapezoidal_map_x, current_x_vert, vert_at_radius, true, segment_tree, radius, longest_valid_subtrajectory, optimal_hotspot);

        //Test right, current vert is at bottom
        vert_at_radius = Vec2(current_x_vert.x, current_x_vert.y + radius);
        frc_test_between_lines(trapezoidal_map_x, current_x_vert, vert_at_radius, false, segment_tree, radius, longest_valid_subtrajectory, optimal_hotspot);

        //Test horizontal lines
        //Test below, current vert is at top
        vert_at_radius = Vec2(current_y_vert.x, current_y_vert.y - radius);
        frc_test_between_lines(trapezoidal_map_y, current_y_vert, vert_at_radius, true, segment_tree, radius, longest_valid_subtrajectory, optimal_hotspot);

        //Test above, current vert is at bottom
        vert_at_radius = Vec2(current_y_vert.x, current_y_vert.y + radius);
        frc_test_between_lines(trapezoidal_map_y, current_y_vert, vert_at_radius, false, segment_tree, radius, longest_valid_subtrajectory, optimal_hotspot);
    }

    //TODO: Don't forget last point? Or can we skip?
    //TODO: Remove bool?

    return optimal_hotspot;
}

void Trajectory::frc_test_between_lines(Trapezoidal_Map& trapezoidal_map, Vec2& current_vert, Vec2& vert_at_radius, bool above, Segment_Search_Tree& segment_tree, Float& radius, Float& longest_valid_subtrajectory, AABB& optimal_hotspot) const
{
    Float subtrajectory_start;
    Float subtrajectory_end;

    frc_get_subtrajectory_within_boundary(trapezoidal_map, current_vert, vert_at_radius, above, subtrajectory_start, subtrajectory_end);


    if ((subtrajectory_end - subtrajectory_start) > longest_valid_subtrajectory)
    {
        AABB subtrajectory_bounding_box = segment_tree.query(subtrajectory_start, subtrajectory_end);

        if (subtrajectory_bounding_box.max_size() <= radius)
        {
            longest_valid_subtrajectory = (subtrajectory_end - subtrajectory_start);
            optimal_hotspot = subtrajectory_bounding_box;
        }
    }
}

void Trajectory::frc_get_subtrajectory_within_boundary(const Trapezoidal_Map& trapezoidal_map, const Vec2& current_vert, const Vec2& vert_at_radius, const bool above_point, Float& subtrajectory_start, Float& subtrajectory_end) const
{
    //Query the start and end of the subtrajectory from the vertex and at the point one radius away.
    Float start_at_vert;
    Float end_at_vert;

    Float start_at_radius;
    Float end_at_radius;

    frc_get_subtrajectory_start_and_end(trapezoidal_map, current_vert, above_point, start_at_vert, end_at_vert);
    frc_get_subtrajectory_start_and_end(trapezoidal_map, vert_at_radius, !above_point, start_at_radius, end_at_radius);

    //Keep the shortest start and end, the trajectory leaves the boundary at that point so the subtrajectory towards the longer intersection will leave the boundary.
    subtrajectory_start = (start_at_vert < start_at_radius) ? start_at_vert : start_at_radius;
    subtrajectory_end = (end_at_vert < end_at_radius) ? end_at_vert : end_at_radius;
}

void Trajectory::frc_get_subtrajectory_start_and_end(const Trapezoidal_Map& trapezoidal_map, const Vec2& query_vert, const bool above_point, Float& subtrajectory_start, Float& subtrajectory_end) const
{
    const Segment* left_segment = nullptr;
    const Segment* right_segment = nullptr;
    trapezoidal_map.trace_left_right(query_vert, above_point, left_segment, right_segment);

    if (left_segment != nullptr && right_segment != nullptr)
    {
        //If the left_segment is at infinity there is no trajectory on the left, set start_t to the start of the trajectory
        if (left_segment->start.x.is_inf())
        {
            subtrajectory_start = this->trajectory_start;
        }
        else
        {
            subtrajectory_start = left_segment->get_time_at_y(query_vert.y);
        }

        //If the right_segment is at infinity there is no trajectory on the right, set end_t to the end of the trajectory
        if (right_segment->start.x.is_inf())
        {
            subtrajectory_end = this->trajectory_end;
        }
        else
        {
            subtrajectory_end = right_segment->get_time_at_y(query_vert.y);
        }
    }
    else
    {
        //Neither segment should ever be nullptr
        assert(false);
    }
}

//Find the smallest hotspot that contains a subtrajectory with at least the given length inside of it
AABB Trajectory::get_hotspot_fixed_length_contiguous(Float length) const
{
    if (length > trajectory_length)
    {
        //Invalid length
        return AABB();
    }

    //TODO:Check if length is enough for an UV to exist..
    Segment_Search_Tree tree(trajectory_segments);

    AABB smallest_hotspot(
        std::numeric_limits<float>::lowest() / 2.f,
        std::numeric_limits<float>::lowest() / 2.f,
        std::numeric_limits<float>::max() / 2.f,
        std::numeric_limits<float>::max() / 2.f);

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

    for (size_t start_index = 0; start_index < trajectory_segments.size(); ++start_index)
    {
        const Segment& start_segment = trajectory_segments[start_index];

        //Find all segments in which the sub-trajectories starting on this segment end
        const Float end_range_start = start_segment.start_t + length;
        const Float end_range_end = start_segment.end_t + length;

        const int end_range_start_index = tree.query(end_range_start);
        const int end_range_end_index = tree.query(end_range_end);

        //Get u, the time at the first vertex after the sub-trajectory start point
        Float start = start_segment.end_t;

        //Loop through all the possible end segments and query for an AABB of the sub-trajectory between u and v
        //Check breakpoints III, IV, and V
        for (size_t end_index = end_range_start_index; end_index <= end_range_end_index; ++end_index)
        {
            if (end_index - start_index < 1)
            {
                //Skip if same segment (No U & V)
                continue;
            }

            AABB current_hotspot;

            const Segment& end_segment = trajectory_segments[end_index];

            //Breakpoint V, the start and end of the subtrajectory lie on the same x or y coordinate
            if (flc_breakpoint_V(tree, length, start_segment, end_segment, true, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_V(tree, length, start_segment, end_segment, false, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }

            if (end_index - start_index < 2)
            {
                //Skip III & IV if connected segments
                //U & V are the same point (so no bounding box)
                continue;
            }


            //Get v, the time at the first vertex before the sub-trajectory end point
            Float end = end_segment.start_t;

            //Obtain the bounding box of the subtrajectory between u and v
            AABB uv_bounding_box = tree.query(start, end);

            //Breakpoints III and IV, Check if any of the four sides of the AABB of the subtrajectory between u and v intersects either the start or end segment, if so, check for new hotspot
            if (flc_breakpoint_III_x(length, start_segment, end_segment, uv_bounding_box.min.x, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_III_x(length, start_segment, end_segment, uv_bounding_box.max.x, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_III_y(length, start_segment, end_segment, uv_bounding_box.min.y, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_III_y(length, start_segment, end_segment, uv_bounding_box.max.y, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }

            if (flc_breakpoint_IV_x(length, start_segment, end_segment, uv_bounding_box.min.x, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_x(length, start_segment, end_segment, uv_bounding_box.max.x, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_y(length, start_segment, end_segment, uv_bounding_box.min.y, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }
            if (flc_breakpoint_IV_y(length, start_segment, end_segment, uv_bounding_box.max.y, uv_bounding_box, current_hotspot)) { if (current_hotspot.max_size() < smallest_hotspot.max_size()) { smallest_hotspot = current_hotspot; } }

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

    //If the found point is the end point of the starting segment we can skip, this is just breakpoint type I
    if (start_segment.end.y == intersection_y)
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

    //If the found point is the end point of the starting segment we can skip, this is just breakpoint type I
    if (start_segment.end.x == intersection_x)
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

    //If the found point is the start point of the end segment we can skip, this is just breakpoint type II
    if (end_segment.start.y == intersection_y)
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

    //If the found point is the start point of the end segment we can skip, this is just breakpoint type II
    if (end_segment.start.x == intersection_x)
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

bool Trajectory::flc_breakpoint_V(const Segment_Search_Tree& tree, const Float length, const Segment& start_segment, const Segment& end_segment, const bool axis, AABB& potential_hotspot) const
{
    //Calculate start and end point
    Vec2 p;
    Vec2 q;

    const bool found_valid_points = Segment::get_points_on_same_axis_with_distance_l(start_segment, end_segment, length, axis, p, q);

    if (!found_valid_points)
    {
        return false;
    }

    //Query the tree using the found start and end point
    potential_hotspot = tree.query(start_segment.get_time_at_point(p), end_segment.get_time_at_point(q));

    return true;
}
