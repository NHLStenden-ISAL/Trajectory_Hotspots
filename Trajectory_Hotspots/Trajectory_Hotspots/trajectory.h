#pragma once

//TODO: Pre-build trees for fast queries?
//TODO: Move tree into this class to make AABB query easier
class Trajectory
{
public:

    Trajectory(std::vector<Segment>& ordered_segments);
    Trajectory(const std::vector<Vec2>& ordered_points);


    AABB get_hotspot_fixed_radius(Float radius) const;
    AABB get_hotspot_fixed_length(Float length) const;

    AABB get_hotspot_fixed_radius_contiguous(Float radius) const;
    AABB get_hotspot_fixed_length_contiguous(Float length) const;

    const std::vector<Segment>& get_ordered_trajectory_segments() const;

private:

    Float trajectory_start = 0.f;
    Float trajectory_end = 0.f;
    Float trajectory_length = 0.f;

    std::vector<Segment> trajectory_segments;

    //Helper functions for fixed_radius_contiguous

    void frc_test_between_lines(Trapezoidal_Map & trapezoidal_map, Vec2 & current_vert, Vec2& vert_at_radius, bool above, Segment_Search_Tree& segment_tree, Float& radius, Float& longest_valid_subtrajectory, AABB& optimal_hotspot) const;
    void frc_get_subtrajectory_start_and_end(const Trapezoidal_Map& trapezoidal_map, const Vec2& current_vert, const Vec2& vert_at_radius, const bool above_point, Float& subtrajectory_start, Float& subtrajectory_end) const;

    //Helper functions for fixed_length_contiguous

    bool flc_breakpoint_III_x(const Float length, const Segment& start_segment, const Segment& end_segment, const Float vertical_line_x, const AABB& uv_bounding_box, AABB& potential_hotspot) const;
    bool flc_breakpoint_III_y(const Float length, const Segment& start_segment, const Segment& end_segment, const Float horizontal_line_y, const AABB& uv_bounding_box, AABB& potential_hotspot) const;
    bool flc_breakpoint_IV_x(const Float length, const Segment& start_segment, const Segment& end_segment, const Float vertical_line_x, const AABB& uv_bounding_box, AABB& potential_hotspot) const;
    bool flc_breakpoint_IV_y(const Float length, const Segment& start_segment, const Segment& end_segment, const Float horizontal_line_y, const AABB& uv_bounding_box, AABB& potential_hotspot) const;

    //bool flc_breakpoint_V(const Segment_Search_Tree& tree, const float length, const Segment& start_segment, const Segment& end_segment, AABB& potential_hotspot) const;
    bool flc_breakpoint_V(const Segment_Search_Tree& tree, const Float length, const Segment& start_segment, const Segment& end_segment, const bool axis, AABB& potential_hotspot) const;
};