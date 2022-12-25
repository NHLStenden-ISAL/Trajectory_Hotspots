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

private:

    Float trajectory_start = 0.f;
    Float trajectory_end = 0.f;
    Float trajectory_length = 0.f;

    std::vector<Segment> trajectory_segments;

    //Helper functions for fixed_length_contiguous
    bool flc_breakpoint_III_x(const Float length, const Segment& start_segment, const Segment& end_segment, const Float vertical_line_x, const AABB& uv_bounding_box, AABB& potential_hotspot) const;
    bool flc_breakpoint_III_y(const Float length, const Segment& start_segment, const Segment& end_segment, const Float horizontal_line_y, const AABB& uv_bounding_box, AABB& potential_hotspot) const;
    bool flc_breakpoint_IV_x(const Float length, const Segment& start_segment, const Segment& end_segment, const Float vertical_line_x, const AABB& uv_bounding_box, AABB& potential_hotspot) const;
    bool flc_breakpoint_IV_y(const Float length, const Segment& start_segment, const Segment& end_segment, const Float horizontal_line_y, const AABB& uv_bounding_box, AABB& potential_hotspot) const;

    //bool flc_breakpoint_V(const Segment_Search_Tree& tree, const float length, const Segment& start_segment, const Segment& end_segment, AABB& potential_hotspot) const;
    bool flc_breakpoint_V_x(const Segment_Search_Tree& tree, const Float length, const Segment& start_segment, const Segment& end_segment, AABB& potential_hotspot) const;
    bool flc_breakpoint_V_y(const Segment_Search_Tree& tree, const Float length, const Segment& start_segment, const Segment& end_segment, AABB& potential_hotspot) const;
};