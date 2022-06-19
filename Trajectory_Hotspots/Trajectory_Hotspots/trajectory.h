#pragma once

//TODO: Pre-build trees for fast queries?
//TODO: Move tree into this class to make AABB query easier
class Trajectory
{
public:

    Trajectory(std::vector<Segment>& ordered_segments);


    AABB get_hotspot_fixed_radius(float radius) const;
    AABB get_hotspot_fixed_length(float length) const;

    AABB get_hotspot_fixed_radius_contiguous(float radius) const;
    AABB get_hotspot_fixed_length_contiguous(float length) const;

private:

    float trajectory_end = 0.f;

    std::vector<Segment> trajectory_segments;

    //Helper functions for fixed_length_contiguous
    bool check_breakpoint_III_x(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_x, AABB& potential_hotspot) const;
    bool check_breakpoint_III_y(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_y, AABB& potential_hotspot) const;
    bool check_breakpoint_IV_x(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_x, AABB& potential_hotspot) const;
    bool check_breakpoint_IV_y(const Segment_Search_Tree& tree, const float length, const Segment& trajectory_segment, const float line_y, AABB& potential_hotspot) const;

};