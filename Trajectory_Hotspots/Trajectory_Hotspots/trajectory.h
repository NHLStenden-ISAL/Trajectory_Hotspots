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

    std::vector<Segment> trajectory_segments;
};