#pragma once

//TODO: Pre-build trees for fast queries?
//TODO: Move tree into this class to make AABB query easier
class Trajectory
{
public:

    Trajectory(std::vector<Segment>& ordered_segments);


    AABB get_hotspot_fixed_radius(Float radius) const;
    AABB get_hotspot_fixed_length(Float length) const;

    AABB get_hotspot_fixed_radius_contiguous(Float radius) const;
    AABB get_hotspot_fixed_length_contiguous(Float length) const;

private:

    std::vector<Segment> trajectory_segments;
};