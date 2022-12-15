#include "pch.h"
#include "trajectory.h"

Trajectory::Trajectory(std::vector<Segment>& ordered_segments) : trajectory_segments(ordered_segments)
{
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
    return AABB();
}