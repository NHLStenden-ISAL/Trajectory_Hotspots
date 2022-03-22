#include "pch.h"
#include "trajectory.h"

Trajectory::Trajectory(std::vector<Segment>& ordered_segments) : trajectory_segments(ordered_segments)
{
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
    return AABB();
}