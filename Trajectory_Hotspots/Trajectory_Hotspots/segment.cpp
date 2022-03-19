#include "pch.h"
#include "vec2.h"
#include "aabb.h"
#include "segment.h"


float Segment::length() const
{
    Vec2 distance_vector = start - end;
    return distance_vector.length();
}

float Segment::squared_length() const
{
    Vec2 distance_vector = start - end;
    return distance_vector.squared_length();
}

AABB Segment::get_AABB() const
{
    return AABB(std::min(start.x, end.x), std::min(start.y, end.y), std::max(start.x, end.x), std::max(start.y, end.y));
}
