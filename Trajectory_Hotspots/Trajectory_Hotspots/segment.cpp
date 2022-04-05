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

//Determine if a point lies to the left or right of a segment, oriented from start to end
//If the point lies on the segment this function will return true (right)
bool point_right_of_segment(const Segment& segment, const Vec2& point)
{
    assert(segment.start.y < segment.end.y);

    float direction = (point.x - segment.start.x) * (segment.end.y - segment.start.y) - (point.y - segment.start.y) * (segment.end.x - segment.start.x);

    return direction < 0 ? false : true;
}