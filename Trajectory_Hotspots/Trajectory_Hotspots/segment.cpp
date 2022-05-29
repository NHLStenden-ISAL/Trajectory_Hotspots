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

float Segment::y_intersect(float y) const
{
    float y_diff = end.y - start.y;

    //Horizontal segment, either no or infinite intersections
    if (y_diff == 0.f)
    {
        return std::numeric_limits<float>::infinity();
    }
    else
    {
        float segment_to_y = y - start.y;
        float intersection_x = start.x + (segment_to_y / y_diff * (end.x - start.x));

        return intersection_x;
    }
}

const Vec2* Segment::get_bottom_point() const
{
    if (start.y > end.y)
    {
        return &end;
    }
    else if (start.y == end.y && start.x > end.x)
    {
        return &end;
    }
    else
    {
        return &start;
    }

}

const Vec2* Segment::get_top_point() const
{
    if (start.y > end.y)
    {
        return &start;
    }
    else if (start.y == end.y && start.x > end.x)
    {
        return &start;
    }
    else
    {
        return &end;
    }

}

Vec2 Segment::to_vector() const
{
    return Vec2(end - start);
}

AABB Segment::get_AABB() const
{
    return AABB(std::min(start.x, end.x), std::min(start.y, end.y), std::max(start.x, end.x), std::max(start.y, end.y));
}

//Determine if a point lies to the left or right of a segment, oriented from start to end
//If the point lies on the segment this function will return true (right)
bool point_right_of_segment(const Segment& segment, const Vec2& point)
{
    //TODO: Force end always top so we can remove this check?
    const Vec2* top = segment.get_top_point();
    const Vec2* bottom = segment.get_bottom_point();

    float direction = (point.x - bottom->x) * (top->y - bottom->y) - (point.y - bottom->y) * (top->x - bottom->x);

    return direction < 0 ? false : true;
}

bool Segment::operator==(const Segment& operand) const
{
    return (start == operand.start && end == operand.end) || (end == operand.start && start == operand.end);
}

bool Segment::operator!=(const Segment& operand) const
{
    return !(*this == operand);;
}


