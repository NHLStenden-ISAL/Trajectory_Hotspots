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

const Vec2* Segment::get_bottom_point() const
{
    if (start.y > end.y)
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
    else
    {
        return &end;
    }

}

//Check if segments share a y-axis
bool Segment::x_overlap(const Segment& other_segment) const
{
    //sort the endpoints
    const Vec2& left_vert = (start.x <= end.x ? start : end);
    const Vec2& right_vert = (start.x > end.x ? start : end);

    const Vec2& other_left_vert = (other_segment.start.x <= other_segment.end.x ? other_segment.start : other_segment.end);
    const Vec2& other_right_vert = (other_segment.start.x > other_segment.end.x ? other_segment.start : other_segment.end);

    return !(right_vert.x < other_left_vert.x || left_vert.x > other_right_vert.x);
}

//Check if segments share a x-axis
bool Segment::y_overlap(const Segment& other_segment) const
{
    //sort the endpoints
    const Vec2& lower_vert = (start.y <= end.y ? start : end);
    const Vec2& upper_vert = (start.y > end.y ? start : end);

    const Vec2& other_lower_vert = (other_segment.start.y <= other_segment.end.y ? other_segment.start : other_segment.end);
    const Vec2& other_upper_vert = (other_segment.start.y > other_segment.end.y ? other_segment.start : other_segment.end);

    return !(upper_vert.y < other_lower_vert.y || lower_vert.y > other_upper_vert.y);
}

AABB Segment::get_AABB() const
{
    return AABB(std::min(start.x, end.x), std::min(start.y, end.y), std::max(start.x, end.x), std::max(start.y, end.y));
}


//Returns the y-coordinate of the intersection with the vertical line at x, or infinity if it lies on the segment
float Segment::x_intersect(float x) const
{
    float x_diff = end.x - start.x;

    //Vertical segment, either no or infinite intersections
    if (x_diff == 0.f)
    {
        return std::numeric_limits<float>::infinity();
    }
    else
    {
        float segment_to_x = x - start.x;
        float intersection_y = start.y + (segment_to_x / x_diff * (end.y - start.y));

        return intersection_y;
    }
}

//Returns the x-coordinate of the intersection with the horizontal line at y, or infinity if it lies on the segment
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

//If the vertical line at position x intersects the segment, gets the y coordinate of the intersection
bool Segment::x_intersects(float x, float& intersection_point_y) const
{
    //If the query line is left or right of segment, return false
    if (x < start.x && x < end.x)
    {
        return false;
    }
    else if (x > start.x && x > end.x)
    {
        return false;
    }

    //Intersection, get intersection point and return true
    intersection_point_y = x_intersect(x);

    return true;
}

//If the horizontal line at position y intersects the segment, gets the x coordinate of the intersection
bool Segment::y_intersects(float y, float& intersection_point_x) const
{
    //If the query line is above or below segment, return false
    if (y < start.y && y < end.y)
    {
        return false;
    }
    else if (y > start.y && y > end.y)
    {
        return false;
    }

    //Intersection, get intersection point and return true
    intersection_point_x = y_intersect(y);

    return true;
}

float Segment::get_time_at_x(const float x) const
{
    float x_fraction = (x - start.x) / (end.x - start.x);
    return x_fraction * (end_t - start_t);
}

float Segment::get_time_at_y(const float y) const
{
    float y_fraction = (y - start.y) / (end.y - start.y);
    return y_fraction * (end_t - start_t);
}

float Segment::get_time_at_point(const Vec2& point) const
{
    Vec2 start_to_point = point - start;
    float time_fraction = start_to_point.length() / length();
    return time_fraction * (end_t - start_t);
}

//Determine if a point lies to the left (false) or right (true) of a segment, oriented from start to end
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