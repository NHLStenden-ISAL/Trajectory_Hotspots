#pragma once

class Vec2;
class AABB;

class Segment
{
public:

    Segment() : start(Vec2(0.f, 0.f)), end(Vec2(0.f, 1.f)), start_t(0.f), end_t(1.f)
    {

    }

    Segment(const Vec2 start, const Vec2 end, const Float start_t, const Float end_t) :
        start(start), end(end), start_t(start_t), end_t(end_t)
    {

    }

    Segment(const Vec2 start, const Vec2 end, const Float start_t) :
        start(start), end(end), start_t(start_t)
    {
        end_t = start_t + this->length();
    }

    Segment(const Vec2 start, const Vec2 end) :
        start(start), end(end), start_t(0.f), end_t(0.f)
    {
    }

    Segment(const Float start_x, const Float start_y, const Float end_x, const Float end_y, const Float start_t, const Float end_t) :
        start(start_x, start_y), end(end_x, end_y), start_t(start_t), end_t(end_t)
    {

    }

    bool operator==(const Segment& operand) const;
    bool operator!=(const Segment& operand) const;

    Float length() const;
    Float squared_length() const;

    //Returns the y-coordinate of the intersection with the vertical line at x, or infinity if it lies on the segment
    Float x_intersect(Float x) const;
    //Returns the x-coordinate of the intersection with the horizontal line at y, or infinity if it lies on the segment
    Float y_intersect(Float y) const;

    //If the vertical line at position x intersects the segment, gets the y coordinate of the intersection
    bool x_intersects(const Float x, Float& intersection_point_y) const;
    //If the horizontal line at position y intersects the segment, gets the x coordinate of the intersection
    bool y_intersects(const Float y, Float& intersection_point_x) const;

    Float get_time_at_x(const Float x) const;
    Float get_time_at_y(const Float y) const;
    Float get_time_at_point(const Vec2& point) const;

    const Vec2* get_bottom_point() const;
    const Vec2* get_top_point() const;

    //Determines if two segments share a y-axis
    bool x_overlap(const Segment& segment) const;
    //Determines if two segments share a x-axis
    bool y_overlap(const Segment& segment) const;

    AABB get_AABB() const;

    Vec2 start;
    Vec2 end;

    //TODO: Not sure I like this in here.. can we move this to the tree?
    Float start_t;
    Float end_t;
};

//Determine if a point lies to the left or right of a segment, oriented from start to end
//If the point lies on the segment this function will return true (right)
bool point_right_of_segment(const Segment& segment, const Vec2& point);