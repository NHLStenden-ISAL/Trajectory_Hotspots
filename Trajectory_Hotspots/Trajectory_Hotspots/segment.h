#pragma once

class Vec2;
class AABB;

class Segment
{
public:

    Segment() : start(Vec2(0.f, 0.f)), end(Vec2(0.f, 1.f)), start_t(0.f), end_t(1.f)
    {

    }

    Segment(const Vec2 start, const Vec2 end, const float start_t, const float end_t) :
        start(start), end(end), start_t(start_t), end_t(end_t)
    {

    }

    Segment(const Vec2 start, const Vec2 end, const float start_t) :
        start(start), end(end), start_t(start_t)
    {
        end_t = start_t + this->length();
    }

    Segment(const Vec2 start, const Vec2 end) :
        start(start), end(end), start_t(0.f), end_t(0.f)
    {
    }

    Segment(const float start_x, const float start_y, const float end_x, const float end_y, const float start_t, const float end_t) :
        start(start_x, start_y), end(end_x, end_y), start_t(start_t), end_t(end_t)
    {

    }

    bool operator==(const Segment& operand) const;
    bool operator!=(const Segment& operand) const;

    float length() const;
    float squared_length() const;

    //Returns the y-coordinate of the intersection with the vertical line at x, or infinity if it lies on the segment
    float x_intersect(float x) const;
    //Returns the x-coordinate of the intersection with the horizontal line at y, or infinity if it lies on the segment
    float y_intersect(float y) const;

    //If the vertical line at position x intersects the segment, gets the y coordinate of the intersection
    bool x_intersects(float x, float& intersection_point_y) const;
    //If the horizontal line at position y intersects the segment, gets the x coordinate of the intersection
    bool y_intersects(float y, float& intersection_point_x) const;

    float get_time_at_x(const float x) const;
    float get_time_at_y(const float y) const;

    const Vec2* get_bottom_point() const;
    const Vec2* get_top_point() const;

    //Determines if two segments share a y-axis
    bool x_overlap(const Segment& segment) const;
    //Determines if two segments share a x-axis
    bool y_overlap(const Segment& segment) const;

    AABB get_AABB() const;

    Vec2 start;
    Vec2 end;

    //TODO: Not sure I like this in here.. can we move this to a wrapper class?
    float start_t;
    float end_t;
};

//Determine if a point lies to the left or right of a segment, oriented from start to end
//If the point lies on the segment this function will return true (right)
bool point_right_of_segment(const Segment& segment, const Vec2& point);