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

    const Vec2* get_bottom_point() const;
    const Vec2* get_top_point() const;

    AABB get_AABB() const;

    Vec2 start;
    Vec2 end;

    //TODO: Not sure I like this in here.. can we move this to the tree?
    Float start_t;
    Float end_t;
};

bool point_right_of_segment(const Segment& segment, const Vec2& point);