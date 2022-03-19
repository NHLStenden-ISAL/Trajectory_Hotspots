#pragma once

class Vec2;
class AABB;

class Segment
{
public:
    Segment(const Vec2 start, const Vec2 end, const float start_t, const float end_t) :
        start(start), end(end), start_t(start_t), end_t(end_t)
    {

    }

    Segment(const float start_x, const float start_y, const float end_x, const float end_y, const float start_t, const float end_t) :
        start(start_x, start_y), end(end_x, end_y), start_t(start_t), end_t(end_t)
    {

    }

    float length() const;
    float squared_length() const;

    AABB get_AABB() const;

    Vec2 start;
    Vec2 end;

    //TODO: Not sure I like this in here.. can we move this to the tree?
    float start_t;
    float end_t;
};