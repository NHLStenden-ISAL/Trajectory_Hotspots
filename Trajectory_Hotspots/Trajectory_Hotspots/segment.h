#pragma once

class Vec2;


class Segment
{
public:
    Segment(const Vec2 start, const Vec2 end) : start(start), end(end)
    {

    }

    Segment(const float start_x, const float start_y, const float end_x, const float end_y) : start(start_x, start_y), end(end_x, end_y)
    {

    }

    float length() const;
    float squared_length() const;

    Vec2 start;
    Vec2 end;
};