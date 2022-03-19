#pragma once

class Vec2;


class Segment
{
public:
    Segment(Vec2 start, Vec2 end) : start(start), end(end)
    {

    }

    Segment(float start_x, float start_y, float end_x, float end_y) : start(start_x, start_y), end(end_x, end_y)
    {

    }

    float length() const;
    float squared_length() const;

    Vec2 start;
    Vec2 end;
};