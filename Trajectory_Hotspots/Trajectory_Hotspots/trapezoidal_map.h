#pragma once


class Trapezoidal_Cell
{
public:


    Trapezoidal_Cell* left;
    Trapezoidal_Cell* right;
    Trapezoidal_Cell* top;
    Trapezoidal_Cell* bottom;
};


class Trapezoidal_Map
{
public:
    Trapezoidal_Map();

    Trapezoidal_Map(std::vector<Segment> trajectory_segments);

    //TODO: Not sure about vec2 or paramenters..
    //Vec2 Query();




};