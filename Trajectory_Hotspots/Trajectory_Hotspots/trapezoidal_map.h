#pragma once

class Trapezoidal_Cell
{
public:

    Trapezoidal_Cell();

    Trapezoidal_Cell* top_left;
    Trapezoidal_Cell* top_right;
    Trapezoidal_Cell* bottom_left;
    Trapezoidal_Cell* bottom_right;


    Segment* left_segment;
    Segment* right_segment;

    Vec2* top_point;
    Vec2* bottom_point;
};

class Trapezoidal_Node
{
public:

    Trapezoidal_Node() = default;

    virtual Vec2 query(const Vec2& point, const bool direction) const = 0;

};

//Trapezoidal leaf nodes point to a cell of the trapezoidal map
class Trapezoidal_Leaf_Node : public Trapezoidal_Node
{
public:

    Trapezoidal_Leaf_Node();
    Trapezoidal_Leaf_Node(Segment& left_border, Segment& right_border, Vec2& top_point, Vec2& bottom_point);


    Trapezoidal_Cell trapezoidal_cell;
};

//Trapezoidal X nodes represent segments of the trapezoidal map
//It tests if the query point lies left or right of the segment
class Trapezoidal_X_Node : public Trapezoidal_Node
{
public:

    Trapezoidal_X_Node() : Trapezoidal_Node()
    {

    }
};

//Trapezoidal Y nodes represent points of the trapezoidal map
//It tests if the query point lies above or below of the point
class Trapezoidal_Y_Node : public Trapezoidal_Node
{
public:

    Trapezoidal_Y_Node() : Trapezoidal_Node()
    {

    }
};

class Trapezoidal_Map
{
public:
    Trapezoidal_Map();

    Trapezoidal_Map(std::vector<Segment> trajectory_segments);

    Vec2 query(const Vec2& point, const bool direction);

    Segment left_border;
    Segment right_border;

    Vec2 top_point;
    Vec2 bottom_point;

    std::unique_ptr<Trapezoidal_Node> root;
};