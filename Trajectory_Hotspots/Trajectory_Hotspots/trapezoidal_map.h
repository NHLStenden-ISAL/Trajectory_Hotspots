#pragma once

class Trapezoidal_Leaf_Node;

class Trapezoidal_Node
{
public:

    Trapezoidal_Node() = default;

    virtual Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment) = 0;

};

//Trapezoidal leaf nodes point to a cell of the trapezoidal map
class Trapezoidal_Leaf_Node : public Trapezoidal_Node
{
public:

    Trapezoidal_Leaf_Node();
    Trapezoidal_Leaf_Node(Segment& left_border, Segment& right_border, Vec2& bottom_point, Vec2& top_point);
    Trapezoidal_Leaf_Node(Segment& left_border, Segment& right_border, Vec2& bottom_point, Vec2& top_point, Trapezoidal_Leaf_Node* bottom_left, Trapezoidal_Leaf_Node* bottom_right, Trapezoidal_Leaf_Node* top_left, Trapezoidal_Leaf_Node* top_right);


    Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment);

    void set_neighbour_pointers(Trapezoidal_Leaf_Node* bottom_left, Trapezoidal_Leaf_Node* bottom_right, Trapezoidal_Leaf_Node* top_left, Trapezoidal_Leaf_Node* top_right);

public:

    std::vector<Trapezoidal_Node*> parents;

    Trapezoidal_Leaf_Node* top_left;
    Trapezoidal_Leaf_Node* top_right;
    Trapezoidal_Leaf_Node* bottom_left;
    Trapezoidal_Leaf_Node* bottom_right;


    const Segment* left_segment;
    const Segment* right_segment;

    const Vec2* top_point;
    const Vec2* bottom_point;
};

//Trapezoidal X nodes represent segments of the trapezoidal map
//It tests if the query point lies left or right of the segment
class Trapezoidal_X_Node : public Trapezoidal_Node
{
public:

    Trapezoidal_X_Node() : Trapezoidal_Node()
    {

    }

    Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment);

public:

    const Segment* segment;

    Trapezoidal_Node* parent;

    std::shared_ptr<Trapezoidal_Node> left;
    std::shared_ptr<Trapezoidal_Node> right;
};

//Trapezoidal Y nodes represent points of the trapezoidal map
//It tests if the query point lies above or below of the point
class Trapezoidal_Y_Node : public Trapezoidal_Node
{
public:

    Trapezoidal_Y_Node() : Trapezoidal_Node(), point(nullptr), parent(nullptr), below(nullptr), above(nullptr)
    {

    }

    Trapezoidal_Y_Node(const Vec2* point) : Trapezoidal_Node(), point(point), parent(nullptr), below(nullptr), above(nullptr)
    {

    }

    Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment);

public:

    const Vec2* point;

    Trapezoidal_Node* parent;

    std::shared_ptr<Trapezoidal_Node> below;
    std::shared_ptr<Trapezoidal_Node> above;
};

class Trapezoidal_Map
{
public:
    Trapezoidal_Map();

    Trapezoidal_Map(std::vector<Segment> trajectory_segments);
private:

    void add_segment(const Segment& segment);

    //const Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment) const;
    std::vector<Trapezoidal_Leaf_Node*> follow_segment(const Segment& query_segment);

public:

    Segment left_border;
    Segment right_border;

    Vec2 top_point;
    Vec2 bottom_point;

    std::unique_ptr<Trapezoidal_Node> root;
};