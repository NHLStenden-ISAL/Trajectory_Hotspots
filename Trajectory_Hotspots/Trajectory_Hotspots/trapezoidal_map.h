#pragma once

class Trapezoidal_Leaf_Node;
class Trapezoidal_Internal_Node;

class Trapezoidal_Node
{
public:

    Trapezoidal_Node() = default;

    virtual Trapezoidal_Leaf_Node* query_point(const Vec2& point) = 0;
    virtual Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment) = 0;

    std::vector<Trapezoidal_Internal_Node*> parents;
};

class Trapezoidal_Internal_Node : public Trapezoidal_Node
{
public:
    Trapezoidal_Internal_Node() = default;

    virtual void replace_child(Trapezoidal_Node* old_child, std::shared_ptr<Trapezoidal_Node> new_child) = 0;

};

//Trapezoidal leaf nodes point to a cell of the trapezoidal map
class Trapezoidal_Leaf_Node : public Trapezoidal_Node
{
public:

    Trapezoidal_Leaf_Node();
    Trapezoidal_Leaf_Node(const Segment* left_border, const Segment* right_border, const Vec2* bottom_point, const Vec2* top_point);
    Trapezoidal_Leaf_Node(const Segment* left_border, const Segment* right_border, const Vec2* bottom_point, const Vec2* top_point, Trapezoidal_Leaf_Node* bottom_left, Trapezoidal_Leaf_Node* bottom_right, Trapezoidal_Leaf_Node* top_left, Trapezoidal_Leaf_Node* top_right);

    Trapezoidal_Leaf_Node* query_point(const Vec2& point);
    Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment);

    void set_neighbour_pointers(Trapezoidal_Leaf_Node* bottom_left, Trapezoidal_Leaf_Node* bottom_right, Trapezoidal_Leaf_Node* top_left, Trapezoidal_Leaf_Node* top_right);

    void replace_bottom_neighbour(Trapezoidal_Leaf_Node* old_bottom_neighbour, Trapezoidal_Leaf_Node* new_bottom_neighbour);
    void replace_top_neighbour(Trapezoidal_Leaf_Node* old_top_neighbour, Trapezoidal_Leaf_Node* new_top_neighbour);

public:


    Trapezoidal_Leaf_Node* bottom_left;
    Trapezoidal_Leaf_Node* bottom_right;
    Trapezoidal_Leaf_Node* top_left;
    Trapezoidal_Leaf_Node* top_right;

    const Segment* left_segment;
    const Segment* right_segment;

    const Vec2* top_point;
    const Vec2* bottom_point;
};

//Trapezoidal X nodes represent segments of the trapezoidal map
//It tests if the query point lies left or right of the segment
class Trapezoidal_X_Node : public Trapezoidal_Internal_Node
{
public:

    Trapezoidal_X_Node() : Trapezoidal_Internal_Node(), segment(nullptr), left(nullptr), right(nullptr)
    {
    }

    Trapezoidal_X_Node(const Segment* segment, std::shared_ptr<Trapezoidal_Node> left, std::shared_ptr<Trapezoidal_Node> right) : segment(segment), left(left), right(right)
    {
    }

    Trapezoidal_Leaf_Node* query_point(const Vec2& point);
    Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment);

    void replace_child(Trapezoidal_Node* old_child, std::shared_ptr<Trapezoidal_Node> new_child);

    const Segment* segment;

    std::shared_ptr<Trapezoidal_Node> left;
    std::shared_ptr<Trapezoidal_Node> right;
};

//Trapezoidal Y nodes represent points of the trapezoidal map
//It tests if the query point lies above or below of the point
class Trapezoidal_Y_Node : public Trapezoidal_Internal_Node
{
public:

    Trapezoidal_Y_Node() : Trapezoidal_Internal_Node(), point(nullptr), below(nullptr), above(nullptr)
    {

    }

    Trapezoidal_Y_Node(const Vec2* point) : Trapezoidal_Internal_Node(), point(point), below(nullptr), above(nullptr)
    {

    }

    Trapezoidal_Y_Node(const Vec2* point, std::shared_ptr<Trapezoidal_Node> below, std::shared_ptr<Trapezoidal_Node> above) : Trapezoidal_Internal_Node(), point(point), below(below), above(above)
    {

    }

    Trapezoidal_Leaf_Node* query_point(const Vec2& point);
    Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment);

    void replace_child(Trapezoidal_Node* old_child, std::shared_ptr<Trapezoidal_Node> new_child);

    const Vec2* point;

    //Trapezoidal_Internal_Node* parent;

    std::shared_ptr<Trapezoidal_Node> below;
    std::shared_ptr<Trapezoidal_Node> above;
};

class Trapezoidal_Map
{
public:
    Trapezoidal_Map();

    Trapezoidal_Map(std::vector<Segment> trajectory_segments);

    Trapezoidal_Leaf_Node* query_point(const Vec2& point);

    void add_segment(const Segment& segment);
    void add_fully_embedded_segment(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment);
    void add_fully_embedded_segment_with_both_endpoints_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment);
    void add_fully_embedded_segment_with_top_endpoint_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment);
    void add_fully_embedded_segment_with_bottom_endpoint_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment);
    void add_overlapping_segment(std::vector<Trapezoidal_Leaf_Node*> overlapping_trapezoids, const Segment& segment);
    //const Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment) const;

private:

    std::vector<Trapezoidal_Leaf_Node*> follow_segment(const Segment& query_segment);

    void replace_leaf_node_with_subgraph(Trapezoidal_Leaf_Node* old_trapezoid, std::shared_ptr<Trapezoidal_Internal_Node> new_subgraph);

public:

    int segment_count;

    Segment left_border;
    Segment right_border;

    Vec2 top_point;
    Vec2 bottom_point;

    std::shared_ptr<Trapezoidal_Node> root;
};