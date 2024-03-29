#pragma once

class Trapezoidal_Leaf_Node;
class Trapezoidal_Internal_Node;

class Trapezoidal_Node
{
public:

    Trapezoidal_Node() = default;

    //TODO: Shouldn't this be const?
    virtual Trapezoidal_Leaf_Node* query_point(const Vec2& point) = 0;
    virtual Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment) = 0;

    virtual void trace_left_right(const Vec2& point, const bool prefer_top, const Segment*& left_segment, const Segment*& right_segment) const = 0;

    std::vector<Trapezoidal_Internal_Node*> parents;

    friend class Trapezoidal_Node;
    friend class Trapezoidal_Leaf_Node;
    friend class Trapezoidal_X_Node;
    friend class Trapezoidal_Y_Node;

private:
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

    /// <summary>
    /// Trace a horizontal ray to the left and right finding the first segments that intersect it to the left and right of the query point.
    /// </summary>
    /// <param name="point">The queried point.</param>
    /// <param name="prefer_top">When the query point lies on an endpoint within the trapezoidal map, we trace either above or below based on this parameter. This prevents unwanted self intersections.</param>
    /// <param name="left_segment">The first segment to the left of the queried point.</param>
    /// <param name="right_segment">The first segment to the right of the queried point.</param>
    void trace_left_right(const Vec2& point, const bool prefer_top, const Segment*& left_segment, const Segment*& right_segment) const;

    Trapezoidal_Leaf_Node* bottom_left;
    Trapezoidal_Leaf_Node* bottom_right;
    Trapezoidal_Leaf_Node* top_left;
    Trapezoidal_Leaf_Node* top_right;

    const Segment* left_segment;
    const Segment* right_segment;

    const Vec2* top_point;
    const Vec2* bottom_point;

private:
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
        left->parents.push_back(this);
        right->parents.push_back(this);
    }

    Trapezoidal_Leaf_Node* query_point(const Vec2& point);
    Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment);

    void trace_left_right(const Vec2& point, const bool prefer_top, const Segment*& left_segment, const Segment*& right_segment) const;

    void replace_child(Trapezoidal_Node* old_child, std::shared_ptr<Trapezoidal_Node> new_child);

    const Segment* segment;

    std::shared_ptr<Trapezoidal_Node> left;
    std::shared_ptr<Trapezoidal_Node> right;

private:

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
        below->parents.push_back(this);
        above->parents.push_back(this);
    }

    Trapezoidal_Leaf_Node* query_point(const Vec2& point);
    Trapezoidal_Leaf_Node* query_start_point(const Segment& query_segment);

    void trace_left_right(const Vec2& point, const bool prefer_top, const Segment*& left_segment, const Segment*& right_segment) const;

    void replace_child(Trapezoidal_Node* old_child, std::shared_ptr<Trapezoidal_Node> new_child);

    const Vec2* point;

    std::shared_ptr<Trapezoidal_Node> below;
    std::shared_ptr<Trapezoidal_Node> above;

private:
};

class Trapezoidal_Map
{
public:
    Trapezoidal_Map();

    Trapezoidal_Map(const std::vector<Segment>& trajectory_segments, const unsigned int seed = 0, const bool randomized_construction = true);


    Trapezoidal_Leaf_Node* query_point(const Vec2& point);

    void add_segment(const Segment& segment);
    void add_fully_embedded_segment(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment);
    void add_fully_embedded_segment_with_both_endpoints_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment);
    void add_fully_embedded_segment_with_top_endpoint_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment);
    void add_fully_embedded_segment_with_bottom_endpoint_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment);
    void add_overlapping_segment(const std::vector<Trapezoidal_Leaf_Node*>& overlapping_trapezoids, const Segment& segment);

    void trace_left_right(const Vec2& point, const bool prefer_top, const Segment*& left_segment, const Segment*& right_segment) const;

private:

    std::vector<Trapezoidal_Leaf_Node*> follow_segment(const Segment& query_segment);

    void replace_leaf_node_with_subgraph(Trapezoidal_Leaf_Node* old_trapezoid, std::shared_ptr<Trapezoidal_Internal_Node> new_subgraph);

public:

    int segment_count = 0;

    Segment left_border;
    Segment right_border;

    Vec2 top_point;
    Vec2 bottom_point;

    std::shared_ptr<Trapezoidal_Node> root;
};