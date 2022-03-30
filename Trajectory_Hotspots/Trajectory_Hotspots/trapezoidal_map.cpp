#include "pch.h"
#include "trapezoidal_map.h"

Trapezoidal_Map::Trapezoidal_Map()
{
    //TODO: Is there a better choice for this? Or do we want to always use infinity points for the bounding box?
    AABB bounding_box(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());

    left_border = Segment(bounding_box.min, Vec2(bounding_box.min.x, bounding_box.max.y), 0.f, 0.f);
    right_border = Segment(bounding_box.min, Vec2(bounding_box.min.x, bounding_box.max.y), 0.f, 0.f);

    top_point = bounding_box.max;
    bottom_point = bounding_box.min;

    std::unique_ptr<Trapezoidal_Node> root = std::make_unique<Trapezoidal_Leaf_Node>(left_border, right_border, top_point, bottom_point);
}

Trapezoidal_Map::Trapezoidal_Map(std::vector<Segment> trajectory_segments)
{
    AABB bounding_box(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

    //Determine the axis-alligned bounding box of the whole trajectory
    for (const Segment& segment : trajectory_segments)
    {
        bounding_box.combine(segment.get_AABB());
    }

    static constexpr Vec2 margin(1.f, 1.f);

    bounding_box.max += margin;
    bounding_box.max -= margin;

    //Compute random permutation
    std::vector<size_t> random_permutation(trajectory_segments.size());

    std::iota(random_permutation.begin(), random_permutation.end(), 0);
    std::shuffle(random_permutation.begin(), random_permutation.end(), std::mt19937{ std::random_device{}() });


    root = std::make_unique<Trapezoidal_Leaf_Node>();
}

Trapezoidal_Leaf_Node::Trapezoidal_Leaf_Node() : Trapezoidal_Node()
{

}

Trapezoidal_Leaf_Node::Trapezoidal_Leaf_Node(Segment& left_border, Segment& right_border, Vec2& top_point, Vec2& bottom_point) : Trapezoidal_Node()
{

}

Trapezoidal_Cell::Trapezoidal_Cell() :
    top_left(nullptr),
    top_right(nullptr),
    bottom_left(nullptr),
    bottom_right(nullptr),
    left_segment(nullptr),
    right_segment(nullptr),
    top_point(nullptr),
    bottom_point(nullptr)
{
}
