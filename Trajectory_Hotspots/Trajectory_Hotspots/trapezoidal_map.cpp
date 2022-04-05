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

    Segment left_segment(bounding_box.min, Vec2(bounding_box.min.x, bounding_box.max.y));
    Segment right_segment(Vec2(bounding_box.max.x, bounding_box.min.x), bounding_box.max);

    root = std::make_unique<Trapezoidal_Leaf_Node>(left_segment, right_segment, bounding_box.min, bounding_box.max);

    //Compute random permutation and add segments in this order
    std::vector<size_t> random_permutation(trajectory_segments.size());

    std::iota(random_permutation.begin(), random_permutation.end(), 0);
    std::shuffle(random_permutation.begin(), random_permutation.end(), std::mt19937{ std::random_device{}() });

    for (int i : random_permutation)
    {
        add_segment(trajectory_segments.at(i));
    }
}

void Trapezoidal_Map::add_segment(const Segment& segment)
{
    //Order endpoints bottom to top
    Segment query_segment;
    if (segment.start.y > segment.end.y)
    {
        query_segment.start = segment.end;
        query_segment.end = segment.start;
    }
    else
    {
        query_segment.start = segment.start;
        query_segment.end = segment.end;
    }

    //Find all the trapezoids that contain a part of this segment
    std::vector<const Trapezoidal_Leaf_Node*> intersecting_trapezoids = follow_segment(query_segment);


}

std::vector<const Trapezoidal_Leaf_Node*> Trapezoidal_Map::follow_segment(const Segment& query_segment) const
{
    //TODO: Test if this goes correct with edge case for point on segment?
    assert(query_segment.start.y < query_segment.end.y);

    std::vector<const Trapezoidal_Leaf_Node*> intersecting_trapezoids;

    //Find the trapezoid the starting point is inside of
    const Trapezoidal_Leaf_Node* starting_trapezoid = root->query_start_point(query_segment);
    intersecting_trapezoids.push_back(starting_trapezoid);

    //Follow along the segment to find all intersecting trapezoids
    while (query_segment.end.y > intersecting_trapezoids.back()->top_point->y)
    {
        if (point_right_of_segment(query_segment, *intersecting_trapezoids.back()->top_point))
        {
            //Top point is right of segment, add top left neighbour
            intersecting_trapezoids.push_back(intersecting_trapezoids.back()->top_left);
        }
        else
        {
            //Top point is left of segment, add top right neighbour
            intersecting_trapezoids.push_back(intersecting_trapezoids.back()->top_right);
        }
    }

    return intersecting_trapezoids;
}

Trapezoidal_Leaf_Node::Trapezoidal_Leaf_Node() : Trapezoidal_Node(),
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

Trapezoidal_Leaf_Node::Trapezoidal_Leaf_Node(Segment& left_border, Segment& right_border, Vec2& bottom_point, Vec2& top_point) : Trapezoidal_Node(),
top_left(nullptr),
top_right(nullptr),
bottom_left(nullptr),
bottom_right(nullptr),
left_segment(&left_border),
right_segment(&right_border),
bottom_point(&bottom_point),
top_point(&top_point)
{

}

const Trapezoidal_Leaf_Node* Trapezoidal_Leaf_Node::query_start_point(const Segment& query_segment) const
{
    //The query ended in this leaf node, return trapezoid
    return this;
}

const Trapezoidal_Leaf_Node* Trapezoidal_X_Node::query_start_point(const Segment& query_segment) const
{
    //Test if the segment lies left or right of this segment

    //If the startpoint of the query segment is the same as the endpoint of this segment, the query segment lies to the right.
    //(This is always true because we order the start and endpoints from left to right and only use this for a graph, so all points have degree 2)
    //TODO: Test if this needs to be nearly_equal, we init with the same points...
    //TODO: Come back to this
    if (nearly_equal(query_segment.start.x, segment->end.x) && nearly_equal(query_segment.start.y, segment->end.y))
    {
        return right->query_start_point(query_segment);
    }
    else if (query_segment.start.x > segment->end.x)
    {
        return right->query_start_point(query_segment);
    }
    else if (nearly_equal(query_segment.end.x, segment->start.x) && nearly_equal(query_segment.end.y, segment->start.y))
    {
        return left->query_start_point(query_segment);
    }
    else
    {
        return left->query_start_point(query_segment);
    }
}

const Trapezoidal_Leaf_Node* Trapezoidal_Y_Node::query_start_point(const Segment& query_segment) const
{
    //Test if query point lies above or below the Y-nodes point
    if (query_segment.start.y >= point->y)
    {
        return above->query_start_point(query_segment);
    }
    else
    {
        return below->query_start_point(query_segment);
    }
}
