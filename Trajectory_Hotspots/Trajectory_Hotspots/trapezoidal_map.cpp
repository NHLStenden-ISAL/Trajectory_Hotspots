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

    for (size_t i : random_permutation)
    {
        add_segment(trajectory_segments.at(i));
    }
}

void Trapezoidal_Map::add_segment(const Segment& segment)
{
    //Order endpoints bottom to top
    const Vec2* queried_bottom_point;
    const Vec2* queried_top_point;
    if (segment.start.y > segment.end.y)
    {
        queried_bottom_point = &segment.end;
        queried_top_point = &segment.start;
    }
    else
    {
        queried_bottom_point = &segment.start;
        queried_top_point = &segment.end;
    }

    //Find all the trapezoids that contain a part of this segment
    std::vector<Trapezoidal_Leaf_Node*> intersecting_trapezoids = follow_segment(Segment(*queried_bottom_point, *queried_top_point));

    if (intersecting_trapezoids.size() == 1)
    {
        Trapezoidal_Leaf_Node* current_trapezoid = intersecting_trapezoids.at(0);
        //Segment is fully contained in a single trapezoid
        //TODO: Split in function

        //TODO: Add trapezoids, check if endpoints are equal to existing endpoint, dont add left and/or right trapezoids if true
        //TODO: Vector for multiple bottom/top points, almost same code
        std::unique_ptr<Trapezoidal_Y_Node> new_subtree;
        new_subtree->point = queried_bottom_point;

        std::shared_ptr<Trapezoidal_Leaf_Node> left_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            current_trapezoid->left_segment,    //Left border
            &segment,                           //Right border
            queried_bottom_point,               //Bottom point
            queried_top_point);                 //Top point

        std::shared_ptr<Trapezoidal_Leaf_Node> right_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            &segment,                           //Left border
            current_trapezoid->right_segment,   //Right border
            queried_bottom_point,               //Bottom point
            queried_top_point);                 //Top point

        std::shared_ptr<Trapezoidal_Leaf_Node> bottom_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            current_trapezoid->left_segment,    //Left border
            current_trapezoid->right_segment,   //Right border
            current_trapezoid->bottom_point,    //Bottom point
            queried_bottom_point,               //Top point
            current_trapezoid->bottom_left,     //BL neighbour
            current_trapezoid->bottom_right,    //BR neighbour
            left_trapezoid.get(),               //TL neighbour
            right_trapezoid.get());             //TR neighbour

        std::shared_ptr<Trapezoidal_Leaf_Node> top_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            current_trapezoid->left_segment,    //Left border
            current_trapezoid->right_segment,   //Right border
            queried_top_point,                  //Bottom point
            current_trapezoid->top_point,       //Top point
            left_trapezoid.get(),               //BL neighbour
            right_trapezoid.get(),              //BR neighbour
            current_trapezoid->top_left,        //TL neighbour
            current_trapezoid->top_right);      //TR neighbour

        left_trapezoid->set_neighbour_pointers(

        );

        right_trapezoid->set_neighbour_pointers();
        //TODO: For the left trapezoid top and bottom are left and for the right trapezoid the opposite.
        //TODO: Move to constructors


    }
    else
    {
        //TODO: Replace trapezoids along the edge, check if the segment on one side is the same. If true, merge into one trapezoid on that side.
    }


}


//Same end points:
//{
//    if (*queried_bottom_point != *intersecting_trapezoids.at(0)->bottom_point)
//    {
//        std::shared_ptr<Trapezoidal_Leaf_Node> bottom_trapezoid;
//        bottom_trapezoid->left_segment = current_trapezoid->left_segment;
//        bottom_trapezoid->right_segment = current_trapezoid->right_segment;
//        bottom_trapezoid->bottom_point = current_trapezoid->bottom_point;
//        bottom_trapezoid->top_point = queried_bottom_point;
//
//        bottom_trapezoid->top_left = left_trapezoid.get();
//        bottom_trapezoid->top_right = right_trapezoid.get();
//        bottom_trapezoid->bottom_left = current_trapezoid->bottom_left;
//        bottom_trapezoid->bottom_right = current_trapezoid->bottom_right;
//    }
//
//    if (*queried_top_point != *intersecting_trapezoids.at(0)->top_point)
//    {
//        std::shared_ptr<Trapezoidal_Leaf_Node> top_trapezoid;
//        top_trapezoid->left_segment = current_trapezoid->left_segment;
//        top_trapezoid->right_segment = current_trapezoid->right_segment;
//        top_trapezoid->bottom_point = queried_top_point;
//        top_trapezoid->top_point = current_trapezoid->top_point;
//
//        top_trapezoid->top_left = current_trapezoid->top_left;
//        top_trapezoid->top_right = current_trapezoid->top_right;
//        top_trapezoid->bottom_left = left_trapezoid.get();
//        top_trapezoid->bottom_right = right_trapezoid.get();
//    }
//}

std::vector<Trapezoidal_Leaf_Node*> Trapezoidal_Map::follow_segment(const Segment& query_segment)
{
    //TODO: Test if this goes correct with edge case for point on segment?
    assert(query_segment.start.y < query_segment.end.y);

    std::vector<Trapezoidal_Leaf_Node*> intersecting_trapezoids;

    //Find the trapezoid the starting point is inside of
    Trapezoidal_Leaf_Node* starting_trapezoid = root->query_start_point(query_segment);
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
left_segment(nullptr),
right_segment(nullptr),
top_point(nullptr),
bottom_point(nullptr),
top_left(nullptr),
top_right(nullptr),
bottom_left(nullptr),
bottom_right(nullptr)
{

}

Trapezoidal_Leaf_Node::Trapezoidal_Leaf_Node(Segment& left_border, Segment& right_border, Vec2& bottom_point, Vec2& top_point) : Trapezoidal_Node(),
left_segment(&left_border),
right_segment(&right_border),
bottom_point(&bottom_point),
top_point(&top_point),
top_left(nullptr),
top_right(nullptr),
bottom_left(nullptr),
bottom_right(nullptr)
{

}

Trapezoidal_Leaf_Node::Trapezoidal_Leaf_Node(
    Segment& left_border, Segment& right_border,
    Vec2& bottom_point, Vec2& top_point,
    Trapezoidal_Leaf_Node* bottom_left, Trapezoidal_Leaf_Node* bottom_right,
    Trapezoidal_Leaf_Node* top_left, Trapezoidal_Leaf_Node* top_right) : Trapezoidal_Node(),
    left_segment(&left_border),
    right_segment(&right_border),
    bottom_point(&bottom_point),
    top_point(&top_point),
    bottom_left(bottom_left),
    bottom_right(bottom_right),
    top_left(top_left),
    top_right(top_right)
{
}

void Trapezoidal_Leaf_Node::set_neighbour_pointers(Trapezoidal_Leaf_Node* bottom_left, Trapezoidal_Leaf_Node* bottom_right, Trapezoidal_Leaf_Node* top_left, Trapezoidal_Leaf_Node* top_right)
{
    this->bottom_left = bottom_left;
    this->bottom_right = bottom_right;
    this->top_left = top_left;
    this->top_right = top_right;
}

Trapezoidal_Leaf_Node* Trapezoidal_Leaf_Node::query_start_point(const Segment& query_segment)
{
    //The query ended in this leaf node, return trapezoid
    return this;
}

Trapezoidal_Leaf_Node* Trapezoidal_X_Node::query_start_point(const Segment& query_segment)
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

Trapezoidal_Leaf_Node* Trapezoidal_Y_Node::query_start_point(const Segment& query_segment)
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
