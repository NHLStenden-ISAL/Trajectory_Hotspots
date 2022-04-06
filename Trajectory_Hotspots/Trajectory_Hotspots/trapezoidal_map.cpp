#include "pch.h"
#include "trapezoidal_map.h"

Trapezoidal_Map::Trapezoidal_Map()
{
    //TODO: Is there a better choice for this? Or do we want to always use infinity points for the bounding box?
    AABB bounding_box(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());

    left_border = Segment(bounding_box.min, Vec2(bounding_box.min.x, bounding_box.max.y));
    right_border = Segment(Vec2(bounding_box.max.x, bounding_box.max.y), bounding_box.max);

    top_point = bounding_box.max;
    bottom_point = bounding_box.min;

    std::unique_ptr<Trapezoidal_Node> root = std::make_unique<Trapezoidal_Leaf_Node>(&left_border, &right_border, &top_point, &bottom_point);
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

    left_border = Segment(bounding_box.min, Vec2(bounding_box.min.x, bounding_box.max.y));
    right_border = Segment(Vec2(bounding_box.max.x, bounding_box.max.y), bounding_box.max);

    top_point = bounding_box.max;
    bottom_point = bounding_box.min;

    root = std::make_unique<Trapezoidal_Leaf_Node>(&left_border, &right_border, &bottom_point, &top_point);

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
    //TODO: Move to member function of segment?
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
        //Segment is fully contained in a single trapezoid
        Trapezoidal_Leaf_Node* current_trapezoid = intersecting_trapezoids.at(0);
        //TODO: Split in function

    }
    else
    {
        //TODO: Replace trapezoids along the edge, check if the segment on one side is the same. If true, merge into one trapezoid on that side.
        //TODO: Add trapezoids, check if endpoints are equal to existing endpoint, dont add left and/or right trapezoids if true
        //TODO: Vector for multiple bottom/top points, almost same code

    }


}

void Trapezoidal_Map::add_fully_embedded_segment(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment)
{
    //Order endpoints bottom to top
    //TODO: Move to member function of segment?
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

    left_trapezoid->bottom_left = bottom_trapezoid.get();
    right_trapezoid->bottom_right = bottom_trapezoid.get();

    std::shared_ptr<Trapezoidal_Leaf_Node> top_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        current_trapezoid->left_segment,    //Left border
        current_trapezoid->right_segment,   //Right border
        queried_top_point,                  //Bottom point
        current_trapezoid->top_point,       //Top point
        left_trapezoid.get(),               //BL neighbour
        right_trapezoid.get(),              //BR neighbour
        current_trapezoid->top_left,        //TL neighbour
        current_trapezoid->top_right);      //TR neighbour

    left_trapezoid->top_left = top_trapezoid.get();
    right_trapezoid->top_right = top_trapezoid.get();

    //Redirect pointers from bottom neighbour to new trapezoid
    current_trapezoid->bottom_left->replace_top_neighbour(current_trapezoid, bottom_trapezoid.get());
    current_trapezoid->bottom_right->replace_top_neighbour(current_trapezoid, bottom_trapezoid.get());

    //Redirect pointers from top neighbour to new trapezoid
    current_trapezoid->top_left->replace_bottom_neighbour(current_trapezoid, top_trapezoid.get());
    current_trapezoid->top_right->replace_bottom_neighbour(current_trapezoid, top_trapezoid.get());

    //TODO: Move to constructors?
    std::shared_ptr<Trapezoidal_X_Node> x_node = std::make_shared<Trapezoidal_X_Node>();
    x_node->left = left_trapezoid;
    x_node->right = right_trapezoid;
    x_node->segment = &segment;

    std::shared_ptr<Trapezoidal_Y_Node> top_y_node = std::make_shared<Trapezoidal_Y_Node>();
    top_y_node->point = queried_bottom_point;
    top_y_node->below = std::move(x_node);
    top_y_node->above = top_trapezoid;

    std::shared_ptr<Trapezoidal_Y_Node> bottom_y_node = std::make_shared<Trapezoidal_Y_Node>();
    bottom_y_node->point = queried_bottom_point;
    bottom_y_node->below = bottom_trapezoid;
    bottom_y_node->above = std::move(top_y_node);

    //Replace leaf node in the graph with the new subgraph
    for (Trapezoidal_Internal_Node* parent_node : current_trapezoid->parents)
    {
        parent_node->replace_child(current_trapezoid, bottom_y_node);
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


Trapezoidal_Leaf_Node::Trapezoidal_Leaf_Node(const Segment* left_border, const Segment* right_border, const Vec2* bottom_point, const Vec2* top_point) : Trapezoidal_Node(),

left_segment(left_border),
right_segment(right_border),
bottom_point(bottom_point),
top_point(top_point),
top_left(nullptr),
top_right(nullptr),
bottom_left(nullptr),
bottom_right(nullptr)
{

}

Trapezoidal_Leaf_Node::Trapezoidal_Leaf_Node(
    const Segment* left_border, const Segment* right_border,
    const Vec2* bottom_point, const Vec2* top_point,
    Trapezoidal_Leaf_Node* bottom_left, Trapezoidal_Leaf_Node* bottom_right,
    Trapezoidal_Leaf_Node* top_left, Trapezoidal_Leaf_Node* top_right) : Trapezoidal_Node(),
    left_segment(left_border),
    right_segment(right_border),
    bottom_point(bottom_point),
    top_point(top_point),
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

void Trapezoidal_Leaf_Node::replace_bottom_neighbour(Trapezoidal_Leaf_Node* old_bottom_neighbour, Trapezoidal_Leaf_Node* new_bottom_neighbour)
{
    if (bottom_left == old_bottom_neighbour)
    {
        bottom_left = new_bottom_neighbour;
    }
    else if (bottom_right == old_bottom_neighbour)
    {
        bottom_right = new_bottom_neighbour;
    }

    assert(false);
    return;
}

void Trapezoidal_Leaf_Node::replace_top_neighbour(Trapezoidal_Leaf_Node* old_top_neighbour, Trapezoidal_Leaf_Node* new_top_neighbour)
{
    if (top_left == old_top_neighbour)
    {
        top_left = new_top_neighbour;
    }
    else if (top_right == old_top_neighbour)
    {
        top_right = new_top_neighbour;
    }

    assert(false);
    return;
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

void Trapezoidal_X_Node::replace_child(Trapezoidal_Node* old_child, std::shared_ptr<Trapezoidal_Node> new_child)
{
    if (left.get() == old_child)
    {
        left = new_child;
        return;
    }
    else if (right.get() == old_child)
    {
        right = new_child;
        return;
    }

    assert(false);
    return;
}

void Trapezoidal_Y_Node::replace_child(Trapezoidal_Node* old_child, std::shared_ptr<Trapezoidal_Node> new_child)
{
    if (below.get() == old_child)
    {
        below = new_child;
        return;
    }
    else if (above.get() == old_child)
    {
        above = new_child;
        return;
    }

    assert(false);
    return;
}
