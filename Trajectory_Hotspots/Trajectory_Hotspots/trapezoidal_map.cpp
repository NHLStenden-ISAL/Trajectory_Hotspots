#include "pch.h"
#include "trapezoidal_map.h"

Trapezoidal_Map::Trapezoidal_Map() : segment_count(0)
{
    //TODO: Is there a better choice for this? Or do we want to always use infinity points for the bounding box?
    AABB bounding_box(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());

    left_border = Segment(bounding_box.min, Vec2(bounding_box.min.x, bounding_box.max.y));
    right_border = Segment(Vec2(bounding_box.max.x, bounding_box.max.y), bounding_box.max);

    bottom_point = bounding_box.min;
    top_point = bounding_box.max;

    root = std::make_unique<Trapezoidal_Leaf_Node>(&left_border, &right_border, &bottom_point, &top_point);
}

Trapezoidal_Map::Trapezoidal_Map(std::vector<Segment> trajectory_segments) : segment_count(0)
{
    AABB bounding_box(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());

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

        //Check if any endpoints overlap the top and bottom points of the trapezoid
        int overlap = 0;
        if (*current_trapezoid->bottom_point == *queried_bottom_point) overlap |= 1;
        if (*current_trapezoid->top_point == *queried_top_point) overlap |= 2;

        switch (overlap)
        {
        case 0: //None
            add_fully_embedded_segment(current_trapezoid, segment);
            break;
        case 1: //Bottom
            add_fully_embedded_segment_with_bottom_endpoint_overlapping(current_trapezoid, segment);
            break;
        case 2: //Top
            add_fully_embedded_segment_with_top_endpoint_overlapping(current_trapezoid, segment);
            break;
        case 3: //Both
            add_fully_embedded_segment_with_both_endpoints_overlapping(current_trapezoid, segment);
            break;
        }
    }
    else
    {
        //TODO: Replace trapezoids along the edge, check if the segment on one side is the same. If true, merge into one trapezoid on that side.
        //TODO: Add trapezoids, check if endpoints are equal to existing endpoint, dont add left and/or right trapezoids if true
        //TODO: Vector for multiple bottom/top points, almost same code

        add_overlapping_segment(intersecting_trapezoids, segment);
    }

    segment_count++;
}

void Trapezoidal_Map::add_fully_embedded_segment(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment)
{
    //Order endpoints bottom to top
    //TODO: Move to member function of segment?
    const Vec2* queried_bottom_point = segment.get_bottom_point();
    const Vec2* queried_top_point = segment.get_top_point();

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

    //Redirect pointers from bottom neighbours to new trapezoid
    if (current_trapezoid->bottom_left != nullptr)
    {
        current_trapezoid->bottom_left->replace_top_neighbour(current_trapezoid, bottom_trapezoid.get());
    }
    if (current_trapezoid->bottom_right != nullptr)
    {
        current_trapezoid->bottom_right->replace_top_neighbour(current_trapezoid, bottom_trapezoid.get());
    }

    //Redirect pointers from top neighbours to new trapezoid
    if (current_trapezoid->top_left != nullptr)
    {
        current_trapezoid->top_left->replace_bottom_neighbour(current_trapezoid, top_trapezoid.get());
    }
    if (current_trapezoid->top_right != nullptr)
    {
        current_trapezoid->top_right->replace_bottom_neighbour(current_trapezoid, top_trapezoid.get());
    }

    //TODO: Move to constructors?
    std::shared_ptr<Trapezoidal_X_Node> x_node = std::make_shared<Trapezoidal_X_Node>();

    left_trapezoid->parents.push_back(x_node.get());
    right_trapezoid->parents.push_back(x_node.get());

    x_node->left = std::move(left_trapezoid);
    x_node->right = std::move(right_trapezoid);
    x_node->segment = &segment;



    std::shared_ptr<Trapezoidal_Y_Node> top_y_node = std::make_shared<Trapezoidal_Y_Node>();

    x_node->parents.push_back(top_y_node.get());
    top_trapezoid->parents.push_back(top_y_node.get());

    top_y_node->point = queried_top_point;
    top_y_node->below = std::move(x_node);
    top_y_node->above = std::move(top_trapezoid);



    std::shared_ptr<Trapezoidal_Y_Node> bottom_y_node = std::make_shared<Trapezoidal_Y_Node>();

    bottom_trapezoid->parents.push_back(bottom_y_node.get());
    top_y_node->parents.push_back(bottom_y_node.get());

    bottom_y_node->point = queried_bottom_point;
    bottom_y_node->below = std::move(bottom_trapezoid);
    bottom_y_node->above = std::move(top_y_node);

    //Replace leaf node in the graph with the new subgraph, this should reduce the shared_ptr count to 0
    replace_leaf_node_with_subgraph(current_trapezoid, bottom_y_node);
}

void Trapezoidal_Map::add_fully_embedded_segment_with_both_endpoints_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment)
{
    std::shared_ptr<Trapezoidal_Leaf_Node> left_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        current_trapezoid->left_segment,    //Left border
        &segment,                           //Right border
        current_trapezoid->bottom_point,    //Bottom point
        current_trapezoid->top_point);      //Top point

    std::shared_ptr<Trapezoidal_Leaf_Node> right_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        &segment,                           //Left border
        current_trapezoid->right_segment,   //Right border
        current_trapezoid->bottom_point,    //Bottom point
        current_trapezoid->top_point);      //Top point

    //Check the orientation of the segment and determine if the trapezoids have a top neighbour
    if (*segment.get_top_point() == *current_trapezoid->right_segment->get_top_point())
    {
        left_trapezoid->top_left = current_trapezoid->top_left;
        current_trapezoid->top_left->replace_bottom_neighbour(current_trapezoid, left_trapezoid.get());
    }
    else if (*segment.get_top_point() == *current_trapezoid->left_segment->get_top_point())
    {
        right_trapezoid->top_right = current_trapezoid->top_right;
        current_trapezoid->top_right->replace_bottom_neighbour(current_trapezoid, right_trapezoid.get());
    }
    else
    {
        //Top point does not overlap one of the sides so both trapezoids have a top neighbour
        left_trapezoid->top_left = current_trapezoid->top_left;
        right_trapezoid->top_right = current_trapezoid->top_right;

        //Redirect pointers from top neighbour to new trapezoid
        current_trapezoid->top_left->replace_bottom_neighbour(current_trapezoid, left_trapezoid.get());
        current_trapezoid->top_right->replace_bottom_neighbour(current_trapezoid, right_trapezoid.get());
    }

    //Check the orientation of the segment and determine if the trapezoids have a bottom neighbour
    if (*segment.get_bottom_point() == *current_trapezoid->right_segment->get_bottom_point())
    {
        left_trapezoid->bottom_left = current_trapezoid->bottom_left;
        current_trapezoid->bottom_left->replace_top_neighbour(current_trapezoid, left_trapezoid.get());

    }
    else if (*segment.get_bottom_point() == *current_trapezoid->left_segment->get_bottom_point())
    {
        right_trapezoid->bottom_right = current_trapezoid->bottom_right;
        current_trapezoid->bottom_right->replace_top_neighbour(current_trapezoid, right_trapezoid.get());
    }
    else
    {
        //bottom point does not overlap one of the sides so both trapezoids have a bottom neighbour
        left_trapezoid->bottom_left = current_trapezoid->bottom_left;
        right_trapezoid->bottom_right = current_trapezoid->bottom_right;

        //Redirect pointers from bottom neighbour to new trapezoid
        current_trapezoid->bottom_left->replace_top_neighbour(current_trapezoid, left_trapezoid.get());
        current_trapezoid->bottom_right->replace_top_neighbour(current_trapezoid, right_trapezoid.get());
    }

    //Segment node with left and right leafs
    std::shared_ptr<Trapezoidal_X_Node> x_node = std::make_shared<Trapezoidal_X_Node>();

    left_trapezoid->parents.push_back(x_node.get());
    right_trapezoid->parents.push_back(x_node.get());

    x_node->left = std::move(left_trapezoid);
    x_node->right = std::move(right_trapezoid);
    x_node->segment = &segment;

    //Replace leaf node in the graph with the new subgraph, this should reduce the shared_ptr count to 0
    replace_leaf_node_with_subgraph(current_trapezoid, x_node);
}

void Trapezoidal_Map::add_fully_embedded_segment_with_top_endpoint_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment)
{
    std::shared_ptr<Trapezoidal_Leaf_Node> left_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        current_trapezoid->left_segment,    //Left border
        &segment,                           //Right border
        segment.get_bottom_point(),         //Bottom point
        current_trapezoid->top_point);      //Top point

    std::shared_ptr<Trapezoidal_Leaf_Node> right_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        &segment,                           //Left border
        current_trapezoid->right_segment,   //Right border
        segment.get_bottom_point(),         //Bottom point
        current_trapezoid->top_point);      //Top point

    std::shared_ptr<Trapezoidal_Leaf_Node> bottom_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        current_trapezoid->left_segment,    //Left border
        current_trapezoid->right_segment,   //Right border
        current_trapezoid->bottom_point,    //Bottom point
        segment.get_bottom_point(),         //Top point
        current_trapezoid->bottom_left,     //Bottom left
        current_trapezoid->bottom_right,    //Bottom right
        left_trapezoid.get(),               //Top left
        right_trapezoid.get());             //Top right

    //TODO: Recheck bottom and top for all cases
    left_trapezoid->bottom_left = bottom_trapezoid.get();
    right_trapezoid->bottom_right = bottom_trapezoid.get();

    //Check the orientation of the segment and determine if the trapezoids have a top neighbour
    //Note: Both can't be true for our use case because at most two points can overlap
    if (*segment.get_top_point() == *current_trapezoid->right_segment->get_top_point())
    {
        left_trapezoid->top_left = current_trapezoid->top_left;
        current_trapezoid->top_left->replace_bottom_neighbour(current_trapezoid, left_trapezoid.get());
    }
    else if (*segment.get_top_point() == *current_trapezoid->left_segment->get_top_point())
    {
        right_trapezoid->top_right = current_trapezoid->top_right;
        current_trapezoid->top_right->replace_bottom_neighbour(current_trapezoid, right_trapezoid.get());
    }
    else
    {
        //Top point does not overlap one of the sides so both trapezoids have a top neighbour
        left_trapezoid->top_left = current_trapezoid->top_left;
        right_trapezoid->top_right = current_trapezoid->top_right;

        //Redirect pointers from top neighbour to new trapezoid
        current_trapezoid->top_left->replace_bottom_neighbour(current_trapezoid, left_trapezoid.get());
        current_trapezoid->top_right->replace_bottom_neighbour(current_trapezoid, right_trapezoid.get());
    }

    //Segment node with left and right leafs
    std::shared_ptr<Trapezoidal_X_Node> x_node = std::make_shared<Trapezoidal_X_Node>();

    left_trapezoid->parents.push_back(x_node.get());
    right_trapezoid->parents.push_back(x_node.get());

    x_node->left = std::move(left_trapezoid);
    x_node->right = std::move(right_trapezoid);
    x_node->segment = &segment;

    std::shared_ptr<Trapezoidal_Y_Node> bottom_y_node = std::make_shared<Trapezoidal_Y_Node>();

    x_node->parents.push_back(bottom_y_node.get());
    bottom_trapezoid->parents.push_back(bottom_y_node.get());

    bottom_y_node->point = segment.get_bottom_point();
    bottom_y_node->below = std::move(bottom_trapezoid);
    bottom_y_node->above = std::move(x_node);

    //Replace leaf node in the graph with the new subgraph, this should reduce the shared_ptr count to 0
    replace_leaf_node_with_subgraph(current_trapezoid, bottom_y_node);
}

void Trapezoidal_Map::add_fully_embedded_segment_with_bottom_endpoint_overlapping(Trapezoidal_Leaf_Node* current_trapezoid, const Segment& segment)
{
    std::shared_ptr<Trapezoidal_Leaf_Node> left_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        current_trapezoid->left_segment,    //Left border
        &segment,                           //Right border
        current_trapezoid->bottom_point,    //Bottom point
        segment.get_top_point());           //Top point

    std::shared_ptr<Trapezoidal_Leaf_Node> right_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        &segment,                           //Left border
        current_trapezoid->right_segment,   //Right border
        current_trapezoid->bottom_point,    //Bottom point
        segment.get_top_point());           //Top point

    std::shared_ptr<Trapezoidal_Leaf_Node> top_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
        current_trapezoid->left_segment,    //Left border
        current_trapezoid->right_segment,   //Right border
        segment.get_top_point(),            //Bottom point
        current_trapezoid->top_point);      //Top point

    //Check the orientation of the segment and determine if the trapezoids have a bottom neighbour
    //Note: Both can't be true for our use case because at most two points can overlap
    if (*segment.get_bottom_point() == *current_trapezoid->right_segment->get_bottom_point())
    {
        left_trapezoid->bottom_left = current_trapezoid->bottom_left;
        current_trapezoid->bottom_left->replace_top_neighbour(current_trapezoid, left_trapezoid.get());
    }
    else if (*segment.get_bottom_point() == *current_trapezoid->left_segment->get_bottom_point())
    {
        right_trapezoid->bottom_right = current_trapezoid->bottom_right;
        current_trapezoid->bottom_right->replace_top_neighbour(current_trapezoid, right_trapezoid.get());
    }
    else
    {
        //Bottom point does not overlap one of the sides so both trapezoids have a bottom neighbour
        left_trapezoid->bottom_left = current_trapezoid->bottom_left;
        right_trapezoid->bottom_right = current_trapezoid->bottom_right;

        //Redirect pointers from bottom neighbour to new trapezoid
        current_trapezoid->bottom_left->replace_top_neighbour(current_trapezoid, left_trapezoid.get());
        current_trapezoid->bottom_right->replace_top_neighbour(current_trapezoid, right_trapezoid.get());
    }

    //Segment node with left and right leafs
    std::shared_ptr<Trapezoidal_X_Node> x_node = std::make_shared<Trapezoidal_X_Node>();

    left_trapezoid->parents.push_back(x_node.get());
    right_trapezoid->parents.push_back(x_node.get());

    x_node->left = std::move(left_trapezoid);
    x_node->right = std::move(right_trapezoid);
    x_node->segment = &segment;

    std::shared_ptr<Trapezoidal_Y_Node> top_y_node = std::make_shared<Trapezoidal_Y_Node>();

    x_node->parents.push_back(top_y_node.get());
    top_trapezoid->parents.push_back(top_y_node.get());

    top_y_node->point = segment.get_top_point();
    top_y_node->below = std::move(x_node);
    top_y_node->above = std::move(top_trapezoid);

    //Replace leaf node in the graph with the new subgraph, this should reduce the shared_ptr count of the old node to 0
    replace_leaf_node_with_subgraph(current_trapezoid, top_y_node);
}

void Trapezoidal_Map::add_overlapping_segment(std::vector<Trapezoidal_Leaf_Node*> overlapping_trapezoids, const Segment& segment)
{
    //TODO: This pointer can probably be const, same for function argument..
    std::vector<Trapezoidal_Leaf_Node*>::iterator current = overlapping_trapezoids.begin();
    std::vector<Trapezoidal_Leaf_Node*>::iterator end = overlapping_trapezoids.end();

    //TODO: Don't forget the check for 0 middle trapezoids
    //TODO: Set pointers from neighbours
    //TODO: Refactor bottom trapezoid same as top trapezoids..

    std::shared_ptr<Trapezoidal_Leaf_Node> left_trapezoid;
    std::shared_ptr<Trapezoidal_Leaf_Node> right_trapezoid;

    //Handle the bottom trapezoid, split it in two or three new trapezoids
    Trapezoidal_Leaf_Node* old_bottom_trapezoid = *current;
    if (*old_bottom_trapezoid->bottom_point != *segment.get_bottom_point())
    {
        std::shared_ptr<Trapezoidal_Leaf_Node> bottom_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            old_bottom_trapezoid->left_segment,  //Left border
            old_bottom_trapezoid->right_segment, //Right border
            old_bottom_trapezoid->bottom_point,  //Bottom point
            segment.get_bottom_point());         //Top point

        bottom_trapezoid->bottom_left = old_bottom_trapezoid->bottom_left;
        bottom_trapezoid->bottom_right = old_bottom_trapezoid->bottom_right;

        left_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            old_bottom_trapezoid->left_segment,  //Left border
            &segment,                            //Right border
            segment.get_bottom_point(),          //Bottom point
            nullptr);                            //Top point

        right_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            &segment,                            //Left border
            old_bottom_trapezoid->right_segment, //Right border
            segment.get_bottom_point(),          //Bottom point
            nullptr);                            //Top point

        left_trapezoid->bottom_left = bottom_trapezoid.get();
        right_trapezoid->bottom_right = bottom_trapezoid.get();

        bottom_trapezoid->top_left = left_trapezoid.get();
        bottom_trapezoid->top_right = right_trapezoid.get();

        //Construct subgraph with the y_node of the bottom point as the root node
        std::shared_ptr<Trapezoidal_X_Node> x_node = std::make_shared<Trapezoidal_X_Node>();
        x_node->left = left_trapezoid;
        x_node->right = right_trapezoid;
        x_node->segment = &segment;

        left_trapezoid->parents.push_back(x_node.get());
        right_trapezoid->parents.push_back(x_node.get());

        std::shared_ptr<Trapezoidal_Y_Node> bottom_y_node = std::make_shared<Trapezoidal_Y_Node>();

        x_node->parents.push_back(bottom_y_node.get());
        bottom_trapezoid->parents.push_back(bottom_y_node.get());

        bottom_y_node->point = segment.get_bottom_point();
        bottom_y_node->below = std::move(bottom_trapezoid);
        bottom_y_node->above = std::move(x_node);

        replace_leaf_node_with_subgraph(old_bottom_trapezoid, bottom_y_node);
    }
    else
    {
        //Bottom points overlap, just split in two
        left_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            old_bottom_trapezoid->left_segment,  //Left border
            &segment,                            //Right border
            old_bottom_trapezoid->bottom_point,  //Bottom point
            nullptr,                             //Top point
            old_bottom_trapezoid->bottom_left,   //Bottom left
            nullptr,                             //Bottom right
            nullptr,                             //Top left
            nullptr);                            //Top right

        right_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            &segment,                            //Left border
            old_bottom_trapezoid->right_segment, //Right border
            old_bottom_trapezoid->bottom_point,  //Bottom point
            nullptr,                             //Top point
            nullptr,                             //Bottom left
            old_bottom_trapezoid->bottom_right,  //Bottom right
            nullptr,                             //Top left
            nullptr);                            //Top right

        //Construct subgraph with the x_node of the segment as root node
        std::shared_ptr<Trapezoidal_X_Node> x_node = std::make_shared<Trapezoidal_X_Node>();
        x_node->left = left_trapezoid;
        x_node->right = right_trapezoid;
        x_node->segment = &segment;

        left_trapezoid->parents.push_back(x_node.get());
        right_trapezoid->parents.push_back(x_node.get());

        replace_leaf_node_with_subgraph(old_bottom_trapezoid, x_node);
    }

    std::shared_ptr<Trapezoidal_Leaf_Node> prev_left_trapezoid = std::move(left_trapezoid);
    std::shared_ptr<Trapezoidal_Leaf_Node> prev_right_trapezoid = std::move(right_trapezoid);

    //Save the top left and right of the previous trapezoid.
    //When the top point of a middle trapezoid is determined (the bottom of the new trapezoid)
    //set the top neighbour of the previously created trapezoid on the same side to the old neighbour
    //We do it this way because it is uncertain on which said the top point is without doing extra (unnecessary) checks
    //TODO: DOUBLE CHECK THIS
    Trapezoidal_Leaf_Node* prev_top_left = nullptr;
    Trapezoidal_Leaf_Node* prev_top_right = nullptr;

    //Handle middle trapezoids
    while (++current != std::prev(end))
    {
        std::shared_ptr<Trapezoidal_X_Node> x_node = std::make_shared<Trapezoidal_X_Node>();
        x_node->segment = &segment;

        Trapezoidal_Leaf_Node* current_trapezoid = *current;
        if (point_right_of_segment(segment, *current_trapezoid->bottom_point))
        {
            //Create new right trapezoid, left extends
            right_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
                &segment,                            //Left border
                current_trapezoid->right_segment,    //Right border
                current_trapezoid->bottom_point,     //Bottom point
                nullptr,                             //Top point, will be filled when this trapezoid ends
                prev_right_trapezoid.get(),          //Bottom left
                current_trapezoid->bottom_right,     //Bottom right
                nullptr,                             //Top left, will be filled with the next right trapezoid
                current_trapezoid->top_right);       //Top right, if not nullptr it will also be the top right neighbour of this trapezoid

            //Finish the x_node and replace the leaf node with the new subgraph
            x_node->left = prev_left_trapezoid;
            x_node->right = right_trapezoid;

            prev_left_trapezoid->parents.push_back(x_node.get());
            right_trapezoid->parents.push_back(x_node.get());

            replace_leaf_node_with_subgraph(current_trapezoid, x_node);

            //Finish the previous right trapezoid by filling the last missing fields
            prev_right_trapezoid->top_point = current_trapezoid->bottom_point;
            prev_right_trapezoid->top_left = right_trapezoid.get();
            prev_right_trapezoid->top_right = prev_top_right;

            prev_right_trapezoid = std::move(right_trapezoid);
        }
        else
        {
            //Create new left trapezoid, right extends
            left_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
                current_trapezoid->left_segment,     //Left border
                &segment,                            //Right border
                current_trapezoid->bottom_point,     //Bottom point
                nullptr,                             //Top point, will be filled when this trapezoid ends
                current_trapezoid->bottom_left,      //Bottom left
                prev_left_trapezoid.get(),           //Bottom right
                current_trapezoid->top_left,         //Top left, if not nullptr it will also be the top left neighbour of this trapezoid
                nullptr);                            //Top right, will be filled with the next left trapezoid

            //Finish the x_node and replace the leaf node with the new subgraph
            x_node->left = left_trapezoid;
            x_node->right = prev_right_trapezoid;

            left_trapezoid->parents.push_back(x_node.get());
            prev_right_trapezoid->parents.push_back(x_node.get());

            replace_leaf_node_with_subgraph(current_trapezoid, x_node);

            //Finish the previous left trapezoid by filling the last missing fields
            prev_left_trapezoid->top_point = current_trapezoid->bottom_point;
            prev_left_trapezoid->top_left = prev_top_left;
            prev_left_trapezoid->top_right = left_trapezoid.get();

            prev_left_trapezoid = std::move(left_trapezoid);
        }

        prev_top_left = current_trapezoid->top_left;
        prev_top_right = current_trapezoid->top_right;
    }

    //Handle the top trapezoid:
    //If top points are the same, finish left and right, set top neighbours same as the old trapezoid
    //Else, create top trapezoid, set bottom neighbours to left and right and fix all the neighbours
    //For both cases, check if the bottom point is left or right, only create the trapezoid on that side, extend the other

    Trapezoidal_Leaf_Node* old_top_trapezoid = *(++current);

    std::shared_ptr<Trapezoidal_Leaf_Node> top_trapezoid;

    Trapezoidal_Leaf_Node* top_left = nullptr;
    Trapezoidal_Leaf_Node* top_right = nullptr;

    //Check if top points overlap, skip top_trapezoid if true
    if (*old_top_trapezoid->bottom_point != *segment.get_top_point())
    {
        top_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            old_top_trapezoid->left_segment,  //Left border
            old_top_trapezoid->right_segment, //Right border
            segment.get_top_point(),          //Bottom point
            old_top_trapezoid->top_point,     //Top point
            nullptr, nullptr,                 //Bottom left, bottom right,
            old_top_trapezoid->top_left,      //Top left 
            old_top_trapezoid->top_right);    //Top right

        top_left = top_trapezoid.get();
        top_right = top_trapezoid.get();
    }
    else
    {
        top_left = old_top_trapezoid->top_left;
        top_right = old_top_trapezoid->top_right;
    }

    if (point_right_of_segment(segment, *old_top_trapezoid->bottom_point))
    {
        //Create new right trapezoid
        right_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            &segment,                         //Left border
            old_top_trapezoid->right_segment, //Right border
            old_top_trapezoid->bottom_point,  //Bottom point
            segment.get_top_point(),          //Top point
            prev_right_trapezoid.get(),       //Bottom left
            nullptr,                          //Bottom right
            nullptr,                          //Top left
            top_right);             //Top right

        //Finish the previous right trapezoid by filling the last missing fields
        prev_right_trapezoid->top_point = old_top_trapezoid->bottom_point;
        prev_right_trapezoid->top_left = right_trapezoid.get();
        prev_right_trapezoid->top_right = prev_top_right;

        //Finish the left trapezoid by filling the last missing fields
        left_trapezoid->top_point = segment.get_top_point();
        left_trapezoid->top_left = top_left;
        left_trapezoid->top_right = nullptr;
    }
    else
    {
        //Create new left trapezoid, right extends
        left_trapezoid = std::make_shared<Trapezoidal_Leaf_Node>(
            old_top_trapezoid->left_segment, //Left border
            &segment,                        //Right border
            old_top_trapezoid->bottom_point, //Bottom point
            segment.get_top_point(),         //Top point
            nullptr,                         //Bottom left
            prev_left_trapezoid.get(),       //Bottom right
            top_left,                        //Top left 
            nullptr);                        //Top right

        //Finish the previous left trapezoid by filling the last missing fields
        prev_left_trapezoid->top_point = old_top_trapezoid->bottom_point;
        prev_left_trapezoid->top_left = prev_top_left;
        prev_left_trapezoid->top_right = left_trapezoid.get();

        //Finish the right trapezoid by filling the last missing fields
        right_trapezoid->top_point = segment.get_top_point();
        right_trapezoid->top_left = nullptr;
        right_trapezoid->top_right = top_right;
    }

    if (top_trapezoid != nullptr)
    {
        top_trapezoid->bottom_left = left_trapezoid.get();
        top_trapezoid->bottom_right = right_trapezoid.get();

        //Create the final subgraph for the top trapezoids with the y_node of the top point as the root node
        std::shared_ptr<Trapezoidal_X_Node> top_x_node = std::make_shared<Trapezoidal_X_Node>();
        top_x_node->left = left_trapezoid;
        top_x_node->right = right_trapezoid;
        top_x_node->segment = &segment;

        left_trapezoid->parents.push_back(top_x_node.get());
        right_trapezoid->parents.push_back(top_x_node.get());

        std::shared_ptr<Trapezoidal_Y_Node> top_y_node = std::make_shared<Trapezoidal_Y_Node>();

        top_x_node->parents.push_back(top_y_node.get());
        top_trapezoid->parents.push_back(top_y_node.get());

        top_y_node->point = segment.get_bottom_point();
        top_y_node->below = std::move(top_trapezoid);
        top_y_node->above = std::move(top_x_node);

        replace_leaf_node_with_subgraph(old_bottom_trapezoid, top_y_node);
    }
    else
    {
        //TODO: Fix top pointers back..
        //Create the final subgraph for the top trapezoids with the x_node of the segment as the root node
        std::shared_ptr<Trapezoidal_X_Node> top_x_node = std::make_shared<Trapezoidal_X_Node>();
        top_x_node->left = left_trapezoid;
        top_x_node->right = right_trapezoid;
        top_x_node->segment = &segment;

        left_trapezoid->parents.push_back(top_x_node.get());
        right_trapezoid->parents.push_back(top_x_node.get());

        replace_leaf_node_with_subgraph(old_bottom_trapezoid, top_x_node);
    }
}

std::vector<Trapezoidal_Leaf_Node*> Trapezoidal_Map::follow_segment(const Segment& query_segment)
{
    //TODO: Test if this goes correct with edge case for point on segment?
    assert(query_segment.start.y < query_segment.end.y);

    std::vector<Trapezoidal_Leaf_Node*> intersecting_trapezoids;

    //Find the trapezoid the starting point is inside of
    Trapezoidal_Leaf_Node* starting_trapezoid = root->query_start_point(query_segment);
    intersecting_trapezoids.push_back(starting_trapezoid);

    //Follow along the segment to find all intersecting trapezoids
    //TODO: Handle starting bounding box, flip -?
    //TODO: query_segment.end? not top?
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

//Replace leaf node in the graph with the new subgraph, this should reduce the shared_ptr count of the leaf node to 0
void Trapezoidal_Map::replace_leaf_node_with_subgraph(Trapezoidal_Leaf_Node* old_trapezoid, std::shared_ptr<Trapezoidal_Internal_Node> new_subgraph)
{
    if (!old_trapezoid->parents.empty())
    {
        for (Trapezoidal_Internal_Node* parent_node : old_trapezoid->parents)
        {
            parent_node->replace_child(old_trapezoid, new_subgraph);
            new_subgraph->parents.push_back(parent_node);
        }
    }
    else
    {
        root = new_subgraph;
    }
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
        return;
    }
    else if (bottom_right == old_bottom_neighbour)
    {
        bottom_right = new_bottom_neighbour;
        return;
    }

    assert(false);
    return;
}

void Trapezoidal_Leaf_Node::replace_top_neighbour(Trapezoidal_Leaf_Node* old_top_neighbour, Trapezoidal_Leaf_Node* new_top_neighbour)
{
    if (top_left == old_top_neighbour)
    {
        top_left = new_top_neighbour;
        return;
    }
    else if (top_right == old_top_neighbour)
    {
        top_right = new_top_neighbour;
        return;
    }

    assert(false);
    return;
}

Trapezoidal_Leaf_Node* Trapezoidal_Map::query_point(const Vec2& point)
{
    return root->query_point(point);
}

Trapezoidal_Leaf_Node* Trapezoidal_Leaf_Node::query_point(const Vec2& point)
{
    return this;
}

Trapezoidal_Leaf_Node* Trapezoidal_X_Node::query_point(const Vec2& point)
{
    //TODO: Point on segment
    if (point_right_of_segment(*segment, point))
    {
        //Right of, or on segment
        return right->query_point(point);
    }
    else
    {
        return left->query_point(point);
    }
}

Trapezoidal_Leaf_Node* Trapezoidal_Y_Node::query_point(const Vec2& point)
{
    //TODO: Point on point
    if (point.y >= this->point->y)
    {
        //Above or on point
        return above->query_point(point);
    }
    else
    {
        return below->query_point(point);
    }
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
