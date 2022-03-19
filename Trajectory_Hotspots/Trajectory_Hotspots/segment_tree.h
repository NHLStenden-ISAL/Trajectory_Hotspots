#pragma once

//TODO: Needs constructor
//TODO: Needs testing
class Segment_Tree_Node
{
public:


    //Query tree, reutrns bounding box from start_t to end_t
    AABB Query(const float start_t, const float end_t) const;

    //Query tree, returns bounding box from start_t to the last point contained in the (sub)tree
    AABB Query_Left(const float start_t) const;

    //Query tree, returns bounding box from the first point in the (sub)tree to end_t
    AABB Query_Right(const float end_t) const;

    Segment_Tree_Node* left;
    Segment_Tree_Node* right;

    float node_start_t;
    float node_end_t;

    Segment* segment;

    AABB bounding_box;
};

class Segment_Tree
{
public:

    AABB Query(const float start_t, const float end_t) const
    {
        return root.Query(start_t, end_t);
    }

    Segment_Tree_Node root;


};