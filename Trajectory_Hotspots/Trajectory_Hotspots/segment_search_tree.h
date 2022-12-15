#pragma once

class Segment_Search_Tree_Node
{
public:

    Segment_Search_Tree_Node();

    //Build the tree bottom-up from a list of ordered segments
    Segment_Search_Tree_Node(const std::vector<Segment>& ordered_segments, const size_t start_index, const size_t end_index);

    //Query tree, returns bounding box from start_t to end_t
    AABB Query(const Float start_t, const Float end_t) const;

    //Query tree, returns bounding box from start_t to the last point contained in the (sub)tree
    AABB Query_Left(const Float start_t) const;

    //Query tree, returns bounding box from the first point in the (sub)tree to end_t
    AABB Query_Right(const Float end_t) const;

    std::unique_ptr<Segment_Search_Tree_Node> left;
    std::unique_ptr<Segment_Search_Tree_Node> right;

    Float node_start_t;
    Float node_end_t;

    const Segment* segment;

    AABB bounding_box;

};

class Segment_Search_Tree
{
public:

    Segment_Search_Tree(const std::vector<Segment>& ordered_segments);

    AABB Query(const Float start_t, const Float end_t) const
    {
        return root.Query(start_t, end_t);
    }

    Segment_Search_Tree_Node root;

};