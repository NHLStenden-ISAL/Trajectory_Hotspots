#pragma once

class Segment_Search_Tree_Node
{
public:

    //Segment_Search_Tree_Node();

    //Build the tree bottom-up from a list of ordered segments
    Segment_Search_Tree_Node(const std::vector<Segment>& ordered_segments, const size_t start_index, const size_t end_index);

    //Query tree, returns bounding box from start_t to end_t
    AABB query(const Float start_t, const Float end_t) const;

    //Query tree, returns bounding box from start_t to the last point contained in the (sub)tree
    AABB query_left(const Float start_t) const;

    //Query tree, returns bounding box from the first point in the (sub)tree to end_t
    AABB query_right(const Float end_t) const;

    //Query tree, returns segment index that contains t (or first/last when before/after range)
    int query(const Float t) const;

    std::unique_ptr<Segment_Search_Tree_Node> left;
    std::unique_ptr<Segment_Search_Tree_Node> right;

    Float node_start_t;
    Float node_end_t;

    int segment_index;
    const std::vector<Segment>& segment_list;

    AABB bounding_box;
};

class Segment_Search_Tree
{
public:

    //Build the tree bottom-up from a list of ordered segments
    Segment_Search_Tree(const std::vector<Segment>& ordered_segments);

    //Query tree, reutrns bounding box from start_t to end_t
    AABB query(const Float start_t, const Float end_t) const
    {
        return root.query(start_t, end_t);
    }

    //Query tree, returns segment index that contains t (or first/last when before/after range)
    int query(const Float t) const
    {
        return root.query(t);
    }

    Segment_Search_Tree_Node root;

};