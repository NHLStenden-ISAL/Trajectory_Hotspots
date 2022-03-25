#pragma once

//TODO: Needs testing

class Segment_Search_Tree_I_Base
{
public:

    Segment_Search_Tree_I_Base();
    Segment_Search_Tree_I_Base(const float node_start_t, const float node_end_t, const AABB& bounding_box);
    //Query tree, reutrns bounding box from start_t to end_t
    virtual AABB Query(const float start_t, const float end_t) const = 0;

    //Query tree, returns bounding box from start_t to the last point contained in the (sub)tree
    virtual AABB Query_Left(const float start_t) const = 0;

    //Query tree, returns bounding box from the first point in the (sub)tree to end_t
    virtual AABB Query_Right(const float end_t) const = 0;

    float node_start_t;
    float node_end_t;

    AABB bounding_box;

};

class Segment_Search_Tree_I_Node : public Segment_Search_Tree_I_Base
{
public:

    Segment_Search_Tree_I_Node();

    Segment_Search_Tree_I_Node(std::unique_ptr<Segment_Search_Tree_I_Base> left, std::unique_ptr<Segment_Search_Tree_I_Base> right);

    //Build the tree bottom-up from a list of ordered segments
    Segment_Search_Tree_I_Node(const std::vector<Segment>& ordered_segments, const size_t start_index, const size_t end_index);

    //Query tree, reutrns bounding box from start_t to end_t
    AABB Query(const float start_t, const float end_t) const override;

    //Query tree, returns bounding box from start_t to the last point contained in the (sub)tree
    AABB Query_Left(const float start_t) const override;

    //Query tree, returns bounding box from the first point in the (sub)tree to end_t
    AABB Query_Right(const float end_t) const override;

private:

    std::unique_ptr<Segment_Search_Tree_I_Base> left;
    std::unique_ptr<Segment_Search_Tree_I_Base> right;

};

class Segment_Search_Tree_I_Leaf : public Segment_Search_Tree_I_Base
{
public:

    Segment_Search_Tree_I_Leaf(const Segment* segment, const float node_start_t, const float node_end_t, const AABB& bounding_box);

    //Query tree, reutrns bounding box from start_t to end_t
    AABB Query(const float start_t, const float end_t) const override;

    //Query tree, returns bounding box from start_t to the last point contained in the (sub)tree
    AABB Query_Left(const float start_t) const override;

    //Query tree, returns bounding box from the first point in the (sub)tree to end_t
    AABB Query_Right(const float end_t) const override;

    const Segment* segment;
};

class Segment_Search_Tree_I
{
public:

    Segment_Search_Tree_I(const std::vector<Segment>& ordered_segments);

    AABB Query(const float start_t, const float end_t) const
    {
        return root->Query(start_t, end_t);
    }

    std::unique_ptr<Segment_Search_Tree_I_Base> root;

};