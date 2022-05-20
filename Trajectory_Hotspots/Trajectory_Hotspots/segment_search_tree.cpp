#include "pch.h"
#include "aabb.h"
#include "segment.h"
#include "segment_search_tree.h"

Segment_Search_Tree_Node::Segment_Search_Tree_Node() : left(nullptr), right(nullptr), node_start_t(0.f), node_end_t(0.f), segment(nullptr)
{
}

//Build the tree bottom-up from a lsit of ordered segments
Segment_Search_Tree_Node::Segment_Search_Tree_Node(const std::vector<Segment>& ordered_segments, const size_t start_index, const size_t end_index)
{
    if (end_index == start_index)
    {
        //Leaf node
        segment = &ordered_segments.at(start_index);

        bounding_box = segment->get_AABB();

        left = nullptr;
        right = nullptr;

        node_start_t = segment->start_t;
        node_end_t = segment->end_t;
    }
    else
    {
        //Internal node, split
        const size_t middle_index = (start_index + end_index) / 2;

        left = std::make_unique<Segment_Search_Tree_Node>(ordered_segments, start_index, middle_index);
        right = std::make_unique<Segment_Search_Tree_Node>(ordered_segments, middle_index + 1, end_index);

        bounding_box = AABB::combine(left->bounding_box, right->bounding_box);

        segment = nullptr;

        node_start_t = left->node_start_t;
        node_end_t = right->node_end_t;
    }
}

//Query tree, reutrns bounding box from start_t to end_t
AABB Segment_Search_Tree_Node::Query(const float start_t, const float end_t) const
{
    //TODO: Pass bounding box as ref to avoid construction?
    AABB bounding_box(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

    if (left != nullptr)
    {
        //Range completely contained in left
        if (left->node_start_t <= start_t && end_t <= left->node_end_t)
        {
            return left->Query(start_t, end_t);
        }//Query range starts in left
        else if (start_t < left->node_end_t)
        {
            bounding_box.combine(left->Query_Left(start_t));
        }
    }

    if (right != nullptr)
    {
        //Range completely contained in right
        if (right->node_start_t <= start_t && end_t <= right->node_end_t)
        {
            return right->Query(start_t, end_t);
        }//Query range ends in right
        else if (right->node_start_t < end_t)
        {
            bounding_box.combine(right->Query_Right(end_t));
        }
    }

    //leaf node, calculate segment portion (both points are in the segment)
    if (segment != nullptr)
    {
        const float segment_length_t = node_end_t - node_start_t;
        const float start_scalar = start_t - node_start_t;
        const float end_scalar = end_t - node_start_t;
        Vec2 segment_vec = segment->end - segment->start;

        Vec2 start_point = segment->start + start_scalar * segment_vec;
        Vec2 end_point = segment->start + end_scalar * segment_vec;

        Vec2 min(std::min(start_point.x, end_point.x), std::min(start_point.y, end_point.y));
        Vec2 max(std::max(start_point.x, end_point.x), std::max(start_point.y, end_point.y));

        return AABB(min, max);
    }

    return bounding_box;
}

//Query tree, returns bounding box from start_t to the last point contained in the (sub)tree
AABB Segment_Search_Tree_Node::Query_Left(const float start_t) const
{
    if (right != nullptr)
    {
        //Right fully contained in query range?
        if (start_t <= right->node_start_t)
        {
            AABB bounding_box = right->bounding_box;

            if (left != nullptr)
            {
                bounding_box.combine(left->Query_Left(start_t));
            }

            return bounding_box;
        }
        else
        {
            //Query range starts in right side, ignore left side
            return right->Query_Left(start_t);
        }
    }

    //Leaf node
    if (segment != nullptr)
    {
        //Calculate boundingbox from point at start_t to the endpoint of the segment
        float length_scalar = (node_end_t - start_t) / (node_end_t - node_start_t);
        Vec2 point_on_segment = segment->end + (length_scalar * (segment->start - segment->end));

        Vec2 min(std::min(point_on_segment.x, segment->end.x), std::min(point_on_segment.y, segment->end.y));
        Vec2 max(std::max(point_on_segment.x, segment->end.x), std::max(point_on_segment.y, segment->end.y));

        return AABB(min, max);
    }

    return AABB(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

}

//Query tree, returns bounding box from the first point in the (sub)tree to end_t
AABB Segment_Search_Tree_Node::Query_Right(const float end_t) const
{
    if (left != nullptr)
    {
        //Left side fully contained in query range?
        if (left->node_end_t <= end_t)
        {
            AABB bounding_box = left->bounding_box;

            if (right != nullptr)
            {
                bounding_box.combine(right->Query_Right(end_t));
            }

            return bounding_box;
        }
        else
        {
            //Query range start in left side, ignore right side
            return right->Query_Right(end_t);
        }
    }

    //Leaf node
    if (segment != nullptr)
    {
        //Calculate boundingbox from the startpoint of the segment to the point at end_t
        float length_scalar = (end_t - node_start_t) / (node_end_t - node_start_t);
        Vec2 point_on_segment = segment->start + (length_scalar * (segment->end - segment->start));

        Vec2 min(std::min(point_on_segment.x, segment->start.x), std::min(point_on_segment.y, segment->start.y));
        Vec2 max(std::max(point_on_segment.x, segment->start.x), std::max(point_on_segment.y, segment->start.y));

        return AABB(min, max);
    }

    return AABB(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
}

Segment_Search_Tree::Segment_Search_Tree(const std::vector<Segment>& ordered_segments)
{
    root = Segment_Search_Tree_Node(ordered_segments, 0, ordered_segments.size() - 1);
}
