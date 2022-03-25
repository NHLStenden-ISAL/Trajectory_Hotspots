#include "pch.h"
#include "segment_search_tee_inheritance.h"

Segment_Search_Tree_I_Node::Segment_Search_Tree_I_Node()
{
}

Segment_Search_Tree_I_Node::Segment_Search_Tree_I_Node(std::unique_ptr<Segment_Search_Tree_I_Base> left, std::unique_ptr<Segment_Search_Tree_I_Base> right)
{
}

Segment_Search_Tree_I_Node::Segment_Search_Tree_I_Node(const std::vector<Segment>& ordered_segments, const size_t start_index, const size_t end_index)
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

        left = std::make_unique<Segment_Search_Tree_I_Node>(ordered_segments, start_index, middle_index);
        right = std::make_unique<Segment_Search_Tree_I_Node>(ordered_segments, middle_index + 1, end_index);

        bounding_box = AABB::combine(left->bounding_box, right->bounding_box);

        node_start_t = left->node_start_t;
        node_end_t = right->node_end_t;
    }
}

AABB Segment_Search_Tree_I_Node::Query(const float start_t, const float end_t) const
{
    AABB bounding_box(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

    //Range completely contained in left
    if (left->node_start_t <= start_t && end_t <= left->node_end_t)
    {
        return left->Query(start_t, end_t);
    }//Query range starts in left
    else if (start_t < left->node_end_t)
    {
        bounding_box.combine(left->Query_Left(start_t));
    }

    //Range completely contained in right
    if (right->node_start_t <= start_t && end_t <= right->node_end_t)
    {
        return right->Query(start_t, end_t);
    }//Query range ends in right
    else if (right->node_start_t < end_t)
    {
        bounding_box.combine(right->Query_Right(end_t));
    }

    return bounding_box;
}

AABB Segment_Search_Tree_I_Node::Query_Left(const float start_t) const
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

AABB Segment_Search_Tree_I_Node::Query_Right(const float end_t) const
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

Segment_Search_Tree_I::Segment_Search_Tree_I(const std::vector<Segment>& ordered_segments)
{
    std::vector<const size_t> start_indices = { 0 };
    std::vector<const size_t> end_indices = { ordered_segments.size() - 1 };
    std::vector<Segment_Search_Tree_I_Node*> current_nodes = { (Segment_Search_Tree_I_Node*)root.get() }; //TODO: Fix this..

    //TODO: Iets met cast

    while (!start_indices.empty())
    {
        const size_t start_index = start_indices.at(start_indices.size() - 1);
        const size_t end_index = end_indices.at(end_indices.size() - 1);
        Segment_Search_Tree_I_Node* current_node = current_nodes.at(current_nodes.size() - 1);

        start_indices.pop_back();
        end_indices.pop_back();
        current_nodes.pop_back();

        if (end_index == start_index)
        {

            //Leaf node
            segment = &ordered_segments.at(start_index);

            bounding_box = segment->get_AABB();

            node_start_t = segment->start_t;
            node_end_t = segment->end_t;
        }
        else
        {
            //Internal node, split
            const size_t middle_index = (start_index + end_index) / 2;

            Segment_Search_Tree_I_Base* left;
            Segment_Search_Tree_I_Base* right;

            if (start_index == middle_index)
            {
                //Left leaf node
                left = new Segment_Search_Tree_I_Leaf(&ordered_segments.at(start_index), ordered_segments.at(start_index).start_t, ordered_segments.at(start_index).end_t, ordered_segments.at(start_index).get_AABB());
            }
            else
            {
                left = new Segment_Search_Tree_I_Node();
                start_indices.push_back(start_index);
                end_indices.push_back(middle_index);
                current_nodes.push_back(left);
            }

            if (middle_index + 1 == end_index)
            {
                //Right leaf node
                right = new Segment_Search_Tree_I_Leaf(&ordered_segments.at(end_index), ordered_segments.at(end_index).start_t, ordered_segments.at(end_index).end_t, ordered_segments.at(end_index).get_AABB());
                start_indices.push_back(middle_index + 1);
                end_indices.push_back(end_index);
                current_nodes.push_back(right);
            }
            else
            {
                left = new Segment_Search_Tree_I_Node();


            }


            left = std::make_unique<Segment_Search_Tree_Node>(ordered_segments, start_index, middle_index);
            right = std::make_unique<Segment_Search_Tree_Node>(ordered_segments, middle_index + 1, end_index);

            bounding_box = AABB::combine(left->bounding_box, right->bounding_box);

            segment = nullptr;

            node_start_t = left->node_start_t;
            node_end_t = right->node_end_t;
        }
    }


}

Segment_Search_Tree_I_Base::Segment_Search_Tree_I_Base()
{

}

Segment_Search_Tree_I_Base::Segment_Search_Tree_I_Base(const float node_start_t, const float node_end_t, const AABB& bounding_box) :
    node_start_t(node_start_t), node_end_t(node_end_t), bounding_box(bounding_box)
{
}

Segment_Search_Tree_I_Leaf::Segment_Search_Tree_I_Leaf(const Segment* segment, const float node_start_t, const float node_end_t, const AABB& bounding_box) :
    Segment_Search_Tree_I_Base(node_start_t, node_end_t, bounding_box), segment(segment)
{
}

AABB Segment_Search_Tree_I_Leaf::Query(const float start_t, const float end_t) const
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

AABB Segment_Search_Tree_I_Leaf::Query_Left(const float start_t) const
{
    //Calculate boundingbox from point at start_t to the endpoint of the segment
    float length_scalar = (node_end_t - start_t) / (node_end_t - node_start_t);
    Vec2 point_on_segment = segment->end + (length_scalar * (segment->start - segment->end));

    Vec2 min(std::min(point_on_segment.x, segment->end.x), std::min(point_on_segment.y, segment->end.y));
    Vec2 max(std::max(point_on_segment.x, segment->end.x), std::max(point_on_segment.y, segment->end.y));

    return AABB(min, max);
}

AABB Segment_Search_Tree_I_Leaf::Query_Right(const float end_t) const
{
    //Calculate boundingbox from the startpoint of the segment to the point at end_t
    float length_scalar = (end_t - node_start_t) / (node_end_t - node_start_t);
    Vec2 point_on_segment = segment->start + (length_scalar * (segment->end - segment->start));

    Vec2 min(std::min(point_on_segment.x, segment->start.x), std::min(point_on_segment.y, segment->start.y));
    Vec2 max(std::max(point_on_segment.x, segment->start.x), std::max(point_on_segment.y, segment->start.y));

    return AABB(min, max);
}

