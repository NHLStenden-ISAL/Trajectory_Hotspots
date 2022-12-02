#include "pch.h"
#include "sweep_line_status_structure.h"

namespace Segment_Intersection_Sweep_Line
{
    /// <summary>
    /// Inserts a new node into the tree, rebalancing when needed.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="segment"></param>
    void Sweep_Line_Status_structure::insert(const std::vector<Segment>& segments, const int new_segment, int& left_node, int& right_node)
    {
        if (root != nullptr)
        {
            const Node* node = nullptr;
            root = insert(std::move(root), segments, new_segment, node);

            if (node != nullptr)
            {
                left_node = node->get_left_neighbour(segments, line_position);
                right_node = node->get_right_neighbour(segments, line_position);
            }
            // get neighbours
            //left_node = 
            //right_node = 
        }
        else
        {
            root = std::make_unique<Node>(new_segment);
        }
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::insert(std::unique_ptr<Node>&& node, const std::vector<Segment>& segments, const int new_segment, const Node*& added_node)
    {
        std::unique_ptr<Node> new_root = std::move(node);
        //TODO: maybe put in a function returns which is smaller
        float current_x_position = segments.at(new_segment).y_intersect(line_position);
        float node_x_position = segments.at(new_root->segment).y_intersect(line_position);

        //A  = new_root-segment
        //B  = new_segment
        // CROSS A x B
        // if cross is negative, then the new segment is on the left

        Vec2 node_segment = *segments.at(new_root->segment).get_bottom_point() - Vec2(current_x_position, line_position);
        Vec2 new_segment_vec = *segments.at(new_segment).get_bottom_point() - Vec2(current_x_position, line_position);

        if (current_x_position < node_x_position || node_segment.cross(new_segment_vec) < 0)
        {
            new_root->left = add_to_subtree(std::move(new_root->left), segments, new_segment, added_node, new_root.get());

            if (new_root->height_difference() == 2)
            {
                if (current_x_position <= segments.at(new_root->left->segment).y_intersect(line_position))
                {
                    new_root = rotate_right(std::move(new_root));
                }
                else
                {
                    new_root = rotate_left_right(std::move(new_root));
                }
            }
        }
        else
        {
            new_root->right = add_to_subtree(std::move(new_root->right), segments, new_segment, added_node, new_root.get());

            if (new_root->height_difference() == -2)
            {
                if (current_x_position > segments.at(new_root->right->segment).y_intersect(line_position))
                {
                    new_root = rotate_left(std::move(new_root));
                }
                else
                {
                    new_root = rotate_right_left(std::move(new_root));
                }
            }
        }
        new_root->calculate_height();

        return new_root;
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::add_to_subtree(std::unique_ptr<Node>&& root, const std::vector<Segment>& segments, const int new_segment, const Node*& added_node, const Node* parent)
    {
        if (root == nullptr)
        {
            std::unique_ptr<Node> new_leaf_node = std::make_unique<Node>(new_segment);
            new_leaf_node->parent = parent;

            added_node = new_leaf_node.get();
            return new_leaf_node;
        }
        else
        {
            return insert(std::move(root), segments, new_segment, added_node);
        }
    }

    void Sweep_Line_Status_structure::remove(const std::vector<Segment>& segments, const int segment_to_remove, int& left_node, int& right_node)
    {
        if (root != nullptr)
        {
            left_node = root.get()->get_left_neighbour(segments, line_position);
            right_node = root.get()->get_right_neighbour(segments, line_position);

            root = remove(std::move(root), segments, segment_to_remove);
        }
        else
        {
            return;
        }
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::remove(std::unique_ptr<Node>&& node, const std::vector<Segment>& segments, const int segment_to_remove)
    {
        std::unique_ptr<Node> new_root = std::move(node);

        float current_x_position = segments.at(segment_to_remove).y_intersect(line_position);

        if (&segments.at(segment_to_remove) == &segments.at(new_root->segment))
        {
            if (new_root->left == nullptr)
            {
                if (new_root->right != nullptr)
                {
                    new_root->right->parent = new_root->parent;
                }
                //Simply return the right subtree
                return std::move(new_root->right);
            }

            //Find most right of left subtree
            std::unique_ptr<Node> child = std::move(new_root->left);
            while (child->right != nullptr)
            {
                child = std::move(child->right);
            }

            //Hoist the segment from the left child to the new root node and remove it from left (balancing in the process)
            const int child_segment = child->segment;
            new_root->left = remove_from_parent(std::move(new_root->left), segments, child_segment);
            new_root->segment = child_segment;

            //Balance tree if necessary
            if (new_root->height_difference() == -2)
            {
                if (new_root->right->height_difference() <= 0)
                {
                    new_root = rotate_left(std::move(new_root));
                }
                else
                {
                    new_root = rotate_right_left(std::move(new_root));
                }
            }
        }
        else if (current_x_position < segments.at(new_root->segment).y_intersect(line_position))
        {
            new_root->left = remove_from_parent(std::move(new_root->left), segments, segment_to_remove);

            if (new_root->height_difference() == -2)
            {
                if (new_root->right->height_difference() <= 0)
                {
                    new_root = rotate_left(std::move(new_root));
                }
                else
                {
                    new_root = rotate_right_left(std::move(new_root));
                }
            }
        }
        else
        {
            new_root->right = remove_from_parent(std::move(new_root->right), segments, segment_to_remove);

            if (new_root->height_difference() == 2)
            {
                if (new_root->left->height_difference() >= 0)
                {
                    new_root = rotate_right(std::move(new_root));
                }
                else
                {
                    new_root = rotate_left_right(std::move(new_root));
                }
            }
        }

        new_root->calculate_height();

        return new_root;
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::remove_from_parent(std::unique_ptr<Node>&& parent, const std::vector<Segment>& segments, const int segment_to_remove)
    {
        if (parent != nullptr)
        {
            return remove(std::move(parent), segments, segment_to_remove);
        }
        else
        {
            return nullptr;
        }
    }

    //TODO: returns true if another segment intersect at the same point.
    bool Sweep_Line_Status_structure::contains(const std::vector<Segment>& segments, const Segment* search_segment)
    {
        Node* node = root.get();

        float current_x_position = search_segment->y_intersect(line_position);

        while (node != nullptr)
        {
            if (current_x_position < segments.at(node->segment).y_intersect(line_position))
            {
                node = node->left.get();
            }
            else if (current_x_position > segments.at(node->segment).y_intersect(line_position))
            {
                node = node->right.get();
            }
            else
            {
                return true;
            }
        }

        return false;
    }
    //TODO: returns true if another segment intersect at the same point.
    Sweep_Line_Status_structure::Node* Sweep_Line_Status_structure::get_node(const std::vector<Segment>& segments, const Vec2& event_point) const
    {
        Node* node = root.get();
        //TODO: Is this correct?
        float current_x_position = event_point.x;

        while (node != nullptr)
        {
            if (current_x_position < segments.at(node->segment).y_intersect(line_position))
            {
                node = node->left.get();
            }
            else if (current_x_position > segments.at(node->segment).y_intersect(line_position))
            {
                node = node->right.get();
            }
            else
            {
                return node;
            }
        }

        return nullptr;
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_left(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> new_root = std::move(old_root->right);
        new_root->parent = old_root->parent;
        old_root->parent = new_root.get();

        old_root->right = std::move(new_root->left);
        if (old_root->right != nullptr)
        {
            old_root->right->parent = old_root.get();
        }
        new_root->left = std::move(old_root);

        new_root->left->calculate_height();

        return new_root;
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_right(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> new_root = std::move(old_root->left);
        new_root->parent = old_root->parent;
        old_root->parent = new_root.get();

        old_root->left = std::move(new_root->right);
        if (old_root->left != nullptr)
        {
            old_root->left->parent = old_root.get();
        }
        new_root->right = std::move(old_root);

        new_root->right->calculate_height();

        return new_root;
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_right_left(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> right = std::move(old_root->right);
        std::unique_ptr<Node> new_root = std::move(right->left);
        new_root->parent = old_root->parent;

        right->left = std::move(new_root->right);
        if (right->left != nullptr)
        {
            right->left->parent = right.get();
        }

        old_root->right = std::move(new_root->left);
        if (old_root->right != nullptr)
        {
            old_root->right->parent = old_root.get();
        }


        new_root->left = std::move(old_root);
        if (new_root->left != nullptr)
        {
            new_root->left->parent = new_root.get();
        }
        new_root->right = std::move(right);
        if (new_root->right != nullptr)
        {
            new_root->right->parent = new_root.get();
        }
        new_root->right->calculate_height();
        new_root->left->calculate_height();

        return new_root;
    }

    std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_left_right(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> left = std::move(old_root->left);
        std::unique_ptr<Node> new_root = std::move(left->right);
        new_root->parent = old_root->parent;

        left->right = std::move(new_root->left);
        if (left->right != nullptr)
        {
            left->right->parent = left.get();
        }


        old_root->left = std::move(new_root->right);
        if (old_root->left != nullptr)
        {
            old_root->left->parent = old_root.get();
        }
        new_root->right = std::move(old_root);
        if (new_root->right != nullptr)
        {
            new_root->right->parent = new_root.get();
        }
        new_root->left = std::move(left);
        if (new_root->left != nullptr)
        {
            new_root->left->parent = new_root.get();
        }
        new_root->left->calculate_height();
        new_root->right->calculate_height();

        return new_root;
    }

    // Sets the height of a subtree based on its child subtrees
    void Sweep_Line_Status_structure::Node::calculate_height()
    {
        height = 0;

        if (left != nullptr)
        {
            height = std::max(height, left->height);
        }

        if (right != nullptr)
        {
            height = std::max(height, right->height);
        }

        height += 1;
    }


    // Calculates the difference in height between a nodes left and right subtree
    int Sweep_Line_Status_structure::Node::height_difference()
    {
        int left_height = 0;
        int right_height = 0;

        if (left != nullptr)
        {
            left_height = left->height + 1;
        }

        if (right != nullptr)
        {
            right_height = right->height + 1;
        }

        return left_height - right_height;
    }

    // Get Left neighbour of the Segment and not the node!
    const int Sweep_Line_Status_structure::Node::get_left_neighbour(const std::vector<Segment>& segments, const float line_position) const
    {
        // voor linker neighbour grootste getal in de linker kant van root
        // voor de rechter neighbour kleinste getal in de rechter kan van root
        // als groot

        // Check if right pointer of left node is empty or not
        if (left != nullptr)
        {
            const Node* current_left = left.get();

            //Traverse to the most right leaf node
            while (current_left->right != nullptr)
            {
                current_left = current_left->right.get();
            }
            return current_left->segment;
        }

        //No left subtree, one of the parents can be the left neighbour
        if (parent != nullptr)
        {
            float current_x_position = segments.at(segment).y_intersect(line_position);
            const Node* current_parent = parent;
            while (current_x_position < segments.at(current_parent->segment).y_intersect(line_position))
            {
                current_parent = current_parent->parent;
                if (current_parent == nullptr)
                {
                    return -1;
                }
            }
            return current_parent->segment;
        }

        return -1;
    }

    const int Sweep_Line_Status_structure::Node::get_right_neighbour(const std::vector<Segment>& segments, const float line_position) const
    {
        // Check if left pointer of right node is empty or not
        if (right != nullptr)
        {
            const Node* current_right = right.get();

            //Traverse to the most left leaf node
            while (current_right->left != nullptr)
            {
                current_right = current_right->left.get();
            }
            return current_right->segment;
        }

        //No right subtree, one of the parents can be the right neighbour
        if (parent != nullptr)
        {
            float current_x_position = segments.at(segment).y_intersect(line_position);
            const Node* current_parent = parent;
            while (current_x_position > segments.at(current_parent->segment).y_intersect(line_position))
            {
                current_parent = current_parent->parent;
                if (current_parent == nullptr)
                {
                    return -1;
                }
            }
            return current_parent->segment;
        }

        return -1;
    }

    void Sweep_Line_Status_structure::Node::get_all_neighbours(
        const std::vector<Segment>& segments,
        const float line_position,
        const Vec2& event_point,
        std::vector<int>& intersections,
        std::vector<int>& bottom_segments,
        int& most_left_intersecting_segment, int& most_right_intersecting_segment,
        int& left_neighbour, int& right_neighbour
    ) const
    {
        //TODO: Can we clean this up?
        std::vector<const Node*> neighbouring_nodes;
        neighbouring_nodes.push_back(this);

        if (left != nullptr)
        {
            //Get all nodes intersecting the same point, the last element returned is the most left node
            left->get_left_neighbours(segments, line_position, event_point, neighbouring_nodes);
        }

        //Set the first left neighbour that doesnt intersect, there should always be at least the root node in the list
        left_neighbour = neighbouring_nodes.back()->get_left_neighbour(segments, line_position);

        //Seperate the segments in bottom and intersection segments
        for (const Node* node : neighbouring_nodes)
        {
            if (*segments[node->segment].get_bottom_point() == event_point)
            {
                bottom_segments.push_back(node->segment);
            }
            else
            {
                intersections.push_back(node->segment);
            }
        }

        if (!intersections.empty())
        {
            //The last segment is the most left node in the tree after the get_left_neighbours call
            most_left_intersecting_segment = intersections.back();
        }

        neighbouring_nodes.clear();

        if (right != nullptr)
        {
            //Get all nodes intersecting the same point, the last element returned is the most right node
            right->get_right_neighbours(segments, line_position, event_point, neighbouring_nodes);
        }

        if (!neighbouring_nodes.empty())
        {
            //Set the first right neighbour that doesnt intersect
            right_neighbour = neighbouring_nodes.back()->get_right_neighbour(segments, line_position);

            //Seperate the segments in bottom and intersection segments
            for (const Node* node : neighbouring_nodes)
            {
                if (*segments[node->segment].get_bottom_point() == event_point)
                {
                    bottom_segments.push_back(node->segment);
                }
                else
                {
                    intersections.push_back(node->segment);
                }
            }
        }
        else
        {
            //If there are no right neighbours, get the right neighbour of the root node
            right_neighbour = this->get_right_neighbour(segments, line_position);
        }

        if (!intersections.empty())
        {
            //The most right intersecting segment in the tree is the last node in the insersections list after the get_right_neighbours call
            most_right_intersecting_segment = intersections.back();
        }
    }

    bool Sweep_Line_Status_structure::Node::get_right_neighbours(const std::vector<Segment>& segments, const float line_position, const Vec2& event_point, std::vector<const Node*>& right_nodes) const
    {
        if (left != nullptr)
        {
            //If left subtree contained a segment not on this point, early out.
            if (!left->get_right_neighbours(segments, line_position, event_point, right_nodes))
            {
                return false;
            }
        }

        if (nearly_equal(event_point.x, segments[segment].y_intersect(line_position)))
        {
            right_nodes.emplace_back(this);
        }
        else
        {
            return false;
        }

        if (right != nullptr)
        {
            //If right subtree contained a segment not on this point, early out.
            if (!right->get_right_neighbours(segments, line_position, event_point, right_nodes))
            {
                return false;
            }
        }

        return true;
    }

    bool Sweep_Line_Status_structure::Node::get_left_neighbours(const std::vector<Segment>& segments, const float line_position, const Vec2& event_point, std::vector<const Node*>& left_nodes) const
    {
        if (right != nullptr)
        {
            //If right subtree contained a segment not on this point, early out.
            if (!right->get_right_neighbours(segments, line_position, event_point, left_nodes))
            {
                return false;
            }
        }

        if (nearly_equal(event_point.x, segments[segment].y_intersect(line_position)))
        {
            left_nodes.emplace_back(this);
        }
        else
        {
            return false;
        }

        if (left != nullptr)
        {
            //If left subtree contained a segment not on this point, early out.
            if (!left->get_right_neighbours(segments, line_position, event_point, left_nodes))
            {
                return false;
            }
        }

        return true;
    }

    void Sweep_Line_Status_structure::swap_elements(const std::vector<Segment>& segments, int segment_index_1, int segment_index_2, int& left_segment, int& right_segment)
    {
        //TODO: test this with left/right neighbour function
        Node* node1 = find_node(segments, segment_index_1);
        Node* node2 = find_node(segments, segment_index_2);
        node1->segment = segment_index_2;
        node2->segment = segment_index_1;

        right_segment = node1->get_right_neighbour(segments, line_position);
        left_segment = node2->get_left_neighbour(segments, line_position);


    }

    //TODO: returns true if another segment intersect at the same point.
    Sweep_Line_Status_structure::Node* Sweep_Line_Status_structure::find_node(const std::vector<Segment>& segments, int segment_index)
    {
        Node* node = root.get();

        float current_x_position = segments.at(segment_index).y_intersect(line_position);

        while (node != nullptr)
        {
            if (current_x_position < segments.at(node->segment).y_intersect(line_position))
            {
                node = node->left.get();
            }
            else if (current_x_position > segments.at(node->segment).y_intersect(line_position))
            {
                node = node->right.get();
            }
            else
            {
                return node;
            }
        }

        return nullptr;
    }

    void Sweep_Line_Status_structure::get_all_nodes_on_point(
        const std::vector<Segment> segments,
        const Vec2& event_point,
        std::vector<int>& intersections,
        std::vector<int>& bottom_segments,
        int& most_left_intersecting_segment,
        int& most_right_intersecting_segment,
        int& left_neighbour,
        int& right_neighbour) const
    {
        //Find the first root node containing a segment intersecting the given event point
        const Node* event_node = get_node(segments, event_point);

        if (event_node != nullptr)
        {
            //Find all the neighbouring nodes that contain segments that intersect the point as well, these are adjacent in the tree
            event_node->get_all_neighbours(segments, line_position, event_point, intersections, bottom_segments, most_left_intersecting_segment, most_right_intersecting_segment, left_neighbour, right_neighbour);
        }
    }
}