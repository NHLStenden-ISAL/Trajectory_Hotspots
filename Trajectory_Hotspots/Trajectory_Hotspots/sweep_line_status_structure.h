#pragma once

namespace Segment_Intersection_Sweep_Line
{

    class Sweep_Line_Status_structure
    {
    public:
        class Node
        {
        public:
            Node(Segment* segment) : segment(segment) {};

            Segment* segment;
            int height = 0;

            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
            Node* parent = nullptr;

            void calculate_height();
            int height_difference();
        };

        Sweep_Line_Status_structure(float line_position) : line_position(line_position) {};

        void insert(Segment* new_segment);
        std::unique_ptr<Node> insert(std::unique_ptr<Node>&& node, Segment* new_segment);
        std::unique_ptr<Node> add_to_subtree(std::unique_ptr<Node>&& root, Segment* new_segment);

        void remove(Segment* segment_to_remove);
        std::unique_ptr<Node> remove(std::unique_ptr<Node>&& node, Segment* segment_to_remove);
        std::unique_ptr<Node> remove_from_parent(std::unique_ptr<Node>&& parent, Segment* segment_to_remove);

        void rebalance();

        std::unique_ptr<Node> rotate_left(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_left_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right_left(std::unique_ptr<Node>&& old_root);

        bool contains(Segment* search_segment);

        std::unique_ptr<Node> root;
        float line_position;

    private:
        bool right_of_segment();
    };

    /// <summary>
    /// Inserts a new node into the tree, rebalancing when needed.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="segment"></param>
    inline void Sweep_Line_Status_structure::insert(Segment* new_segment)
    {
        if (root != nullptr)
        {
            root = insert(std::move(root), new_segment);
        }
        else
        {
            root = std::make_unique<Node>(new_segment);
        }
    }

    inline std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::insert(std::unique_ptr<Node>&& node, Segment* new_segment)
    {
        std::unique_ptr<Node> new_root = std::move(node);

        float current_x_position = new_segment->y_intersect(line_position);

        if (current_x_position <= new_root->segment->y_intersect(line_position))
        {
            new_root->left = add_to_subtree(std::move(new_root->left), new_segment);

            if (new_root->height_difference() == 2)
            {
                if (current_x_position <= new_root->left->segment->y_intersect(line_position))
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
            new_root->right = add_to_subtree(std::move(new_root->right), new_segment);
            if (new_root->height_difference() == -2)
            {
                if (current_x_position > new_root->right->segment->y_intersect(line_position))
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

    inline std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::add_to_subtree(std::unique_ptr<Node>&& root, Segment* new_segment)
    {
        if (root == nullptr)
        {
            return std::make_unique<Node>(new_segment);
        }
        else
        {
            return insert(std::move(root), new_segment);
        }
    }

    inline void Sweep_Line_Status_structure::remove(Segment* segment_to_remove)
    {
        if (root != nullptr)
        {
            root = remove(std::move(root), segment_to_remove);
        }
        else
        {
            return;
        }
    }

    inline std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::remove(std::unique_ptr<Node>&& node, Segment* segment_to_remove)
    {
        std::unique_ptr<Node> new_root = std::move(node);

        float current_x_position = segment_to_remove->y_intersect(line_position);

        if (*segment_to_remove == *new_root->segment)
        {
            if (new_root->left == nullptr)
            {
                //Simply return the right subtree
                return std::move(new_root->right);
            }

            //Find most right of lift subtree
            std::unique_ptr<Node> child = std::move(new_root->left);
            while (child->right != nullptr)
            {
                child = std::move(child->right);
            }

            //Hoist the segment from the left child to the new root node and remove it from left (balancing in the process)
            Segment* child_segment = child->segment;
            new_root->left = remove_from_parent(std::move(new_root->left), child_segment);
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
        else if (current_x_position < new_root->segment->y_intersect(line_position))
        {
            new_root->left = remove_from_parent(std::move(new_root->left), segment_to_remove);

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
            new_root->right = remove_from_parent(std::move(new_root->right), segment_to_remove);

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

    inline std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::remove_from_parent(std::unique_ptr<Node>&& parent, Segment* segment_to_remove)
    {
        if (parent != nullptr)
        {
            return remove(std::move(parent), segment_to_remove);
        }
        else
        {
            return nullptr;
        }
    }

    inline bool Sweep_Line_Status_structure::contains(Segment* search_segment)
    {
        Node* node = root.get();

        float current_x_position = search_segment->y_intersect(line_position);

        while (node != nullptr)
        {
            if (current_x_position < node->segment->y_intersect(line_position))
            {
                node = node->left.get();
            }
            else if (current_x_position > node->segment->y_intersect(line_position))
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


    inline std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_left(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> new_root = std::move(old_root->right);
        old_root->right = std::move(new_root->left);
        new_root->left = std::move(old_root);

        new_root->left->calculate_height();

        return new_root;
    }

    inline std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_right(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> new_root = std::move(old_root->left);
        old_root->left = std::move(new_root->right);
        new_root->right = std::move(old_root);

        new_root->right->calculate_height();

        return new_root;
    }

    inline std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_right_left(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> right = std::move(old_root->right);
        std::unique_ptr<Node> new_root = std::move(right->left);
        right->left = std::move(new_root->right);
        old_root->right = std::move(new_root->left);

        new_root->left = std::move(old_root);
        new_root->right = std::move(right);

        new_root->right->calculate_height();
        new_root->left->calculate_height();

        return new_root;
    }

    inline std::unique_ptr<typename Sweep_Line_Status_structure::Node> Sweep_Line_Status_structure::rotate_left_right(std::unique_ptr<Node>&& old_root)
    {
        std::unique_ptr<Node> left = std::move(old_root->left);
        std::unique_ptr<Node> new_root = std::move(left->right);
        left->right = std::move(new_root->left);
        old_root->left = std::move(new_root->right);

        new_root->left = std::move(left);
        new_root->right = std::move(old_root);

        new_root->left->calculate_height();
        new_root->right->calculate_height();

        return new_root;
    }

    // Sets the height of a subtree based on its child subtrees
    inline void Sweep_Line_Status_structure::Node::calculate_height()
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
    inline int Sweep_Line_Status_structure::Node::height_difference()
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

}