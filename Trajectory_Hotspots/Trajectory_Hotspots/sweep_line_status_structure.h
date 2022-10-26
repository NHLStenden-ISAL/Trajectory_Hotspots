#pragma once

namespace Segment_Intersection_Sweep_Line
{

    class Sweep_Line_Status_structure
    {
    public:

        class Node
        {
        public:
            Node(const Segment* segment) : segment(segment) {};

            const Segment* segment;
            int height = 0;

            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
            const Node* parent = nullptr;

            void calculate_height();
            int height_difference();
            const Segment* get_right_neighbour(const float line_position) const;
            const Segment* get_left_neighbour(const float line_position) const;
        };

        Sweep_Line_Status_structure(float line_position) : line_position(line_position) {};

        void insert(const Segment* new_segment, const Segment*& left_node, const Segment*& right_node);
        void remove(const Segment* segment_to_remove);
        bool contains(const Segment* search_segment);
        
        void set_line_position(const float new_position) { line_position = new_position; };

        std::unique_ptr<Node> root;

    private:

        float line_position;
        
        std::unique_ptr<Node> insert(std::unique_ptr<Node>&& node, const Segment* new_segment,const Node*& added_node);
        std::unique_ptr<Node> add_to_subtree(std::unique_ptr<Node>&& root, const Segment* new_segment, const Node*& added_node, const Node* parent);

        std::unique_ptr<Node> remove(std::unique_ptr<Node>&& node, const Segment* segment_to_remove);
        std::unique_ptr<Node> remove_from_parent(std::unique_ptr<Node>&& parent, const Segment* segment_to_remove);

        std::unique_ptr<Node> rotate_left(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_left_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right_left(std::unique_ptr<Node>&& old_root);


    };

}