#pragma once

namespace Segment_Intersection_Sweep_Line
{
    class Sweep_Line_Status_structure
    {
    public:
        Sweep_Line_Status_structure() : line_position(0.0f)
        {
        };

        class Node
        {
        public:
            Node(const int segment) : segment(segment) {};



            int segment; //index
            int height = 0;
            std::unique_ptr<Node> left = nullptr;
            std::unique_ptr<Node> right = nullptr;
            const Node* parent = nullptr;

            void calculate_height();
            int height_difference();
            const int get_right_neighbour(const std::vector<Segment>& segments, const Float line_position) const;
            const int get_left_neighbour(const std::vector<Segment>& segments, const Float line_position) const;

            void get_all_neighbours(const std::vector<Segment>& segments,
                const Float line_position,
                const Vec2& event_point,
                std::vector<int>& intersections,
                std::vector<int>& bottom_segments,
                int& most_left_intersecting_segment, int& most_right_intersecting_segment,
                int& left_neighbour, int& right_neighbour) const;

            bool get_intersecting_left_to_right(const std::vector<Segment>& segments, const Float line_position, const Vec2& event_point, std::vector<const Node*>& right_nodes) const;
            bool get_intersecting_right_to_left(const std::vector<Segment>& segments, const Float line_position, const Vec2& event_point, std::vector<const Node*>& left_nodes) const;


            void print_tree(Node* root, int spacing, std::string& tree_string) const;
        };

        Sweep_Line_Status_structure(Float line_position) : line_position(line_position) {};

        void insert(const std::vector<Segment>& segments, const int new_segment, int& left_node, int& right_node);
        void remove(const std::vector<Segment>& segments, const int segment_to_remove, int& left_node, int& right_node);
        bool contains(const std::vector<Segment>& segments, const Segment* search_segment);

        void set_line_position(const Float new_position) { line_position = new_position; };

        void get_all_nodes_on_point(
            const std::vector<Segment> segments,
            const Vec2& event_point,
            std::vector<int>& intersections,
            std::vector<int>& bottom_segments,
            int& most_left_intersecting_segment,
            int& most_right_intersecting_segment,
            int& left_neighbour,
            int& right_neighbour) const;

        Node* get_node(const std::vector<Segment>& segments, const Float& x_position) const;

        std::string print_tree() const;

        std::unique_ptr<Node> root;

    private:
        //TODO: Segment* vector here, ref?

        Float line_position;

        bool test_order_left_right(const std::vector<Segment>& segments, const int& new_segment, const std::unique_ptr<Node>& new_root) const;

        std::unique_ptr<Node> insert(std::unique_ptr<Node>&& node, const std::vector<Segment>& segments, const int new_segment, const Node*& added_node);
        std::unique_ptr<Node> add_to_subtree(std::unique_ptr<Node>&& root, const std::vector<Segment>& segments, const int new_segment, const Node*& added_node, const Node* parent);

        std::unique_ptr<Node> remove(std::unique_ptr<Node>&& node, const std::vector<Segment>& segments, const int segment_to_remove);
        std::unique_ptr<Node> remove_from_parent(std::unique_ptr<Node>&& parent, const std::vector<Segment>& segments, const int segment_to_remove);

        std::unique_ptr<Node> rotate_left(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_left_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right_left(std::unique_ptr<Node>&& old_root);

        void print_tree(Node* root, int spacing, std::string& tree_string) const;
    };

}