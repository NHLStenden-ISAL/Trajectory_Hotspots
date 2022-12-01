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
            const int get_right_neighbour(const std::vector<Segment>& segments, const float line_position) const;
            const int get_left_neighbour(const std::vector<Segment>& segments, const float line_position) const;
            
            void get_all_neighbours(const std::vector<Segment>& segments, const float line_position, const Vec2& event_point, std::vector<const Node*>& neighbouring_nodes, int& left_neighbour, int& right_neighbour) const;
            bool get_right_neighbours(const std::vector<Segment>& segments, const float line_position, const Vec2& event_point, std::vector<const Node*>& right_nodes) const;
            bool get_left_neighbours(const std::vector<Segment>& segments, const float line_position, const Vec2& event_point, std::vector<const Node*>& left_nodes) const;


        };

        Sweep_Line_Status_structure(float line_position) : line_position(line_position) {};

        void insert(const std::vector<Segment>& segments, const int new_segment, int& left_node, int& right_node);
        void remove(const std::vector<Segment>& segments, const int segment_to_remove, int& left_node, int& right_node );
        bool contains(const std::vector<Segment>& segments, const Segment* search_segment);
        
        void set_line_position(const float new_position) { line_position = new_position; };
        void swap_elements(const std::vector<Segment>& segments, int segment_index_1, int segment_index_2, int& left_segment, int& right_segment);
        
        void get_all_nodes_on(const std::vector<Segment> segments, const Vec2& event_point, std::vector<int>& intersections , std::vector<int>& bottom_segments, int& most_left_segment, int& most_right_segment);
        Node* get_node(const std::vector<Segment>& segments, const Vec2& event_point);


        std::unique_ptr<Node> root;

    private:
        //TODO: Segment* vector here

        float line_position;
        
        std::unique_ptr<Node> insert(std::unique_ptr<Node>&& node, const std::vector<Segment>& segments, const int new_segment,const Node*& added_node);
        std::unique_ptr<Node> add_to_subtree(std::unique_ptr<Node>&& root, const std::vector<Segment>& segments, const int new_segment, const Node*& added_node, const Node* parent);

        std::unique_ptr<Node> remove(std::unique_ptr<Node>&& node, const std::vector<Segment>& segments, const int segment_to_remove);
        std::unique_ptr<Node> remove_from_parent(std::unique_ptr<Node>&& parent, const std::vector<Segment>& segments, const int segment_to_remove);

        std::unique_ptr<Node> rotate_left(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_left_right(std::unique_ptr<Node>&& old_root);
        std::unique_ptr<Node> rotate_right_left(std::unique_ptr<Node>&& old_root);

        Node* find_node(const std::vector<Segment>& segments, int segment_index);
        //get node
        

    };

}