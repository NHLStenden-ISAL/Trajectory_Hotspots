#pragma once

namespace Segment_Intersection_Sweep_Line
{
    struct Intersection_Info;
}

class DCEL
{
public:

    class DCEL_Vertex;
    class DCEL_Half_Edge;
    class DCEL_Face;

    //Class representing a vertex of the DCEL.
    //The class points to one of the outgoing half-edges (with the vertex as its origin).
    class DCEL_Vertex
    {
    public:

        DCEL_Vertex() = default;

        explicit DCEL_Vertex(const Vec2& position) : position(position) {}

        DCEL_Vertex(const Vec2& position, DCEL_Half_Edge* incident_half_edge)
            : position(position), incident_half_edge(incident_half_edge)
        {
        }

        //Returns the incident (outgoing) half-edges in clockwise order
        std::vector<const DCEL_Half_Edge*> get_incident_half_edges() const;

        //Returns the incident (outgoing) half-edges in clockwise order
        std::vector<DCEL_Half_Edge*> get_incident_half_edges();

        //Search around the given vertex in counter-clockwise order for the first clockwise and counter-clockwise half-edges adjacent to the given half-edge (pointing outwards)
        void find_adjacent_half_edges(const DCEL::DCEL_Half_Edge* query_edge, DCEL::DCEL_Half_Edge* starting_half_edge, DCEL::DCEL_Half_Edge*& CW_half_edge, DCEL::DCEL_Half_Edge*& CCW_half_edge) const;

        //Reassigns all the origin pointers of the incident half-edges to this vertex
        void set_all_origins_to_this();

        Vec2 position;
        DCEL_Half_Edge* incident_half_edge = nullptr; //One of the half-edges with the vertex as its origin
    };

    //Class representing a half-edge of the DCEL.
    //Every half-edge has an accompanying twin half-edge.
    //It also stores the incident face that lies to the left of the half-edge 
    //and the next and previous half-edges in the chain that lies around the face.
    class DCEL_Half_Edge
    {
    public:

        DCEL_Half_Edge(DCEL_Vertex* origin, DCEL_Face* incident_face, DCEL_Half_Edge* twin, DCEL_Half_Edge* next, DCEL_Half_Edge* prev)
            : origin(origin), incident_face(incident_face), twin(twin), next(next), prev(prev)
        {
        }

        explicit DCEL_Half_Edge(DCEL_Vertex* origin) : origin(origin)
        {
        }

        DCEL_Half_Edge() = default;

        //The origin of this half-edges twin
        DCEL_Vertex* target() const { return twin->origin; };

        DCEL_Vertex* origin = nullptr;

        DCEL_Face* incident_face = nullptr;

        DCEL_Half_Edge* twin = nullptr;
        DCEL_Half_Edge* next = nullptr;
        DCEL_Half_Edge* prev = nullptr;

        //Returns if the half-edges origin lies to the right of its destination, 
        //if they share an x axis it returns true if its origin is below its destination.
        bool is_orientated_top_left() const;

        //Returns the cycle of half-edges pointing to each other, starting with this
        std::vector<DCEL_Half_Edge*> get_cycle();

        //Returns the cycle of half-edges pointing to each other, starting with this
        std::vector<const DCEL_Half_Edge*> get_cycle() const;
    };

    //Wrapper class used for the overlay of two DCELs. Each instance represents an edge that overlaps with two twin half-edges.
    class DCEL_Overlay_Edge_Wrapper
    {
    public:
        DCEL_Overlay_Edge_Wrapper(DCEL_Half_Edge* underlying_half_edge, bool original_dcel) :
            underlying_half_edge(underlying_half_edge),
            edge_segment(underlying_half_edge->origin->position, underlying_half_edge->twin->origin->position),
            original_dcel(original_dcel)
        {};

        //Returns the x-coordinate of the intersection with the horizontal line at y, or infinity if it lies on the segment
        Float y_intersect(Float y) const;

        const Vec2* get_top_point() const;
        const Vec2* get_bottom_point() const;
        const Vec2* get_left_point() const;
        const Vec2* get_right_point() const;

        DCEL_Vertex* get_top_dcel_vertex();
        DCEL_Vertex* get_bottom_dcel_vertex();

        Segment::Intersection_Type intersects(const DCEL_Overlay_Edge_Wrapper& other, Vec2& intersection_point) const;

        DCEL_Half_Edge* underlying_half_edge;
        Segment edge_segment;
        bool original_dcel;
    };


    //Class representing a face of the DCEL, surrounded by half-edges.
    //It stores a pointer to one of the half-edges on its boundary
    //and pointers to the outer boundary of any optional holes.
    class DCEL_Face
    {
    public:
        DCEL_Half_Edge* outer_component = nullptr;
        std::vector<DCEL_Half_Edge*> inner_components;

        std::vector<Vec2> get_vertices() const;
    };

    size_t vertex_count() const { return vertices.size(); };
    size_t half_edge_count() const { return half_edges.size(); };
    size_t face_count() const { return faces.size(); };

    void overlay_dcel(DCEL& other_dcel);

    void insert_segment(const Segment& segment);

    DCEL::DCEL_Half_Edge* create_free_segment_records(const Segment& segment);

    DCEL_Vertex* get_vertex_at_position(const Vec2& position);

    void clear()
    {
        vertices.clear();
        half_edges.clear();
        faces.clear();
    };

    std::vector<std::unique_ptr<DCEL_Vertex>> vertices;
    std::vector<std::unique_ptr<DCEL_Half_Edge>> half_edges;
    std::vector<std::unique_ptr<DCEL_Face>> faces;

private:

    void intersection_on_endpoint(const Vec2& intersection_point,
        const DCEL::DCEL_Half_Edge* old_half_edge,
        const DCEL::DCEL_Half_Edge* new_half_edge,
        DCEL_Vertex*& old_overlapping_vertex,
        DCEL_Vertex*& new_overlapping_vertex) const;

    void handle_point_intersection(const Vec2& intersection_point, DCEL::DCEL_Half_Edge* old_half_edge, DCEL::DCEL_Half_Edge* new_half_edge);

    void resolve_edge_intersections(std::vector<DCEL_Overlay_Edge_Wrapper>& DCEL_edges);

    void handle_overlay_event(std::vector<DCEL_Overlay_Edge_Wrapper>& DCEL_edges, Segment_Intersection_Sweep_Line::Intersection_Info& intersection_results, const Vec2& event_point);

    bool overlay_event_contains_both_dcels(const std::vector<DCEL_Overlay_Edge_Wrapper>& DCEL_edges, const Segment_Intersection_Sweep_Line::Intersection_Info& intersection_results) const;

    void overlay_edge_on_vertex(DCEL_Half_Edge* edge, DCEL_Vertex* vertex);
    DCEL_Vertex* overlay_edge_on_edge(DCEL_Half_Edge* edge_1, DCEL_Half_Edge* edge_2, const Vec2& intersection_point);
    void overlay_vertex_on_vertex(DCEL_Vertex* vertex_1, DCEL_Vertex* vertex_2)  const;

};

//Given two collinear segments, returns if they overlap and if true also provides the start and end points of the overlap.
bool collinear_overlap(const DCEL::DCEL_Overlay_Edge_Wrapper& segment1, const DCEL::DCEL_Overlay_Edge_Wrapper& segment2, Vec2& overlap_start, Vec2& overlap_end);